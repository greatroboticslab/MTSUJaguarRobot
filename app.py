from flask import Flask, render_template, jsonify, request, Response
import paho.mqtt.client as mqtt
import json
import threading
import queue
import socket
import math
import time
import cv2

app = Flask(__name__)

# ------------------ Global Storage for Data ------------------ #
current_data = {
    'imu': None,         # raw IMU
    'gps': None,         # raw GPS
    'fused': None,       # fused/filtered navigation data
    'camera': None,      # camera tracking data
    'weed_detections': [],  # weed detection data
    'laser': {           # laser status and parameters
        'status': 'OFF',
        'power': 2,      # Changed from 5 to 2 for safety
        'aim_power': 2,
        'duration': 5,
        'safety_delay': 2,
        'mode': 'MANUAL'  # 'MANUAL' or 'AUTO'
    }
}

# Latest camera frame
latest_frame = None
mqtt_received_data = []  # Stores raw MQTT data
data_queue = queue.Queue()
mqtt_connected = False  # Track MQTT connection status

# 摄像头相关变量
camera_url = "http://root:drrobot@192.168.0.65:8081/axis-cgi/mjpg/video.cgi"
camera_cap = None
camera_thread = None
camera_running = False

# Navigation mode variables
navigation_active = False
navigation_thread = None
navigation_direction = 1  # 1 for forward, -1 for backward
navigation_cycle_time = 3  # seconds per direction

def camera_stream():
    global camera_cap, camera_running, latest_frame
    camera_cap = cv2.VideoCapture(camera_url)
    camera_running = True
    
    while camera_running:
        ret, frame = camera_cap.read()
        if ret:
            latest_frame = frame
        time.sleep(0.03)  # 约30fps
    
    if camera_cap:
        camera_cap.release()

def start_camera():
    global camera_thread
    if camera_thread is None or not camera_thread.is_alive():
        camera_thread = threading.Thread(target=camera_stream, daemon=True)
        camera_thread.start()

def stop_camera():
    global camera_running, camera_cap
    camera_running = False
    if camera_cap:
        camera_cap.release()
        camera_cap = None

# Simple PID parameter storage
class PIDParams:
    def __init__(self):
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        self.speed = 150  # default speed

pid_params = PIDParams()

# ------------------ Extended FusionState ------------------ #
class FusionState:
    """
    A lightweight structure to track fused heading and lat/lon.
    Weighted approach:
      - Weighted heading (GPS-based vs IMU-based)
      - Weighted position (GPS vs short-range IMU)
      - Basic friction logic: if rotating but heading not changing => up speed
      - Supports static detection if movement < threshold => zero out acc
      - Dynamic offset correction if traveling > 5m in same direction => heading_offset
    """
    def __init__(self):
        self.lat = 0.0
        self.lon = 0.0
        self.heading = 0.0

        self.last_gps_lat = None
        self.last_gps_lon = None
        self.last_update_time = time.time()
        self.last_heading = 0.0

        self.turn_stuck_counter = 0
        self.turn_override_speed = None

        # IMU accel bias if needed
        self.imu_accX_bias = 0.0
        self.imu_accY_bias = 0.0

        # heading offset for dynamic correction
        self.heading_offset = 0.0

        # For dynamic correction
        self.last_gps_heading = None
        self.dist_same_dir = 0.0

fusion_state = FusionState()

# thresholds
GPS_NOISE_THRESHOLD   = 2       # if distance <2 => consider ~static
STATIC_ACC_THRESHOLD  = 100.0   # if |accX|,|accY|<100 => consider static
DIRECTION_DIFF_MAX    = 10.0    # if gps heading differs <10°, consider same direction
MIN_DISTANCE_CORRECT  = 5.0     # 5m => begin offset correction
MAX_TRUST_DISTANCE    = 100.0   # up to 100m => 100% trust

def on_connect(client, userdata, flags, rc):
    global mqtt_connected
    if rc == 0:
        print("Connected to MQTT broker with result code", rc)
        mqtt_connected = True
        client.subscribe([
            ("IMU/data", 0),
            ("CAMERA/tracking", 0),
            ("camera/detections", 0),
            ("camera/frame", 0)  # Subscribe to camera frames
        ])
    else:
        print(f"Failed to connect to MQTT broker with result code {rc}")
        mqtt_connected = False

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        mqtt_received_data.append(data)
        if len(mqtt_received_data) > 40:
            mqtt_received_data.pop(0)
        data_queue.put(data)
    except Exception as e:
        print(f"Error processing message: {e}")

def distance_meters(lat1, lon1, lat2, lon2):
    # basic haversine
    R = 6371000.0
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = math.sin(dLat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dLon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def bearing_degs(lat1, lon1, lat2, lon2):
    dLat = lat2 - lat1
    dLon = lon2 - lon1
    if abs(dLat) > 1e-7 or abs(dLon) > 1e-7:
        # original approach
        bearing = math.degrees(math.atan2(math.radians(dLon), math.radians(dLat)))
        return (bearing + 360) % 360
    else:
        return 0.0

def process_data():
    while True:
        try:
            raw = data_queue.get()
            imu_heading = float(raw.get('Heading', 0.0))
            imu_accX    = float(raw.get('accX', 0.0))
            imu_accY    = float(raw.get('accY', 0.0))
            gps_lat     = float(raw.get('gps:Lat', 0.0))
            gps_lon     = float(raw.get('Lon', 0.0))

            new_imu = {
                'heading': imu_heading,
                'accX': imu_accX,
                'accY': imu_accY
            }
            new_gps = {
                'lat': gps_lat,
                'lon': gps_lon
            }

            now_t = time.time()
            dt = now_t - fusion_state.last_update_time
            fusion_state.last_update_time = now_t

            dist = 0.0
            gps_heading = None
            if fusion_state.last_gps_lat is not None and fusion_state.last_gps_lon is not None:
                dist = distance_meters(fusion_state.last_gps_lat, fusion_state.last_gps_lon, gps_lat, gps_lon)
                dLat = gps_lat - fusion_state.last_gps_lat
                dLon = gps_lon - fusion_state.last_gps_lon
                if abs(dLat) > 1e-7 or abs(dLon) > 1e-7:
                    gps_heading = bearing_degs(fusion_state.last_gps_lat, fusion_state.last_gps_lon, gps_lat, gps_lon)

            # apply heading_offset
            corrected_imu_heading = (imu_heading + fusion_state.heading_offset) % 360

            # Weighted heading
            alpha_imu = 0.4
            alpha_gps = 0.6
            fused_heading = corrected_imu_heading
            if gps_heading is not None:
                if dist < 3.0:
                    alpha_imu = 0.6
                    alpha_gps = 0.4
                fused_heading = alpha_imu * fusion_state.heading + alpha_gps * gps_heading
            else:
                fused_heading = corrected_imu_heading

            # Weighted lat/lon
            alpha_pos_imu = 0.4
            alpha_pos_gps = 0.6
            lat_fused = alpha_pos_imu * fusion_state.lat + alpha_pos_gps * gps_lat
            lon_fused = alpha_pos_imu * fusion_state.lon + alpha_pos_gps * gps_lon

            # If "not moving", set acc=0
            if dist < GPS_NOISE_THRESHOLD or (abs(imu_accX) < STATIC_ACC_THRESHOLD and abs(imu_accY) < STATIC_ACC_THRESHOLD):
                imu_accX = 0.0
                imu_accY = 0.0

            # friction logic
            dHeading = abs(fused_heading - fusion_state.last_heading)
            if dHeading < 1.0 and dt > 0:
                fusion_state.turn_stuck_counter += 1
                if fusion_state.turn_stuck_counter > 5:
                    fusion_state.turn_override_speed = pid_params.speed + 50
            else:
                fusion_state.turn_stuck_counter = 0
                fusion_state.turn_override_speed = None

            # dynamic offset correction
            if gps_heading is not None:
                if fusion_state.last_gps_heading is None:
                    fusion_state.last_gps_heading = gps_heading
                    fusion_state.dist_same_dir = 0.0
                else:
                    dir_diff = abs(gps_heading - fusion_state.last_gps_heading)
                    # normalize to [-180..180]
                    dir_diff = ((dir_diff + 180) % 360) - 180
                    if abs(dir_diff) < DIRECTION_DIFF_MAX:
                        fusion_state.dist_same_dir += dist
                    else:
                        fusion_state.dist_same_dir = 0.0
                        fusion_state.last_gps_heading = gps_heading

                    if fusion_state.dist_same_dir > MIN_DISTANCE_CORRECT:
                        offset_suggestion = gps_heading - imu_heading
                        offset_suggestion = ((offset_suggestion + 180) % 360) - 180
                        trust = min(fusion_state.dist_same_dir, MAX_TRUST_DISTANCE)
                        ratio = trust / MAX_TRUST_DISTANCE
                        old_off = fusion_state.heading_offset
                        new_off = old_off * (1 - ratio) + offset_suggestion * ratio
                        fusion_state.heading_offset = new_off

            fusion_state.last_gps_lat = gps_lat
            fusion_state.last_gps_lon = gps_lon
            fusion_state.last_heading = fused_heading

            fusion_state.lat = lat_fused
            fusion_state.lon = lon_fused
            fusion_state.heading = fused_heading

            fused_dict = {
                'lat': fusion_state.lat,
                'lon': fusion_state.lon,
                'heading': fusion_state.heading
            }
            global current_data
            current_data = {
                'imu': {
                    'heading': imu_heading,
                    'accX': imu_accX,
                    'accY': imu_accY
                },
                'gps': {
                    'lat': gps_lat,
                    'lon': gps_lon
                },
                'fused': fused_dict
            }

        except Exception as e:
            print("Error in data processing:", e)

# Initialize MQTT client
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_reconnect_delay = 1  # Start with 1 second delay
mqtt_reconnect_max_delay = 60  # Maximum delay of 60 seconds
mqtt_broker_ip = "192.168.1.103"
mqtt_broker_port = 1883

def mqtt_connect_with_retry():
    global mqtt_reconnect_delay, mqtt_connected
    
    def on_mqtt_connect_fail(client, userdata, flags, rc):
        global mqtt_reconnect_delay
        print(f"MQTT connection failed with code {rc}, retrying in {mqtt_reconnect_delay} seconds...")
        time.sleep(mqtt_reconnect_delay)
        # Exponential backoff with cap
        mqtt_reconnect_delay = min(mqtt_reconnect_delay * 2, mqtt_reconnect_max_delay)
        mqtt_thread = threading.Thread(target=mqtt_connect_with_retry, daemon=True)
        mqtt_thread.start()
    
    try:
        # Set temporary failure callback
        mqtt_client.on_connect = on_mqtt_connect_fail
        mqtt_client.connect(mqtt_broker_ip, mqtt_broker_port, 60)
        
        # If we get here, connection succeeded, restore normal callback
        mqtt_client.on_connect = on_connect
        print(f"Connected to MQTT broker at {mqtt_broker_ip}:{mqtt_broker_port}")
        mqtt_reconnect_delay = 1  # Reset delay on successful connection
        mqtt_connected = True
        mqtt_client.loop_forever()
    except Exception as e:
        print(f"MQTT connection error: {e}, retrying in {mqtt_reconnect_delay} seconds...")
        time.sleep(mqtt_reconnect_delay)
        # Exponential backoff with cap
        mqtt_reconnect_delay = min(mqtt_reconnect_delay * 2, mqtt_reconnect_max_delay)
        mqtt_thread = threading.Thread(target=mqtt_connect_with_retry, daemon=True)
        mqtt_thread.start()

# Start the data processing thread
process_thread = threading.Thread(target=process_data, daemon=True)
process_thread.start()

# Start the MQTT client with retry capability
mqtt_thread = threading.Thread(target=mqtt_connect_with_retry, daemon=True)
mqtt_thread.start()

# ------------------ Robot Socket Class & Movement Commands ------------------ #
class RobotSocket:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None

    def connect(self):
        if self.sock:
            self.sock.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.ip, self.port))
            print(f"Connected to robot at {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"Error connecting to robot: {e}")
            self.sock = None
            return False

    def send_command(self, cmd):
        if not self.sock:
            print("Robot socket is not connected. Attempting to reconnect...")
            if not self.connect():
                return None
        
        try:
            self.sock.sendall((cmd + "\r\n").encode())
            # Set a timeout for receiving response
            self.sock.settimeout(1.0)
            resp = self.sock.recv(1024).decode()
            return resp
        except socket.timeout:
            print(f"Timeout sending command: {cmd}")
            return None
        except Exception as e:
            print(f"Error sending command '{cmd}': {e}")
            # Try to reconnect on error
            self.sock = None
            if self.connect():
                try:
                    self.sock.sendall((cmd + "\r\n").encode())
                    self.sock.settimeout(1.0)
                    resp = self.sock.recv(1024).decode()
                    return resp
                except Exception as e2:
                    print(f"Retry failed for command '{cmd}': {e2}")
            return None

robot = RobotSocket('192.168.0.60', 10001)

def robot_forward():
    # Simplified command sequence
    robot.send_command("MMW !MG")
    robot.send_command(f"MMW !M {pid_params.speed} -{pid_params.speed}")

def robot_backward():
    robot.send_command("MMW !MG")
    robot.send_command(f"MMW !M -{pid_params.speed} {pid_params.speed}")

def robot_left():
    turn_speed = fusion_state.turn_override_speed if fusion_state.turn_override_speed else pid_params.speed
    robot.send_command("MMW !MG")
    robot.send_command(f"MMW !M -{turn_speed} -{turn_speed}")

def robot_right():
    turn_speed = fusion_state.turn_override_speed if fusion_state.turn_override_speed else pid_params.speed
    robot.send_command("MMW !MG")
    robot.send_command(f"MMW !M {turn_speed} {turn_speed}")

def robot_stop():
    robot.send_command("MMW !MG")
    robot.send_command("MMW !M 0 0")

# Navigation functions
def navigation_cycle():
    """Simple back-and-forth navigation cycle"""
    global navigation_active, navigation_direction
    
    while navigation_active:
        start_time = time.time()
        
        if navigation_direction == 1:
            print("Navigation: Moving Forward")
            robot_forward()
        else:
            print("Navigation: Moving Backward")
            robot_backward()
        
        # Keep moving for the full cycle time, checking connection periodically
        while navigation_active and (time.time() - start_time) < navigation_cycle_time:
            time.sleep(0.5)  # Check every 0.5 seconds
            # Re-send movement command to maintain motion
            if navigation_direction == 1:
                robot.send_command(f"MMW !M {pid_params.speed} -{pid_params.speed}")
            else:
                robot.send_command(f"MMW !M -{pid_params.speed} {pid_params.speed}")
        
        if not navigation_active:
            break
            
        # Stop and pause between direction changes
        robot_stop()
        time.sleep(0.5)
        
        # Switch direction
        navigation_direction *= -1
        print(f"Navigation: Switching direction to {'Forward' if navigation_direction == 1 else 'Backward'}")

def start_navigation():
    global navigation_active, navigation_thread
    if not navigation_active:
        navigation_active = True
        navigation_thread = threading.Thread(target=navigation_cycle, daemon=True)
        navigation_thread.start()
        print("Navigation mode started")

def stop_navigation():
    global navigation_active
    navigation_active = False
    robot_stop()
    print("Navigation mode stopped")

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/data')
def get_data():
    return jsonify(current_data)

@app.route('/mqtt-data')
def get_mqtt_data():
    return jsonify(mqtt_received_data)

@app.route('/connect-robot', methods=['POST'])
def connect_robot():
    ok = robot.connect()
    return jsonify({'success': ok})

@app.route('/robot-cmd', methods=['POST'])
def robot_cmd():
    data = request.json
    cmd = data.get('cmd', '').lower()
    if cmd == 'w':
        robot_forward()
    elif cmd == 's':
        robot_backward()
    elif cmd == 'a':
        robot_left()
    elif cmd == 'd':
        robot_right()
    elif cmd == 'stop':
        robot_stop()
    elif cmd == 'n':  # Navigate command
        if navigation_active:
            stop_navigation()
        else:
            start_navigation()
    else:
        print("Unknown command:", cmd)
    return jsonify({'success': True})

@app.route('/navigation-status')
def get_navigation_status():
    return jsonify({
        'active': navigation_active,
        'direction': 'forward' if navigation_direction == 1 else 'backward',
        'cycle_time': navigation_cycle_time
    })

@app.route('/robot-autopilot', methods=['POST'])
def robot_autopilot():
    data = request.json
    action = data.get('action', '')
    speed = data.get('speed', pid_params.speed)
    actual_speed = fusion_state.turn_override_speed if fusion_state.turn_override_speed else speed
    if action == 'rotateLeft':
        robot.send_command("SYS CAL")
        robot.send_command("MMW !MG")
        robot.send_command(f"MMW !M -{actual_speed} -{actual_speed}")
    elif action == 'rotateRight':
        robot.send_command("SYS CAL")
        robot.send_command("MMW !MG")
        robot.send_command(f"MMW !M {actual_speed} {actual_speed}")
    elif action == 'forward':
        robot_forward()
    elif action == 'stop':
        robot_stop()
    return jsonify({'success': True})

@app.route('/pid-update', methods=['POST'])
def pid_update():
    data = request.get_json() or {}
    try:
        pid_params.kp = float(data.get('kp', pid_params.kp))
        pid_params.ki = float(data.get('ki', pid_params.ki))
        pid_params.kd = float(data.get('kd', pid_params.kd))
        pid_params.speed = int(data.get('speed', pid_params.speed))
    except Exception as e:
        print("Error updating PID:", e)
    return jsonify({
        "kp": pid_params.kp,
        "ki": pid_params.ki,
        "kd": pid_params.kd,
        "speed": pid_params.speed
    })

@app.route('/camera')
def camera_view():
    return render_template('camera.html')

@app.route('/camera-stream')
def camera_feed():
    def generate():
        while True:
            if latest_frame is not None:
                ret, buffer = cv2.imencode('.jpg', latest_frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.03)

    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/toggle-camera', methods=['POST'])
def toggle_camera():
    data = request.json
    action = data.get('action', '')
    
    if action == 'start':
        start_camera()
        return jsonify({'success': True, 'status': 'started'})
    elif action == 'stop':
        stop_camera()
        return jsonify({'success': True, 'status': 'stopped'})
    
    return jsonify({'success': False, 'error': 'Invalid action'})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5014, debug=True)
