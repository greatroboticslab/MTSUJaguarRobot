"""
Enhanced Flask Robot Controller with MQTT Integration
Improvements:
  - Better error handling and connection recovery
  - Improved threading and daemon management
  - Enhanced navigation with better state tracking
  - Photo capture and management
  - System monitoring and health stats
  - Configurable parameters
"""

from flask import Flask, render_template, jsonify, request, Response
import paho.mqtt.client as mqtt
import json
import threading
import queue
import socket
import math
import time
import cv2
import os
from datetime import datetime
from collections import deque

app = Flask(__name__)

# Create photos directory
PHOTOS_DIR = 'captured_photos'
if not os.path.exists(PHOTOS_DIR):
    os.makedirs(PHOTOS_DIR)

# ============== Global Data Storage ==============
current_data = {
    'imu': {'heading': 0, 'accX': 0, 'accY': 0, 'accZ': 0},
    'gps': {'lat': 0, 'lon': 0, 'alt': 0},
    'fused': {'heading': 0, 'lat': 0, 'lon': 0},
    'system_health': {}
}

latest_frame = None
mqtt_received_data = deque(maxlen=50)
data_queue = queue.Queue(maxsize=100)
mqtt_connected = False

# Camera variables
camera_url = "http://root:drrobot@192.168.0.65:8081/axis-cgi/mjpg/video.cgi"
camera_cap = None
camera_running = False
camera_lock = threading.Lock()

# Navigation variables
navigation_active = False
navigation_thread = None
navigation_direction = 1
navigation_cycle_time = 20
navigation_command_interval = 0.5
navigation_max_speed = None
navigation_min_speed = 50

# Command rate limiting
last_command_time = 0
command_delay = 0.1
last_movement_time = 0
stop_timeout = 0.3
stop_thread = None
stop_thread_active = False

# PID parameters
class PIDParams:
    def __init__(self):
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        self.speed = 150

pid_params = PIDParams()

# ============== Fusion State ==============
class FusionState:
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
        self.heading_offset = 0.0
        self.last_gps_heading = None
        self.dist_same_dir = 0.0

fusion_state = FusionState()

GPS_NOISE_THRESHOLD = 2
STATIC_ACC_THRESHOLD = 100.0
DIRECTION_DIFF_MAX = 10.0
MIN_DISTANCE_CORRECT = 5.0
MAX_TRUST_DISTANCE = 100.0


def camera_stream():
    """Camera streaming thread"""
    global camera_cap, camera_running, latest_frame
    
    try:
        camera_cap = cv2.VideoCapture(camera_url)
        if not camera_cap.isOpened():
            print("✗ Failed to open camera URL")
            camera_running = False
            return
        
        camera_running = True
        while camera_running:
            ret, frame = camera_cap.read()
            if ret:
                with camera_lock:
                    latest_frame = frame
            else:
                print("✗ Camera frame read failed")
                break
            time.sleep(0.03)
    except Exception as e:
        print(f"✗ Camera stream error: {e}")
    finally:
        if camera_cap:
            camera_cap.release()
            camera_cap = None
        camera_running = False


def start_camera():
    """Start camera thread"""
    global camera_thread, camera_running
    
    if not camera_running:
        camera_thread = threading.Thread(target=camera_stream, daemon=True)
        camera_thread.start()
        print("✓ Camera started")


def stop_camera():
    """Stop camera thread"""
    global camera_running, camera_cap
    
    camera_running = False
    if camera_cap:
        camera_cap.release()
        camera_cap = None
    print("✓ Camera stopped")


# ============== MQTT Handlers ==============
def on_connect(client, userdata, flags, rc):
    global mqtt_connected
    
    if rc == 0:
        print("✓ MQTT connected")
        mqtt_connected = True
        client.subscribe([
            ("IMU/data", 0),
            ("GPS/data", 0),
            ("system/health", 0)
        ])
    else:
        mqtt_connected = False
        print(f"✗ MQTT connection failed: {rc}")


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        mqtt_received_data.append(data)
        
        # Queue data for processing if not full
        try:
            data_queue.put_nowait(data)
        except queue.Full:
            pass
    except Exception as e:
        print(f"✗ MQTT message error: {e}")


# ============== Data Fusion ==============
def distance_meters(lat1, lon1, lat2, lon2):
    """Haversine distance calculation"""
    R = 6371000.0
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = (math.sin(dLat/2)**2 + 
         math.cos(math.radians(lat1)) * 
         math.cos(math.radians(lat2)) * 
         math.sin(dLon/2)**2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c


def bearing_degs(lat1, lon1, lat2, lon2):
    """Calculate bearing in degrees"""
    dLat = lat2 - lat1
    dLon = lon2 - lon1
    
    if abs(dLat) > 1e-7 or abs(dLon) > 1e-7:
        bearing = math.degrees(math.atan2(math.radians(dLon), math.radians(dLat)))
        return (bearing + 360) % 360
    return 0.0


def process_data():
    """Process incoming MQTT data and fuse sensor readings"""
    while True:
        try:
            raw = data_queue.get(timeout=1)
            
            # Extract IMU data
            imu_heading = float(raw.get('Heading', 0.0))
            imu_accX = float(raw.get('accX', 0.0))
            imu_accY = float(raw.get('accY', 0.0))
            
            # Extract GPS data (with fallback)
            gps_lat = float(raw.get('gps:Lat', 0.0))
            gps_lon = float(raw.get('Lon', 0.0))
            
            now_t = time.time()
            dt = now_t - fusion_state.last_update_time
            fusion_state.last_update_time = now_t
            
            # Calculate distance and bearing
            dist = 0.0
            gps_heading = None
            
            if (fusion_state.last_gps_lat is not None and 
                fusion_state.last_gps_lon is not None):
                dist = distance_meters(
                    fusion_state.last_gps_lat, fusion_state.last_gps_lon,
                    gps_lat, gps_lon
                )
                dLat = gps_lat - fusion_state.last_gps_lat
                dLon = gps_lon - fusion_state.last_gps_lon
                
                if abs(dLat) > 1e-7 or abs(dLon) > 1e-7:
                    gps_heading = bearing_degs(
                        fusion_state.last_gps_lat, fusion_state.last_gps_lon,
                        gps_lat, gps_lon
                    )
            
            # Apply heading offset correction
            corrected_imu_heading = (imu_heading + fusion_state.heading_offset) % 360
            
            # Fuse heading: weight by distance
            alpha_imu = 0.4
            alpha_gps = 0.6
            
            if gps_heading is not None:
                if dist < 3.0:
                    alpha_imu = 0.6
                    alpha_gps = 0.4
                fused_heading = (alpha_imu * fusion_state.heading + 
                               alpha_gps * gps_heading)
            else:
                fused_heading = corrected_imu_heading
            
            # Fuse position
            alpha_pos_imu = 0.4
            alpha_pos_gps = 0.6
            lat_fused = alpha_pos_imu * fusion_state.lat + alpha_pos_gps * gps_lat
            lon_fused = alpha_pos_imu * fusion_state.lon + alpha_pos_gps * gps_lon
            
            # Detect static state
            if dist < GPS_NOISE_THRESHOLD or (abs(imu_accX) < STATIC_ACC_THRESHOLD and 
                                             abs(imu_accY) < STATIC_ACC_THRESHOLD):
                imu_accX = 0.0
                imu_accY = 0.0
            
            # Friction logic: detect if turning but not changing heading
            dHeading = abs(fused_heading - fusion_state.last_heading)
            if dHeading < 1.0 and dt > 0:
                fusion_state.turn_stuck_counter += 1
                if fusion_state.turn_stuck_counter > 5:
                    fusion_state.turn_override_speed = pid_params.speed + 50
            else:
                fusion_state.turn_stuck_counter = 0
                fusion_state.turn_override_speed = None
            
            # Dynamic heading offset correction
            if gps_heading is not None:
                if fusion_state.last_gps_heading is None:
                    fusion_state.last_gps_heading = gps_heading
                    fusion_state.dist_same_dir = 0.0
                else:
                    dir_diff = abs(gps_heading - fusion_state.last_gps_heading)
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
                        new_off = (fusion_state.heading_offset * (1 - ratio) + 
                                 offset_suggestion * ratio)
                        fusion_state.heading_offset = new_off
            
            # Update state
            fusion_state.last_gps_lat = gps_lat
            fusion_state.last_gps_lon = gps_lon
            fusion_state.last_heading = fused_heading
            fusion_state.lat = lat_fused
            fusion_state.lon = lon_fused
            fusion_state.heading = fused_heading
            
            # Update global current_data
            global current_data
            current_data = {
                'imu': {
                    'heading': round(imu_heading, 2),
                    'accX': round(imu_accX, 2),
                    'accY': round(imu_accY, 2)
                },
                'gps': {
                    'lat': round(gps_lat, 8),
                    'lon': round(gps_lon, 8)
                },
                'fused': {
                    'heading': round(fused_heading, 2),
                    'lat': round(lat_fused, 8),
                    'lon': round(lon_fused, 8)
                }
            }
        
        except queue.Empty:
            pass
        except Exception as e:
            print(f"✗ Data processing error: {e}")


# ============== MQTT Connection ==============
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_reconnect_delay = 1
mqtt_reconnect_max_delay = 60
mqtt_broker_ip = "192.168.1.103"
mqtt_broker_port = 1883


def mqtt_connect_with_retry():
    """MQTT connection with exponential backoff retry"""
    global mqtt_reconnect_delay, mqtt_connected
    
    try:
        print(f"Connecting to MQTT broker {mqtt_broker_ip}:{mqtt_broker_port}...")
        mqtt_client.connect(mqtt_broker_ip, mqtt_broker_port, 60)
        mqtt_reconnect_delay = 1
        mqtt_client.loop_forever()
    except Exception as e:
        print(f"✗ MQTT error: {e}, retry in {mqtt_reconnect_delay}s")
        time.sleep(mqtt_reconnect_delay)
        mqtt_reconnect_delay = min(mqtt_reconnect_delay * 2, mqtt_reconnect_max_delay)
        mqtt_thread = threading.Thread(target=mqtt_connect_with_retry, daemon=True)
        mqtt_thread.start()


# ============== Robot Socket ==============
class RobotSocket:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        self.lock = threading.Lock()

    def connect(self):
        with self.lock:
            if self.sock:
                try:
                    self.sock.close()
                except:
                    pass
            
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self.sock.connect((self.ip, self.port))
                print(f"✓ Connected to robot at {self.ip}:{self.port}")
                return True
            except Exception as e:
                print(f"✗ Robot connection error: {e}")
                self.sock = None
                return False

    def send_command(self, cmd):
        with self.lock:
            if not self.sock:
                print("✗ Robot socket disconnected")
                return None
            
            try:
                self.sock.sendall((cmd + "\r\n").encode())
                self.sock.settimeout(1.0)
                resp = self.sock.recv(1024).decode()
                return resp
            except socket.timeout:
                print(f"⚠ Timeout: {cmd}")
                return None
            except Exception as e:
                print(f"✗ Send error: {e}")
                self.sock = None
                self.connect()
                return None


robot = RobotSocket('192.168.0.60', 10001)

# ============== Movement Commands ==============
def robot_forward():
    """Forward movement"""
    robot.send_command("MMW !MG")
    speed = fusion_state.turn_override_speed if fusion_state.turn_override_speed else pid_params.speed
    robot.send_command(f"MMW !M {speed} -{speed}")


def robot_backward():
    """Backward movement"""
    robot.send_command("MMW !MG")
    speed = fusion_state.turn_override_speed if fusion_state.turn_override_speed else pid_params.speed
    robot.send_command(f"MMW !M -{speed} {speed}")


def robot_left():
    """Left turn"""
    robot.send_command("MMW !MG")
    speed = fusion_state.turn_override_speed if fusion_state.turn_override_speed else pid_params.speed
    robot.send_command(f"MMW !M -{speed} -{speed}")


def robot_right():
    """Right turn"""
    robot.send_command("MMW !MG")
    speed = fusion_state.turn_override_speed if fusion_state.turn_override_speed else pid_params.speed
    robot.send_command(f"MMW !M {speed} {speed}")


def robot_stop():
    """Stop movement"""
    robot.send_command("MMW !MG")
    robot.send_command("MMW !M 0 0")


def robot_stop_enhanced():
    """Enhanced stop with multiple commands"""
    for _ in range(3):
        robot.send_command("MMW !MG")
        time.sleep(0.02)
        robot.send_command("MMW !M 0 0")
        time.sleep(0.02)


# ============== Navigation ==============
def navigation_cycle_improved():
    """Improved autonomous navigation cycle"""
    global navigation_active, navigation_direction
    
    print("✓ Navigation started")
    
    while navigation_active:
        cycle_start = time.time()
        direction_name = "Forward" if navigation_direction == 1 else "Backward"
        
        try:
            if navigation_direction == 1:
                robot_forward()
            else:
                robot_backward()
            
            # Maintain movement
            last_cmd_time = time.time()
            while navigation_active and (time.time() - cycle_start) < navigation_cycle_time:
                if time.time() - last_cmd_time >= navigation_command_interval:
                    if navigation_direction == 1:
                        robot_forward()
                    else:
                        robot_backward()
                    last_cmd_time = time.time()
                
                time.sleep(0.1)
            
            if not navigation_active:
                break
            
            # Stop and switch direction
            robot_stop_enhanced()
            navigation_direction *= -1
            time.sleep(1.0)
        
        except Exception as e:
            print(f"✗ Navigation error: {e}")
            robot_stop_enhanced()
            time.sleep(1.0)
    
    print("✓ Navigation stopped")


def start_navigation_improved():
    """Start autonomous navigation"""
    global navigation_active, navigation_thread
    
    if navigation_active:
        return False
    
    if not robot.sock and not robot.connect():
        return False
    
    print("✓ Starting navigation")
    navigation_active = True
    navigation_thread = threading.Thread(target=navigation_cycle_improved, daemon=True)
    navigation_thread.start()
    return True


def stop_navigation_improved():
    """Stop autonomous navigation"""
    global navigation_active
    
    if not navigation_active:
        return False
    
    navigation_active = False
    robot_stop_enhanced()
    print("✓ Navigation stopped")
    return True


# ============== Flask Routes ==============
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/data')
def get_data():
    return jsonify(current_data)


@app.route('/mqtt-status')
def mqtt_status():
    return jsonify({'connected': mqtt_connected})


@app.route('/connect-robot', methods=['POST'])
def connect_robot():
    ok = robot.connect()
    return jsonify({'success': ok})


@app.route('/robot-cmd', methods=['POST'])
def robot_cmd():
    global last_command_time, last_movement_time
    
    current_time = time.time()
    if current_time - last_command_time < command_delay:
        return jsonify({'success': True, 'throttled': True})
    
    last_command_time = current_time
    
    data = request.json
    cmd = data.get('cmd', '').lower()
    
    if cmd in ['w', 's', 'a', 'd']:
        last_movement_time = current_time
    
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
        last_movement_time = 0
    elif cmd == 'n':
        if navigation_active:
            success = stop_navigation_improved()
        else:
            success = start_navigation_improved()
        return jsonify({'success': success, 'navigation_active': navigation_active})
    
    return jsonify({'success': True, 'throttled': False})


@app.route('/navigation-status')
def get_navigation_status():
    return jsonify({
        'active': navigation_active,
        'direction': 'forward' if navigation_direction == 1 else 'backward'
    })


@app.route('/pid-update', methods=['POST'])
def pid_update():
    global navigation_cycle_time
    
    data = request.get_json() or {}
    try:
        pid_params.kp = float(data.get('kp', pid_params.kp))
        pid_params.ki = float(data.get('ki', pid_params.ki))
        pid_params.kd = float(data.get('kd', pid_params.kd))
        pid_params.speed = int(data.get('speed', pid_params.speed))
        if 'nav' in data:
            navigation_cycle_time = max(1, float(data.get('nav')))
    except Exception as e:
        print(f"✗ PID update error: {e}")
    
    return jsonify({
        'kp': pid_params.kp,
        'ki': pid_params.ki,
        'kd': pid_params.kd,
        'speed': pid_params.speed,
        'nav': navigation_cycle_time
    })


@app.route('/camera-stream')
def camera_feed():
    """Video stream endpoint"""
    def generate():
        while True:
            with camera_lock:
                if latest_frame is not None:
                    ret, buffer = cv2.imencode('.jpg', latest_frame)
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.03)
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


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
    
    return jsonify({'success': False})


@app.route('/capture-photo', methods=['POST'])
def capture_photo():
    """Capture photo from camera"""
    with camera_lock:
        if latest_frame is None:
            return jsonify({'success': False, 'error': 'No frame available'})
        
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'photo_{timestamp}.jpg'
            filepath = os.path.join(PHOTOS_DIR, filename)
            
            if cv2.imwrite(filepath, latest_frame):
                file_size = os.path.getsize(filepath)
                return jsonify({
                    'success': True,
                    'filename': filename,
                    'size': file_size,
                    'timestamp': timestamp
                })
            return jsonify({'success': False, 'error': 'Save failed'})
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)})


@app.route('/photos', methods=['GET'])
def list_photos():
    """List captured photos"""
    try:
        photos = []
        if os.path.exists(PHOTOS_DIR):
            for filename in sorted(os.listdir(PHOTOS_DIR), reverse=True):
                if filename.endswith('.jpg'):
                    filepath = os.path.join(PHOTOS_DIR, filename)
                    file_size = os.path.getsize(filepath)
                    file_time = os.path.getmtime(filepath)
                    photos.append({
                        'filename': filename,
                        'size': file_size,
                        'timestamp': datetime.fromtimestamp(file_time).strftime('%Y-%m-%d %H:%M:%S')
                    })
        
        return jsonify({'success': True, 'photos': photos, 'count': len(photos)})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/delete-photo/<filename>', methods=['DELETE'])
def delete_photo(filename):
    """Delete a photo"""
    try:
        if '..' in filename or '/' in filename or '\\' in filename:
            return jsonify({'success': False, 'error': 'Invalid filename'})
        
        filepath = os.path.join(PHOTOS_DIR, filename)
        if os.path.exists(filepath) and filename.endswith('.jpg'):
            os.remove(filepath)
            return jsonify({'success': True})
        return jsonify({'success': False, 'error': 'Not found'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


if __name__ == '__main__':
    # Start background threads
    process_thread = threading.Thread(target=process_data, daemon=True)
    process_thread.start()
    
    mqtt_thread = threading.Thread(target=mqtt_connect_with_retry, daemon=True)
    mqtt_thread.start()
    
    print("""
    ╔═══════════════════════════════════════╗
    ║  Flask Robot Controller v2.2 Started  ║
    ║  http://0.0.0.0:5014                  ║
    ╚═══════════════════════════════════════╝
    """)
    
    app.run(host='0.0.0.0', port=5014, debug=False, threaded=True)
