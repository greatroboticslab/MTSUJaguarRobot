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
import csv
from datetime import datetime
from pathlib import Path

app = Flask(__name__)

# Create photos directory if it doesn't exist
PHOTOS_DIR = 'captured_photos'
if not os.path.exists(PHOTOS_DIR):
    os.makedirs(PHOTOS_DIR)

# Create sensor logs directory on Desktop
DESKTOP_PATH = str(Path.home() / 'Desktop')
SENSOR_LOGS_DIR = os.path.join(DESKTOP_PATH, 'sensor_logs')
if not os.path.exists(SENSOR_LOGS_DIR):
    os.makedirs(SENSOR_LOGS_DIR)

# Create CSV files for logging
SENSOR_LOG_FILE = os.path.join(SENSOR_LOGS_DIR, f'sensor_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
CSV_HEADERS = ['timestamp', 'source', 'latitude', 'longitude', 'altitude', 'heading', 'roll', 'pitch',
               'velocity', 'est_x', 'est_y', 'est_z', 'fix_type', 'satellites', 'accel_x', 'accel_y', 'accel_z']

# Initialize CSV file with headers
with open(SENSOR_LOG_FILE, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(CSV_HEADERS)

print(f"Sensor logs will be saved to: {SENSOR_LOG_FILE}")

# Global sensor data storage
sensor_data_lock = threading.Lock()
sensor_data = {
    'position': None,  # From Pi tracker
    'imu': None,       # Raw IMU data
    'gps': None,       # Raw GPS data
    'fused': None      # Fused navigation data
}

# Original global storage
current_data = {
    'imu': None,
    'gps': None,
    'fused': None,
    'camera': None,
    'weed_detections': [],
    'laser': {
        'status': 'OFF',
        'power': 2,
        'aim_power': 2,
        'duration': 5,
        'safety_delay': 2,
        'mode': 'MANUAL'
    }
}

latest_frame = None
mqtt_received_data = []
data_queue = queue.Queue()
mqtt_connected = False

camera_url = "http://root:drrobot@192.168.0.65:8081/axis-cgi/mjpg/video.cgi"
camera_cap = None
camera_thread = None
camera_running = False

navigation_active = False
navigation_thread = None
navigation_direction = 1
navigation_cycle_time = 20
navigation_command_interval = 0.5
navigation_ramp_time = 1.0
navigation_max_speed = None
navigation_min_speed = 50

last_command_time = 0
command_delay = 0.1

last_movement_time = 0
stop_timeout = 0.3
stop_thread = None
stop_thread_active = False

# -------------------- SENSOR LOGGING FUNCTIONS -------------------- #

def log_sensor_data(data_dict):
    """Log sensor data to CSV file"""
    try:
        with open(SENSOR_LOG_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            
            row = [
                data_dict.get('timestamp', datetime.now().isoformat()),
                data_dict.get('source', 'unknown'),
                data_dict.get('latitude', ''),
                data_dict.get('longitude', ''),
                data_dict.get('altitude', ''),
                data_dict.get('heading', ''),
                data_dict.get('roll', ''),
                data_dict.get('pitch', ''),
                data_dict.get('velocity', ''),
                data_dict.get('est_x', ''),
                data_dict.get('est_y', ''),
                data_dict.get('est_z', ''),
                data_dict.get('fix_type', ''),
                data_dict.get('satellites', ''),
                data_dict.get('accel_x', ''),
                data_dict.get('accel_y', ''),
                data_dict.get('accel_z', '')
            ]
            writer.writerow(row)
    except Exception as e:
        print(f"Error logging sensor data: {e}")

def camera_stream():
    global camera_cap, camera_running, latest_frame
    camera_cap = cv2.VideoCapture(camera_url)
    camera_running = True
    
    while camera_running:
        ret, frame = camera_cap.read()
        if ret:
            latest_frame = frame
        time.sleep(0.03)
    
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

class PIDParams:
    def __init__(self):
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        self.speed = 150

pid_params = PIDParams()

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
        self.imu_accX_bias = 0.0
        self.imu_accY_bias = 0.0
        self.heading_offset = 0.0
        self.last_gps_heading = None
        self.dist_same_dir = 0.0

fusion_state = FusionState()

GPS_NOISE_THRESHOLD = 2
STATIC_ACC_THRESHOLD = 100.0
DIRECTION_DIFF_MAX = 10.0
MIN_DISTANCE_CORRECT = 5.0
MAX_TRUST_DISTANCE = 100.0

# -------------------- MQTT CALLBACKS -------------------- #

def on_connect(client, userdata, flags, rc):
    global mqtt_connected
    if rc == 0:
        print("Connected to MQTT broker with result code", rc)
        mqtt_connected = True
        client.subscribe([
            ("jaguar/position", 0),  # Subscribe to Pi position tracker
            ("jaguar/imu", 0),       # Subscribe to Pi IMU data
            ("jaguar/gps", 0),       # Subscribe to Pi GPS data
            ("IMU/data", 0),
            ("CAMERA/tracking", 0),
            ("camera/detections", 0),
            ("camera/frame", 0)
        ])
        print("Subscribed to sensor topics")
    else:
        print(f"Failed to connect to MQTT broker with result code {rc}")
        mqtt_connected = False

def on_message(client, userdata, msg):
    global sensor_data, current_data
    
    try:
        data = json.loads(msg.payload.decode())
        mqtt_received_data.append(data)
        if len(mqtt_received_data) > 40:
            mqtt_received_data.pop(0)
        
        # Route data based on topic
        topic = msg.topic
        
        with sensor_data_lock:
            if topic == "jaguar/position":
                # Position data from Pi tracker
                sensor_data['position'] = data
                log_entry = {
                    'timestamp': data.get('t', datetime.now().isoformat()),
                    'source': 'pi_tracker',
                    'latitude': data.get('lat'),
                    'longitude': data.get('lon'),
                    'altitude': data.get('alt'),
                    'heading': data.get('h'),
                    'velocity': data.get('v'),
                    'est_x': data.get('x'),
                    'est_y': data.get('y'),
                    'fix_type': data.get('fix'),
                    'satellites': data.get('sats')
                }
                log_sensor_data(log_entry)
                print(f"Position update: X={data.get('x'):.2f}m Y={data.get('y'):.2f}m")
            
            elif topic == "jaguar/imu":
                # IMU data from Pi
                sensor_data['imu'] = data
                log_entry = {
                    'timestamp': data.get('timestamp', datetime.now().isoformat()),
                    'source': 'pi_imu',
                    'heading': data.get('euler', {}).get('heading'),
                    'roll': data.get('euler', {}).get('roll'),
                    'pitch': data.get('euler', {}).get('pitch'),
                    'accel_x': data.get('accel', {}).get('x'),
                    'accel_y': data.get('accel', {}).get('y'),
                    'accel_z': data.get('accel', {}).get('z')
                }
                log_sensor_data(log_entry)
            
            elif topic == "jaguar/gps":
                # GPS data from Pi
                sensor_data['gps'] = data
                log_entry = {
                    'timestamp': data.get('timestamp', datetime.now().isoformat()),
                    'source': 'pi_gps',
                    'latitude': data.get('latitude'),
                    'longitude': data.get('longitude'),
                    'altitude': data.get('altitude'),
                    'fix_type': data.get('fix_type'),
                    'satellites': data.get('satellites')
                }
                log_sensor_data(log_entry)
        
        # Keep data_queue for legacy processing
        data_queue.put(data)
        
    except Exception as e:
        print(f"Error processing message: {e}")

def distance_meters(lat1, lon1, lat2, lon2):
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
        bearing = math.degrees(math.atan2(math.radians(dLon), math.radians(dLat)))
        return (bearing + 360) % 360
    else:
        return 0.0

def process_data():
    while True:
        try:
            raw = data_queue.get()
            imu_heading = float(raw.get('Heading', 0.0))
            imu_accX = float(raw.get('accX', 0.0))
            imu_accY = float(raw.get('accY', 0.0))
            gps_lat = float(raw.get('gps:Lat', 0.0))
            gps_lon = float(raw.get('Lon', 0.0))

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

            corrected_imu_heading = (imu_heading + fusion_state.heading_offset) % 360

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

            alpha_pos_imu = 0.4
            alpha_pos_gps = 0.6
            lat_fused = alpha_pos_imu * fusion_state.lat + alpha_pos_gps * gps_lat
            lon_fused = alpha_pos_imu * fusion_state.lon + alpha_pos_gps * gps_lon

            if dist < GPS_NOISE_THRESHOLD or (abs(imu_accX) < STATIC_ACC_THRESHOLD and abs(imu_accY) < STATIC_ACC_THRESHOLD):
                imu_accX = 0.0
                imu_accY = 0.0

            dHeading = abs(fused_heading - fusion_state.last_heading)
            if dHeading < 1.0 and dt > 0:
                fusion_state.turn_stuck_counter += 1
                if fusion_state.turn_stuck_counter > 5:
                    fusion_state.turn_override_speed = pid_params.speed + 50
            else:
                fusion_state.turn_stuck_counter = 0
                fusion_state.turn_override_speed = None

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
                'imu': new_imu,
                'gps': new_gps,
                'fused': fused_dict
            }

        except Exception as e:
            print("Error in data processing:", e)

mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_reconnect_delay = 1
mqtt_reconnect_max_delay = 60
mqtt_broker_ip = "192.168.1.103"
mqtt_broker_port = 1883

def mqtt_connect_with_retry():
    global mqtt_reconnect_delay, mqtt_connected
    
    def on_mqtt_connect_fail(client, userdata, flags, rc):
        global mqtt_reconnect_delay
        print(f"MQTT connection failed with code {rc}, retrying in {mqtt_reconnect_delay} seconds...")
        time.sleep(mqtt_reconnect_delay)
        mqtt_reconnect_delay = min(mqtt_reconnect_delay * 2, mqtt_reconnect_max_delay)
        mqtt_thread = threading.Thread(target=mqtt_connect_with_retry, daemon=True)
        mqtt_thread.start()
    
    try:
        mqtt_client.on_connect = on_mqtt_connect_fail
        mqtt_client.connect(mqtt_broker_ip, mqtt_broker_port, 60)
        mqtt_client.on_connect = on_connect
        print(f"Connected to MQTT broker at {mqtt_broker_ip}:{mqtt_broker_port}")
        mqtt_reconnect_delay = 1
        mqtt_connected = True
        mqtt_client.loop_forever()
    except Exception as e:
        print(f"MQTT connection error: {e}, retrying in {mqtt_reconnect_delay} seconds...")
        time.sleep(mqtt_reconnect_delay)
        mqtt_reconnect_delay = min(mqtt_reconnect_delay * 2, mqtt_reconnect_max_delay)
        mqtt_thread = threading.Thread(target=mqtt_connect_with_retry, daemon=True)
        mqtt_thread.start()

process_thread = threading.Thread(target=process_data, daemon=True)
process_thread.start()

mqtt_thread = threading.Thread(target=mqtt_connect_with_retry, daemon=True)
mqtt_thread.start()

# -------------------- ROBOT SOCKET & MOVEMENT -------------------- #

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
            self.sock.settimeout(1.0)
            resp = self.sock.recv(1024).decode()
            return resp
        except socket.timeout:
            print(f"Timeout sending command: {cmd}")
            return None
        except Exception as e:
            print(f"Error sending command '{cmd}': {e}")
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

def robot_forward_enhanced():
    try:
        result1 = robot.send_command("MMW !MG")
        time.sleep(0.05)
        current_speed = fusion_state.turn_override_speed if fusion_state.turn_override_speed else pid_params.speed
        if navigation_max_speed:
            current_speed = min(current_speed, navigation_max_speed)
        current_speed = max(current_speed, navigation_min_speed)
        result2 = robot.send_command(f"MMW !M {current_speed} -{current_speed}")
        if result1 is None or result2 is None:
            print("Warning: Forward command may not have been received properly")
            return False
        return True
    except Exception as e:
        print(f"Error in robot_forward_enhanced: {e}")
        return False

def robot_backward_enhanced():
    try:
        result1 = robot.send_command("MMW !MG")
        time.sleep(0.05)
        current_speed = fusion_state.turn_override_speed if fusion_state.turn_override_speed else pid_params.speed
        if navigation_max_speed:
            current_speed = min(current_speed, navigation_max_speed)
        current_speed = max(current_speed, navigation_min_speed)
        result2 = robot.send_command(f"MMW !M -{current_speed} {current_speed}")
        if result1 is None or result2 is None:
            print("Warning: Backward command may not have been received properly")
            return False
        return True
    except Exception as e:
        print(f"Error in robot_backward_enhanced: {e}")
        return False

def robot_stop_enhanced():
    try:
        for i in range(3):
            robot.send_command("MMW !MG")
            time.sleep(0.02)
            robot.send_command("MMW !M 0 0")
            time.sleep(0.02)
        print("Enhanced stop commands sent")
        return True
    except Exception as e:
        print(f"Error in robot_stop_enhanced: {e}")
        return False

def robot_forward():
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

def navigation_cycle_improved():
    global navigation_active, navigation_direction
    print("Starting improved navigation cycle")
    
    while navigation_active:
        cycle_start_time = time.time()
        direction_name = "Forward" if navigation_direction == 1 else "Backward"
        print(f"Navigation: Starting {direction_name} cycle")
        
        initial_lat = fusion_state.lat if hasattr(fusion_state, 'lat') else None
        initial_lon = fusion_state.lon if hasattr(fusion_state, 'lon') else None
        
        command_success = False
        if navigation_direction == 1:
            command_success = robot_forward_enhanced()
        else:
            command_success = robot_backward_enhanced()
        
        if not command_success:
            print(f"Failed to start {direction_name} movement, retrying...")
            time.sleep(0.5)
            continue
        
        last_command_time = time.time()
        movement_confirmed = False
        
        while navigation_active and (time.time() - cycle_start_time) < navigation_cycle_time:
            current_time = time.time()
            
            if current_time - last_command_time >= navigation_command_interval:
                if navigation_direction == 1:
                    success = robot_forward_enhanced()
                else:
                    success = robot_backward_enhanced()
                
                if success:
                    last_command_time = current_time
                else:
                    print(f"Command refresh failed for {direction_name} movement")
            
            if initial_lat is not None and initial_lon is not None:
                try:
                    current_distance = distance_meters(initial_lat, initial_lon, 
                                                     fusion_state.lat, fusion_state.lon)
                    if current_distance > 0.5:
                        if not movement_confirmed:
                            print(f"Movement confirmed: {current_distance:.2f}m traveled")
                            movement_confirmed = True
                except:
                    pass
            
            time.sleep(0.2)
        
        if not navigation_active:
            break
        
        print(f"Stopping {direction_name} movement")
        robot_stop_enhanced()
        
        pause_start = time.time()
        while navigation_active and (time.time() - pause_start) < 1.0:
            time.sleep(0.1)
        
        if not navigation_active:
            break
        
        navigation_direction *= -1
        new_direction = "Forward" if navigation_direction == 1 else "Backward"
        print(f"Navigation: Switching to {new_direction}")
    
    print("Navigation cycle ending - sending final stop commands")
    robot_stop_enhanced()

def start_navigation_improved():
    global navigation_active, navigation_thread
    
    if navigation_active:
        print("Navigation already active")
        return False
    
    if not robot.sock:
        print("Robot not connected, attempting to connect...")
        if not robot.connect():
            print("Failed to connect to robot for navigation")
            return False
    
    print("Starting improved navigation mode")
    navigation_active = True
    navigation_thread = threading.Thread(target=navigation_cycle_improved, daemon=True)
    navigation_thread.start()
    return True

def stop_navigation_improved():
    global navigation_active
    
    if not navigation_active:
        print("Navigation not active")
        return False
    
    print("Stopping navigation mode")
    navigation_active = False
    robot_stop_enhanced()
    
    if navigation_thread and navigation_thread.is_alive():
        navigation_thread.join(timeout=2.0)
    
    print("Navigation mode stopped")
    return True

def auto_stop_monitor():
    global stop_thread_active, last_movement_time
    
    while stop_thread_active:
        current_time = time.time()
        time_since_movement = current_time - last_movement_time
        
        if time_since_movement > stop_timeout and last_movement_time > 0:
            print("No movement commands detected, sending stop commands...")
            for i in range(3):
                robot_stop()
                time.sleep(0.05)
            last_movement_time = 0
        
        time.sleep(0.1)

def start_auto_stop_monitor():
    global stop_thread, stop_thread_active
    if not stop_thread_active:
        stop_thread_active = True
        stop_thread = threading.Thread(target=auto_stop_monitor, daemon=True)
        stop_thread.start()
        print("Auto-stop monitor started")

# -------------------- FLASK ROUTES -------------------- #

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/data')
def get_data():
    with sensor_data_lock:
        return jsonify({
            'pi_tracker': sensor_data.get('position'),
            'pi_imu': sensor_data.get('imu'),
            'pi_gps': sensor_data.get('gps'),
            'legacy': current_data
        })

@app.route('/sensor-logs')
def get_sensor_logs():
    """Get list of sensor log files and path"""
    try:
        files = []
        if os.path.exists(SENSOR_LOGS_DIR):
            for filename in sorted(os.listdir(SENSOR_LOGS_DIR), reverse=True):
                if filename.endswith('.csv'):
                    filepath = os.path.join(SENSOR_LOGS_DIR, filename)
                    file_size = os.path.getsize(filepath)
                    file_time = os.path.getmtime(filepath)
                    files.append({
                        'filename': filename,
                        'size': file_size,
                        'path': filepath,
                        'timestamp': datetime.fromtimestamp(file_time).strftime('%Y-%m-%d %H:%M:%S')
                    })
        
        return jsonify({
            'success': True,
            'logs': files,
            'directory': SENSOR_LOGS_DIR,
            'count': len(files)
        })
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})

@app.route('/mqtt-data')
def get_mqtt_data():
    return jsonify(mqtt_received_data)

@app.route('/connect-robot', methods=['POST'])
def connect_robot():
    ok = robot.connect()
    return jsonify({'success': ok})

@app.route('/robot-cmd', methods=['POST'])
def robot_cmd():
    global last_command_time, last_movement_time
    
    current_time = time.time()
    time_since_last = current_time - last_command_time
    
    if time_since_last < command_delay:
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
        return jsonify({'success': success, 'throttled': False, 'navigation_active': navigation_active})
    else:
        print("Unknown command:", cmd)
    return jsonify({'success': True, 'throttled': False})

@app.route('/navigation-status')
def get_navigation_status():
    return jsonify({
        'active': navigation_active,
        'direction': 'forward' if navigation_direction == 1 else 'backward',
        'cycle_time': navigation_cycle_time,
        'command_interval': navigation_command_interval,
        'max_speed': navigation_max_speed,
        'min_speed': navigation_min_speed
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
        print("Error updating PID:", e)
    return jsonify({
        "kp": pid_params.kp,
        "ki": pid_params.ki,
        "kd": pid_params.kd,
        "speed": pid_params.speed,
        "nav": navigation_cycle_time
    })

@app.route('/navigation-config', methods=['POST'])
def update_navigation_config():
    """Update navigation configuration parameters"""
    global navigation_cycle_time, navigation_command_interval, navigation_max_speed, navigation_min_speed
    
    data = request.get_json() or {}
    
    try:
        if 'cycle_time' in data:
            navigation_cycle_time = max(1, float(data['cycle_time']))
        if 'command_interval' in data:
            navigation_command_interval = max(0.1, float(data['command_interval']))
        if 'max_speed' in data:
            navigation_max_speed = int(data['max_speed']) if data['max_speed'] else None
        if 'min_speed' in data:
            navigation_min_speed = max(20, int(data['min_speed']))
        
        return jsonify({
            'success': True,
            'config': {
                'cycle_time': navigation_cycle_time,
                'command_interval': navigation_command_interval,
                'max_speed': navigation_max_speed,
                'min_speed': navigation_min_speed
            }
        })
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})

@app.route('/navigation-config', methods=['GET'])
def get_navigation_config():
    """Get current navigation configuration"""
    return jsonify({
        'cycle_time': navigation_cycle_time,
        'command_interval': navigation_command_interval,
        'max_speed': navigation_max_speed,
        'min_speed': navigation_min_speed,
        'active': navigation_active,
        'direction': 'forward' if navigation_direction == 1 else 'backward'
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

@app.route('/capture-photo', methods=['POST'])
def capture_photo():
    """Capture a photo from the current camera frame and save it locally"""
    global latest_frame
    
    if latest_frame is None:
        return jsonify({'success': False, 'error': 'No camera frame available'})
    
    try:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'photo_{timestamp}.jpg'
        filepath = os.path.join(PHOTOS_DIR, filename)
        
        success = cv2.imwrite(filepath, latest_frame)
        
        if success:
            file_size = os.path.getsize(filepath)
            print(f"Photo captured: {filename} ({file_size} bytes)")
            
            return jsonify({
                'success': True,
                'filename': filename,
                'filepath': filepath,
                'size': file_size,
                'timestamp': timestamp
            })
        else:
            return jsonify({'success': False, 'error': 'Failed to save image'})
            
    except Exception as e:
        print(f"Error capturing photo: {e}")
        return jsonify({'success': False, 'error': str(e)})

@app.route('/photos', methods=['GET'])
def list_photos():
    """List all captured photos"""
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
        
        return jsonify({
            'success': True,
            'photos': photos,
            'count': len(photos)
        })
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})

@app.route('/delete-photo/<filename>', methods=['DELETE'])
def delete_photo(filename):
    """Delete a specific photo"""
    try:
        filepath = os.path.join(PHOTOS_DIR, filename)
        
        if '..' in filename or '/' in filename or '\\' in filename:
            return jsonify({'success': False, 'error': 'Invalid filename'})
        
        if os.path.exists(filepath) and filename.endswith('.jpg'):
            os.remove(filepath)
            print(f"Photo deleted: {filename}")
            return jsonify({'success': True, 'message': f'Deleted {filename}'})
        else:
            return jsonify({'success': False, 'error': 'Photo not found'})
            
    except Exception as e:
        print(f"Error deleting photo: {e}")
        return jsonify({'success': False, 'error': str(e)})

if __name__ == '__main__':
    start_auto_stop_monitor()
    print(f"\n{'='*60}")
    print("Jaguar Control Server Started")
    print(f"{'='*60}")
    print(f"Sensor logs directory: {SENSOR_LOGS_DIR}")
    print(f"Current log file: {SENSOR_LOG_FILE}")
    print(f"{'='*60}\n")
    app.run(host='0.0.0.0', port=5014, debug=True)
