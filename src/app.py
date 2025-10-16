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

app = Flask(__name__)

# Create photos directory if it doesn't exist
PHOTOS_DIR = 'captured_photos'
if not os.path.exists(PHOTOS_DIR):
    os.makedirs(PHOTOS_DIR)

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

# Navigation mode variables - IMPROVED
navigation_active = False
navigation_thread = None
navigation_direction = 1  # 1 for forward, -1 for backward
navigation_cycle_time = 20  # seconds per direction
navigation_command_interval = 0.5  # Send commands every 500ms for persistence
navigation_ramp_time = 1.0  # Time to ramp up/down speed
navigation_max_speed = None  # Will use pid_params.speed if None
navigation_min_speed = 50   # Minimum speed for reliable movement

# Command rate limiting variables
last_command_time = 0
command_delay = 0.1  # Minimum delay between commands in seconds (100ms)

# Stop command variables
last_movement_time = 0
stop_timeout = 0.3  # Send stop if no movement commands for 300ms
stop_thread = None
stop_thread_active = False

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
def auto_capture_photos(interval = 2):
	while True:
		save_current_frame()
		time.sleep(interval)
capture_thread = threading.Thread(target = auto_capture_photos, daemon = True)
capture_thread.start()
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
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
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

# ------------------ IMPROVED NAVIGATION FUNCTIONS ------------------ #

def robot_forward_enhanced():
    """Enhanced forward movement with error handling and confirmation"""
    try:
        # Send motor go command first
        result1 = robot.send_command("MMW !MG")
        time.sleep(0.05)  # Small delay between commands
        
        # Get current speed (use override speed if available)
        current_speed = fusion_state.turn_override_speed if fusion_state.turn_override_speed else pid_params.speed
        if navigation_max_speed:
            current_speed = min(current_speed, navigation_max_speed)
        current_speed = max(current_speed, navigation_min_speed)
        
        # Send movement command with proper motor directions
        result2 = robot.send_command(f"MMW !M {current_speed} -{current_speed}")
        
        # Verify commands were successful
        if result1 is None or result2 is None:
            print("Warning: Forward command may not have been received properly")
            return False
        return True
    except Exception as e:
        print(f"Error in robot_forward_enhanced: {e}")
        return False

def robot_backward_enhanced():
    """Enhanced backward movement with error handling and confirmation"""
    try:
        # Send motor go command first
        result1 = robot.send_command("MMW !MG")
        time.sleep(0.05)  # Small delay between commands
        
        # Get current speed
        current_speed = fusion_state.turn_override_speed if fusion_state.turn_override_speed else pid_params.speed
        if navigation_max_speed:
            current_speed = min(current_speed, navigation_max_speed)
        current_speed = max(current_speed, navigation_min_speed)
        
        # Send movement command with proper motor directions for backward
        result2 = robot.send_command(f"MMW !M -{current_speed} {current_speed}")
        
        # Verify commands were successful
        if result1 is None or result2 is None:
            print("Warning: Backward command may not have been received properly")
            return False
        return True
    except Exception as e:
        print(f"Error in robot_backward_enhanced: {e}")
        return False

def robot_stop_enhanced():
    """Enhanced stop with multiple confirmation commands"""
    try:
        # Send multiple stop commands for reliability
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

# Original movement functions (kept for compatibility)
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
    """Improved navigation cycle with better command persistence and monitoring"""
    global navigation_active, navigation_direction
    
    print("Starting improved navigation cycle")
    
    while navigation_active:
        cycle_start_time = time.time()
        direction_name = "Forward" if navigation_direction == 1 else "Backward"
        print(f"Navigation: Starting {direction_name} cycle")
        
        # Store initial position for monitoring (if available)
        initial_lat = fusion_state.lat if hasattr(fusion_state, 'lat') else None
        initial_lon = fusion_state.lon if hasattr(fusion_state, 'lon') else None
        
        # Send initial movement command
        command_success = False
        if navigation_direction == 1:
            command_success = robot_forward_enhanced()
        else:
            command_success = robot_backward_enhanced()
        
        if not command_success:
            print(f"Failed to start {direction_name} movement, retrying...")
            time.sleep(0.5)
            continue
        
        # Maintain movement during cycle with periodic command refresh
        last_command_time = time.time()
        movement_confirmed = False
        
        while navigation_active and (time.time() - cycle_start_time) < navigation_cycle_time:
            current_time = time.time()
            
            # Send refresh commands periodically
            if current_time - last_command_time >= navigation_command_interval:
                if navigation_direction == 1:
                    success = robot_forward_enhanced()
                else:
                    success = robot_backward_enhanced()
                
                if success:
                    last_command_time = current_time
                else:
                    print(f"Command refresh failed for {direction_name} movement")
            
            # Monitor movement progress (if GPS/fusion data available)
            if initial_lat is not None and initial_lon is not None:
                try:
                    current_distance = distance_meters(initial_lat, initial_lon, 
                                                     fusion_state.lat, fusion_state.lon)
                    if current_distance > 0.5:  # If we've moved more than 50cm
                        if not movement_confirmed:
                            print(f"Movement confirmed: {current_distance:.2f}m traveled")
                            movement_confirmed = True
                except:
                    pass  # GPS data might not be available
            
            time.sleep(0.2)  # Check every 200ms
        
        # Check if navigation is still active before continuing
        if not navigation_active:
            break
        
        # Enhanced stopping sequence
        print(f"Stopping {direction_name} movement")
        robot_stop_enhanced()
        
        # Pause between direction changes with safety checks
        pause_start = time.time()
        while navigation_active and (time.time() - pause_start) < 1.0:  # 1 second pause
            time.sleep(0.1)
        
        if not navigation_active:
            break
        
        # Switch direction
        navigation_direction *= -1
        new_direction = "Forward" if navigation_direction == 1 else "Backward"
        print(f"Navigation: Switching to {new_direction}")
    
    # Final stop when navigation ends
    print("Navigation cycle ending - sending final stop commands")
    robot_stop_enhanced()

def start_navigation_improved():
    """Start improved navigation with better initialization"""
    global navigation_active, navigation_thread
    
    if navigation_active:
        print("Navigation already active")
        return False
    
    # Check robot connection first
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
    """Stop navigation with proper cleanup"""
    global navigation_active
    
    if not navigation_active:
        print("Navigation not active")
        return False
    
    print("Stopping navigation mode")
    navigation_active = False
    
    # Send immediate stop commands
    robot_stop_enhanced()
    
    # Wait a moment for thread to finish
    if navigation_thread and navigation_thread.is_alive():
        navigation_thread.join(timeout=2.0)
    
    print("Navigation mode stopped")
    return True

def auto_stop_monitor():
    """Monitor for lack of movement commands and send stop commands"""
    global stop_thread_active, last_movement_time
    
    while stop_thread_active:
        current_time = time.time()
        time_since_movement = current_time - last_movement_time
        
        # If no movement commands for stop_timeout duration, send stop commands
        if time_since_movement > stop_timeout and last_movement_time > 0:
            print("No movement commands detected, sending stop commands...")
            # Send multiple stop commands to ensure robot stops
            for i in range(3):
                robot_stop()
                time.sleep(0.05)  # Small delay between stop commands
            
            # Reset to prevent continuous stopping
            last_movement_time = 0
        
        time.sleep(0.1)  # Check every 100ms

def start_auto_stop_monitor():
    """Start the auto-stop monitoring thread"""
    global stop_thread, stop_thread_active
    if not stop_thread_active:
        stop_thread_active = True
        stop_thread = threading.Thread(target=auto_stop_monitor, daemon=True)
        stop_thread.start()
        print("Auto-stop monitor started")

# ------------------ FLASK ROUTES ------------------ #

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
    global last_command_time, last_movement_time
    
    # Rate limiting: check if enough time has passed since last command
    current_time = time.time()
    time_since_last = current_time - last_command_time
    
    if time_since_last < command_delay:
        # Too soon, ignore this command to prevent backlog
        return jsonify({'success': True, 'throttled': True})
    
    # Update last command time
    last_command_time = current_time
    
    data = request.json
    cmd = data.get('cmd', '').lower()
    
    # Track movement commands (not stop commands)
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
        # Reset movement time when explicit stop is sent
        last_movement_time = 0
    elif cmd == 'n':  # Navigate command - IMPROVED
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

# NEW: Navigation configuration endpoints
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

def save_current_frame():
    global latest_frame

    if latest_frame is None:
        return {'success': False, 'error': 'No camera frame available'}
    
    try:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'photo_{timestamp}.jpg'
        filepath = os.path.join(PHOTOS_DIR, filename)
        success = cv2.imwrite(filepath, latest_frame)
        
        if success:
            file_size = os.path.getsize(filepath)
            print(f"Photo captured: {filename} ({file_size} bytes)")
            return {
                'success': True,
                'filename': filename,
                'filepath': filepath,
                'size': file_size,
                'timestamp': timestamp
            }
        else:
            return {'success': False, 'error': 'Failed to save image'}
    except Exception as e:
        print(f"Error capturing photo: {e}")
        return {'success': False, 'error': str(e)}


# NEW: Photo capture endpoint

@app.route('/capture-photo', methods=['POST'])
def capture_photo():
    result = save_current_frame()
    return jsonify(result)

# NEW: List captured photos
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

# NEW: Delete a photo
@app.route('/delete-photo/<filename>', methods=['DELETE'])
def delete_photo(filename):
    """Delete a specific photo"""
    try:
        filepath = os.path.join(PHOTOS_DIR, filename)
        
        # Security check: ensure filename doesn't contain path traversal
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
    # Start the auto-stop monitor
    start_auto_stop_monitor()
    app.run(host='0.0.0.0', port=5014, debug=True)
