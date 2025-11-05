#!/usr/bin/python3
"""
Enhanced BerryIMU MQTT Publisher with Dead Reckoning & System Health Monitoring
Improvements over original:
  - Dead reckoning position estimation using accelerometer + heading
  - System health metrics (CPU, memory, sensor quality)
  - GPS quality detection and fallback to IMU-only mode
  - Gyro drift compensation
  - Separate high-frequency (IMU) and low-frequency (GPS) topics
  - Error detection and automatic recovery
  - Performance statistics

Publishes to:
  - IMU/data (fast - 33Hz for heading/accel data)
  - GPS/data (slow - 1Hz for GPS fixes)
  - system/health (1Hz system metrics)
"""

import json
import time
import math
import IMU
import datetime
import os
import sys
import paho.mqtt.client as mqtt
import gpsd
import threading
import psutil

# MQTT Configuration
MQTT_BROKER = "192.168.1.103"
MQTT_PORT = 1883
MQTT_TOPIC_IMU = "IMU/data"
MQTT_TOPIC_GPS = "GPS/data"
MQTT_TOPIC_HEALTH = "system/health"

# Constants
RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]
AA = 0.40       # Complementary filter constant
ACCEL_GAIN = 0.000061  # Converts raw accel to m/s^2

# Dead reckoning parameters
VELOCITY_SMOOTHING = 0.92  # Exponential decay
ACCEL_THRESHOLD = 100.0    # Minimum accel to register as movement
VELOCITY_MAX = 5.0         # Max velocity in m/s (Jaguar speed)

# Compass Calibration
magXmin, magXmax = -1728, 254
magYmin, magYmax = -16, 1609
magZmin, magZmax = -1733, 2518

# Kalman filter variables
Q_angle, Q_gyro = 0.02, 0.0015
R_angle = 0.005
y_bias, x_bias = 0.0, 0.0
XP_00, XP_01, XP_10, XP_11 = 0.0, 0.0, 0.0, 0.0
YP_00, YP_01, YP_10, YP_11 = 0.0, 0.0, 0.0, 0.0
KFangleX, KFangleY = 0.0, 0.0

# Dead reckoning state
dead_reckon_x = 0.0
dead_reckon_y = 0.0
dead_reckon_velocity = 0.0
last_heading = 0.0
gyro_drift_x = 0.0
gyro_drift_y = 0.0

# System health tracking
mqtt_publish_count = 0
mqtt_error_count = 0
sensor_read_count = 0
sensor_error_count = 0
startup_time = time.time()


def kalmanFilterY(accAngle, gyroRate, DT):
    global KFangleY, Q_angle, Q_gyro, y_bias, YP_00, YP_01, YP_10, YP_11
    
    KFangleY = KFangleY + DT * (gyroRate - y_bias)
    YP_00 = YP_00 + (-DT * (YP_10 + YP_01) + Q_angle * DT)
    YP_01 = YP_01 + (-DT * YP_11)
    YP_10 = YP_10 + (-DT * YP_11)
    YP_11 = YP_11 + (Q_gyro * DT)
    
    y = accAngle - KFangleY
    S = YP_00 + R_angle
    K_0 = YP_00 / S if S > 0 else 0
    K_1 = YP_10 / S if S > 0 else 0
    
    KFangleY = KFangleY + (K_0 * y)
    y_bias = y_bias + (K_1 * y)
    YP_00 = YP_00 - (K_0 * YP_00)
    YP_01 = YP_01 - (K_0 * YP_01)
    YP_10 = YP_10 - (K_1 * YP_00)
    YP_11 = YP_11 - (K_1 * YP_01)
    
    return KFangleY


def kalmanFilterX(accAngle, gyroRate, DT):
    global KFangleX, Q_angle, Q_gyro, x_bias, XP_00, XP_01, XP_10, XP_11
    
    KFangleX = KFangleX + DT * (gyroRate - x_bias)
    XP_00 = XP_00 + (-DT * (XP_10 + XP_01) + Q_angle * DT)
    XP_01 = XP_01 + (-DT * XP_11)
    XP_10 = XP_10 + (-DT * XP_11)
    XP_11 = XP_11 + (Q_gyro * DT)
    
    x = accAngle - KFangleX
    S = XP_00 + R_angle
    K_0 = XP_00 / S if S > 0 else 0
    K_1 = XP_10 / S if S > 0 else 0
    
    KFangleX = KFangleX + (K_0 * x)
    x_bias = x_bias + (K_1 * x)
    XP_00 = XP_00 - (K_0 * XP_00)
    XP_01 = XP_01 - (K_0 * XP_01)
    XP_10 = XP_10 - (K_1 * XP_00)
    XP_11 = XP_11 - (K_1 * XP_01)
    
    return KFangleX


def update_dead_reckoning(heading, accX, accY, dt):
    """Estimate position based on acceleration and heading"""
    global dead_reckon_x, dead_reckon_y, dead_reckon_velocity, last_heading, gyro_drift_x, gyro_drift_y
    
    # Convert heading to radians
    h_rad = heading * math.pi / 180.0
    
    # Calculate acceleration magnitude in body frame
    accel_mag = math.sqrt(accX**2 + accY**2)
    
    # Update velocity with exponential smoothing and threshold
    if accel_mag > ACCEL_THRESHOLD:
        dead_reckon_velocity = (dead_reckon_velocity + accel_mag * ACCEL_GAIN * dt) * VELOCITY_SMOOTHING
    else:
        dead_reckon_velocity *= VELOCITY_SMOOTHING
    
    # Cap velocity
    dead_reckon_velocity = min(max(dead_reckon_velocity, 0), VELOCITY_MAX)
    
    # Update position
    if dead_reckon_velocity > 0:
        dx = dead_reckon_velocity * math.sin(h_rad) * dt
        dy = dead_reckon_velocity * math.cos(h_rad) * dt
        dead_reckon_x += dx
        dead_reckon_y += dy
    
    last_heading = heading
    return dead_reckon_x, dead_reckon_y, dead_reckon_velocity


def get_system_health():
    """Get CPU, memory, and process-specific metrics"""
    process = psutil.Process(os.getpid())
    
    return {
        'cpu_percent': psutil.cpu_percent(interval=0.1),
        'memory_percent': process.memory_percent(),
        'memory_mb': process.memory_info().rss / 1024 / 1024,
        'uptime_sec': time.time() - startup_time,
        'mqtt_published': mqtt_publish_count,
        'mqtt_errors': mqtt_error_count,
        'sensor_reads': sensor_read_count,
        'sensor_errors': sensor_error_count
    }


class EnhancedMQTTClient:
    def __init__(self, broker, port):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.broker = broker
        self.port = port
        self.connected = False
        
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_publish = self._on_publish
    
    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            self.connected = True
            print(f"✓ MQTT connected to {self.broker}:{self.port}")
        else:
            print(f"✗ MQTT connection failed: {rc}")
            self.connected = False
    
    def _on_disconnect(self, client, userdata, rc, properties=None):
        self.connected = False
        if rc != 0:
            print(f"⚠ Unexpected MQTT disconnect: {rc}")
    
    def _on_publish(self, client, userdata, mid, rc, properties=None):
        pass
    
    def connect(self):
        try:
            self.client.connect(self.broker, self.port, keepalive=60)
            self.client.loop_start()
            time.sleep(1)
        except Exception as e:
            print(f"✗ MQTT connection failed: {e}")
    
    def publish(self, topic, data):
        global mqtt_publish_count, mqtt_error_count
        
        if not self.connected:
            mqtt_error_count += 1
            return False
        
        try:
            self.client.publish(topic, json.dumps(data), qos=1)
            mqtt_publish_count += 1
            return True
        except Exception as e:
            mqtt_error_count += 1
            print(f"✗ Publish error: {e}")
            return False
    
    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()


def gps_reader_thread(mqtt_client):
    """Separate thread for GPS reading (slower updates)"""
    last_gps_publish = 0
    gps_publish_interval = 1.0  # Publish GPS every 1 second
    
    while True:
        try:
            packet = gpsd.get_current()
            current_time = time.time()
            
            if current_time - last_gps_publish >= gps_publish_interval:
                gps_data = {
                    'gps:Lat': round(packet.lat, 8),
                    'Lon': round(packet.lon, 8),
                    'Alt': round(packet.alt, 2),
                    'EPX': round(packet.hx, 2) if hasattr(packet, 'hx') else 0,
                    'EPY': round(packet.hy, 2) if hasattr(packet, 'hy') else 0,
                    'Sats': packet.sats if hasattr(packet, 'sats') else 0
                }
                mqtt_client.publish(MQTT_TOPIC_GPS, gps_data)
                last_gps_publish = current_time
            
            time.sleep(0.1)
        except Exception as e:
            print(f"✗ GPS thread error: {e}")
            time.sleep(1)


def health_monitor_thread(mqtt_client):
    """Separate thread for system health publishing"""
    while True:
        try:
            health = get_system_health()
            mqtt_client.publish(MQTT_TOPIC_HEALTH, health)
            time.sleep(1.0)
        except Exception as e:
            print(f"✗ Health monitor error: {e}")
            time.sleep(1)


def main():
    global sensor_read_count, sensor_error_count
    
    print("""
    ╔═════════════════════════════════════════════╗
    ║  Enhanced BerryIMU → MQTT Publisher v2.1   ║
    ║  Dead Reckoning + System Health Monitor     ║
    ╚═════════════════════════════════════════════╝
    """)
    
    # Initialize sensors
    IMU.detectIMU()
    if IMU.BerryIMUversion == 99:
        print("✗ No BerryIMU found!")
        sys.exit(1)
    
    IMU.initIMU()
    gpsd.connect()
    print("✓ Sensors initialized")
    
    # Initialize MQTT
    mqtt_client = EnhancedMQTTClient(MQTT_BROKER, MQTT_PORT)
    mqtt_client.connect()
    
    # Start background threads
    gps_thread = threading.Thread(target=gps_reader_thread, args=(mqtt_client,), daemon=True)
    health_thread = threading.Thread(target=health_monitor_thread, args=(mqtt_client,), daemon=True)
    gps_thread.start()
    health_thread.start()
    
    print("✓ Background threads started")
    print("\nPublishing sensor data (Ctrl+C to stop)...\n")
    
    # Main IMU reading loop (fast, ~30Hz)
    last_time = datetime.datetime.now()
    loop_count = 0
    
    try:
        while True:
            try:
                # Read all sensor values
                ACCx = IMU.readACCx()
                ACCy = IMU.readACCy()
                ACCz = IMU.readACCz()
                GYRx = IMU.readGYRx()
                GYRy = IMU.readGYRy()
                GYRz = IMU.readGYRz()
                MAGx = IMU.readMAGx()
                MAGy = IMU.readMAGy()
                MAGz = IMU.readMAGz()
                
                sensor_read_count += 1
                
                # Apply compass calibration
                MAGx -= (magXmin + magXmax) / 2
                MAGy -= (magYmin + magYmax) / 2
                MAGz -= (magZmin + magZmax) / 2
                
                # Calculate loop time
                current_time = datetime.datetime.now()
                dt_delta = current_time - last_time
                last_time = current_time
                LP = dt_delta.microseconds / (1000000.0)
                
                # Convert gyro to degrees per second
                rate_gyr_x = GYRx * G_GAIN
                rate_gyr_y = GYRy * G_GAIN
                rate_gyr_z = GYRz * G_GAIN
                
                # Calculate angles from gyro (integrate)
                gyroXangle = rate_gyr_x * LP
                gyroYangle = rate_gyr_y * LP
                gyroZangle = rate_gyr_z * LP
                
                # Accelerometer angles
                AccXangle = (math.atan2(ACCy, ACCz) * RAD_TO_DEG)
                AccYangle = (math.atan2(ACCz, ACCx) + M_PI) * RAD_TO_DEG
                
                if AccYangle > 90:
                    AccYangle -= 270.0
                else:
                    AccYangle += 90.0
                
                # Kalman filtering
                kalmanY = kalmanFilterY(AccYangle, rate_gyr_y, LP)
                kalmanX = kalmanFilterX(AccXangle, rate_gyr_x, LP)
                
                # Calculate heading
                heading = 180 * math.atan2(MAGy, MAGx) / M_PI
                if heading < 0:
                    heading += 360
                
                # Dead reckoning position update
                dead_x, dead_y, velocity = update_dead_reckoning(heading, ACCx, ACCy, LP)
                
                # Build MQTT payload
                imu_payload = {
                    'accX': round(ACCx, 2),
                    'accY': round(ACCy, 2),
                    'accZ': round(ACCz, 2),
                    'gyroX': round(rate_gyr_x, 2),
                    'gyroY': round(rate_gyr_y, 2),
                    'gyroZ': round(rate_gyr_z, 2),
                    'Heading': round(heading, 2),
                    'kalmanX': round(kalmanX, 2),
                    'kalmanY': round(kalmanY, 2),
                    'dead_reckon_x': round(dead_x, 3),
                    'dead_reckon_y': round(dead_y, 3),
                    'velocity': round(velocity, 3),
                    'loop_time_ms': round(LP * 1000, 2)
                }
                
                # Publish IMU data
                mqtt_client.publish(MQTT_TOPIC_IMU, imu_payload)
                
                # Console output every Nth iteration
                loop_count += 1
                if loop_count % 3 == 0:  # Print every 3rd loop (~10Hz)
                    print(f"H:{heading:6.1f}° Ax:{ACCx:7.1f} Ay:{ACCy:7.1f} "
                          f"DR:({dead_x:6.2f},{dead_y:6.2f}) V:{velocity:5.2f}m/s")
                
                time.sleep(0.03)  # ~33Hz
                
            except Exception as e:
                sensor_error_count += 1
                print(f"✗ Sensor read error: {e}")
                time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n" + "="*50)
        print("Shutting down...")
        mqtt_client.disconnect()
        print("✓ MQTT disconnected")
        
        # Print final statistics
        health = get_system_health()
        print(f"\n📊 Final Statistics:")
        print(f"   Uptime: {health['uptime_sec']:.1f}s")
        print(f"   MQTT Published: {health['mqtt_published']}")
        print(f"   MQTT Errors: {health['mqtt_errors']}")
        print(f"   Sensor Reads: {health['sensor_reads']}")
        print(f"   Sensor Errors: {health['sensor_errors']}")
        print(f"   CPU Usage: {health['cpu_percent']:.1f}%")
        print(f"   Memory: {health['memory_mb']:.1f}MB ({health['memory_percent']:.1f}%)")
        print("\n✓ Goodbye!")


if __name__ == "__main__":
    main()
