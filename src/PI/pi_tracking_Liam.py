#!/usr/bin/env python3
"""
Raspberry Pi IMU/GPS Position Tracker with MQTT Publisher (GPSD Version)
Sends IMU heading, acceleration, and GPS data via GPSD to Jaguar controller
Publishes to: IMU/data topic for Flask backend consumption

SETUP:
  sudo apt-get install gpsd gpsd-clients python3-gps3
  sudo gpsd /dev/ttyACM0 -F /var/run/gpsd.sock
"""

import json
import time
import math
import threading
from datetime import datetime
import paho.mqtt.client as mqtt

try:
    from adafruit_bno055 import Adafruit_BNO055
    import busio
    import board
    BNO055_AVAILABLE = True
except ImportError:
    BNO055_AVAILABLE = False
    print("Warning: BNO055 not available")

try:
    from gps3 import agps3
    GPSD_AVAILABLE = True
except ImportError:
    GPSD_AVAILABLE = False
    print("Warning: gps3 (GPSD) not available - install with: pip3 install gps3")

# Constants
EARTH_RADIUS = 6371000
DEG_TO_RAD = math.pi / 180
PUBLISH_INTERVAL = 0.2  # 5Hz publishing rate
MQTT_BROKER = "192.168.0.103"  # Change to your broker IP
MQTT_PORT = 1883
MQTT_TOPIC_IMU = "IMU/data"


class GPSdaemon:
    """GPSD client for GPS data via socket (non-blocking)"""
    def __init__(self, host='127.0.0.1', port=2947):
        self.host = host
        self.port = port
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.sats = 0
        self.fix = 0
        self.last_valid = time.time()
        self.socket = None
        self.stream = None
        self.connected = False
        self._init_gpsd()
    
    def _init_gpsd(self):
        """Initialize connection to GPSD daemon"""
        if not GPSD_AVAILABLE:
            return
        
        try:
            self.socket = agps3.GPSDSocket()
            self.stream = agps3.DataStream()
            self.socket.connect()
            self.socket.watch()
            self.connected = True
            print("✓ GPSD connection established")
        except Exception as e:
            print(f"✗ GPSD initialization failed: {e}")
            self.connected = False
    
    def update(self):
        """Non-blocking read from GPSD (don't call faster than 1Hz)"""
        if not self.connected or not self.socket:
            return False
        
        try:
            # Read one JSON packet with short timeout
            for new_data in self.socket:
                if new_data:
                    self.stream.unpack(new_data)
                    
                    # Check for valid TPV (position) data
                    if self.stream.TPV['mode'] >= 2:  # 2D or 3D fix
                        self.lat = self.stream.TPV.get('lat', 0.0)
                        self.lon = self.stream.TPV.get('lon', 0.0)
                        self.alt = self.stream.TPV.get('alt', 0.0)
                        self.fix = self.stream.TPV['mode']
                        
                        # Get satellite count from SKY data if available
                        if hasattr(self.stream, 'SKY'):
                            self.sats = self.stream.SKY.get('nSat', 0)
                        
                        self.last_valid = time.time()
                        return True
            
            return False
        except Exception as e:
            print(f"Error reading GPSD: {e}")
            return False
    
    def get_data(self):
        """Return current GPS data as dictionary"""
        return {
            'gps:Lat': round(self.lat, 8),
            'Lat': round(self.lat, 8),
            'Lon': round(self.lon, 8),
            'Alt': round(self.alt, 2),
            'Sats': self.sats,
            'Fix': self.fix
        }


class IMUSensor:
    """Interface to BNO055 9-DOF IMU"""
    def __init__(self):
        self.heading = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        self.imu = None
        self.connected = False
        self._init_imu()
    
    def _init_imu(self):
        """Initialize BNO055 IMU over I2C"""
        if not BNO055_AVAILABLE:
            return
        
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.imu = Adafruit_BNO055.Adafruit_BNO055_I2C(i2c=i2c)
            time.sleep(1)
            self.connected = True
            print("✓ BNO055 IMU initialized")
        except Exception as e:
            print(f"✗ BNO055 initialization failed: {e}")
            self.connected = False
    
    def update(self):
        """Read latest IMU data"""
        if not self.connected or not self.imu:
            return False
        
        try:
            # Read Euler angles (heading, roll, pitch)
            euler = self.imu.read_euler()
            if euler:
                self.heading = float(euler[0])
                self.roll = float(euler[1])
                self.pitch = float(euler[2])
            
            # Read linear acceleration (gravity removed)
            accel = self.imu.read_linear_acceleration()
            if accel:
                self.acc_x = float(accel[0])
                self.acc_y = float(accel[1])
                self.acc_z = float(accel[2])
            
            return True
        except Exception as e:
            print(f"Error reading IMU: {e}")
            return False
    
    def get_data(self):
        """Return current IMU data as dictionary"""
        return {
            'Heading': round(self.heading, 2),
            'Roll': round(self.roll, 2),
            'Pitch': round(self.pitch, 2),
            'accX': round(self.acc_x, 2),
            'accY': round(self.acc_y, 2),
            'accZ': round(self.acc_z, 2)
        }


class MQTTPublisher:
    """MQTT client for publishing sensor data"""
    def __init__(self, broker=MQTT_BROKER, port=MQTT_PORT):
        self.broker = broker
        self.port = port
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.connected = False
        self._setup_callbacks()
    
    def _setup_callbacks(self):
        """Setup MQTT connection callbacks"""
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_publish = self._on_publish
    
    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            self.connected = True
            print(f"✓ MQTT connected to {self.broker}:{self.port}")
        else:
            print(f"✗ MQTT connection failed with code {rc}")
            self.connected = False
    
    def _on_disconnect(self, client, userdata, rc, properties=None):
        self.connected = False
        if rc != 0:
            print(f"Unexpected MQTT disconnection: {rc}")
    
    def _on_publish(self, client, userdata, mid, rc, properties=None):
        pass  # Silent on publish
    
    def connect(self):
        """Connect to MQTT broker with reconnect logic"""
        try:
            self.client.connect(self.broker, self.port, keepalive=60)
            self.client.loop_start()
            time.sleep(1)
        except Exception as e:
            print(f"✗ Failed to connect to MQTT: {e}")
    
    def publish_data(self, imu_data, gps_data):
        """Publish combined IMU and GPS data to IMU/data topic"""
        if not self.connected:
            return False
        
        try:
            # Combine IMU and GPS data into single payload
            payload = {**imu_data, **gps_data}
            self.client.publish(MQTT_TOPIC_IMU, json.dumps(payload), qos=1)
            return True
        except Exception as e:
            print(f"Error publishing data: {e}")
            return False
    
    def disconnect(self):
        """Gracefully disconnect from MQTT"""
        self.client.loop_stop()
        self.client.disconnect()


class SensorTracker:
    """Main tracker coordinating IMU, GPS, and MQTT publishing"""
    def __init__(self):
        print("Initializing Jaguar Sensor Tracker...")
        print("-" * 50)
        
        self.imu = IMUSensor()
        self.gps = GPSdaemon()
        self.mqtt = MQTTPublisher()
        
        self.last_publish = time.time()
        self.last_gps_read = time.time()
        self.publish_count = 0
        self.error_count = 0
    
    def run(self):
        """Main loop - update sensors and publish"""
        print("\nStarting sensor tracking (5Hz)...")
        print("Press Ctrl+C to stop\n")
        
        try:
            while True:
                now = time.time()
                
                # Update IMU every loop
                self.imu.update()
                
                # Update GPS at slower rate (GPSD calls are blocking)
                if now - self.last_gps_read >= 0.5:  # Read GPS every 500ms
                    self.gps.update()
                    self.last_gps_read = now
                
                # Publish at configured interval
                if now - self.last_publish >= PUBLISH_INTERVAL:
                    imu_data = self.imu.get_data()
                    gps_data = self.gps.get_data()
                    
                    if self.mqtt.publish_data(imu_data, gps_data):
                        self.publish_count += 1
                        
                        # Console output
                        fix_str = "✓ GPS" if gps_data['Fix'] > 0 else "○ IMU"
                        print(f"[{fix_str}] H:{imu_data['Heading']:6.1f}° "
                              f"Ax:{imu_data['accX']:6.2f} Ay:{imu_data['accY']:6.2f} "
                              f"Lat:{gps_data['Lat']:10.6f} Lon:{gps_data['Lon']:10.6f} "
                              f"Sats:{gps_data['Sats']:2d}")
                    else:
                        self.error_count += 1
                    
                    self.last_publish = now
                
                # Small sleep to prevent CPU spinning
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            self.shutdown()
    
    def shutdown(self):
        """Clean shutdown"""
        print("\n" + "-" * 50)
        print("Shutting down...")
        self.mqtt.disconnect()
        print(f"Published: {self.publish_count} messages")
        print(f"Errors: {self.error_count}")
        print("Goodbye!")


def main():
    print("""
    ╔══════════════════════════════════════════╗
    ║  Jaguar Robot - Sensor Tracker v3.0      ║
    ║  IMU + GPSD → MQTT Publisher             ║
    ╚══════════════════════════════════════════╝
    """)
    
    tracker = SensorTracker()
    
    # Attempt MQTT connection
    print("Connecting to MQTT broker...")
    tracker.mqtt.connect()
    
    if tracker.imu.connected or tracker.gps.connected:
        tracker.run()
    else:
        print("✗ No sensors initialized. Check connections and try again.")


if __name__ == "__main__":
    main()
