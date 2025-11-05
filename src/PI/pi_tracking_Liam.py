#!/usr/bin/env python3
"""
Raspberry Pi IMU/GPS Position Tracker with MQTT Publisher
Sends IMU heading, acceleration, and GPS data to Jaguar controller
Publishes to: IMU/data topic for Flask backend consumption
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
    import serial
    GPS_AVAILABLE = True
except ImportError:
    GPS_AVAILABLE = False
    print("Warning: pyserial not available")

# Constants
EARTH_RADIUS = 6371000
DEG_TO_RAD = math.pi / 180
PUBLISH_INTERVAL = 0.2  # 5Hz publishing rate
MQTT_BROKER = "192.168.1.103"  # Change to your broker IP
MQTT_PORT = 1883
MQTT_TOPIC_IMU = "IMU/data"
MQTT_TOPIC_GPS = "GPS/data"

class GPSParser:
    """Fast NMEA parser - processes GGA sentences for position data"""
    def __init__(self):
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.sats = 0
        self.fix = 0
        self.last_valid = time.time()
    
    def parse(self, line):
        """Parse GPGGA sentence: $GPGGA,time,lat,lat_dir,lon,lon_dir,fix,sats,hdop,alt,alt_unit"""
        if not line.startswith("$GPGGA"):
            return False
        
        try:
            parts = line.split(',')
            if len(parts) < 10:
                return False
            
            # Check if we have valid fix and data
            fix_quality = int(parts[6]) if parts[6] else 0
            if fix_quality == 0:
                return False
            
            # Parse latitude (DDMM.MMMMM format)
            if parts[2] and parts[3]:
                lat_val = float(parts[2])
                self.lat = int(lat_val / 100) + (lat_val % 100) / 60
                if parts[3] == 'S':
                    self.lat = -self.lat
            
            # Parse longitude (DDDMM.MMMMM format)
            if parts[4] and parts[5]:
                lon_val = float(parts[4])
                self.lon = int(lon_val / 100) + (lon_val % 100) / 60
                if parts[5] == 'W':
                    self.lon = -self.lon
            
            # Parse altitude and satellite count
            self.alt = float(parts[9]) if parts[9] else 0.0
            self.sats = int(parts[7]) if parts[7] else 0
            self.fix = fix_quality
            self.last_valid = time.time()
            
            return True
        except (ValueError, IndexError) as e:
            return False


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


class GPSSensor:
    """Interface to GPS module via serial"""
    def __init__(self, port="/dev/ttyUSB0", baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.parser = GPSParser()
        self.connected = False
        self._init_gps()
    
    def _init_gps(self):
        """Initialize GPS serial connection"""
        if not GPS_AVAILABLE:
            return
        
        # Try multiple port configurations
        ports = ["/dev/ttyUSB0", "/dev/ttyAMA0", "/dev/serial0"]
        
        for p in ports:
            try:
                self.ser = serial.Serial(p, self.baudrate, timeout=0.5)
                self.connected = True
                print(f"✓ GPS connected on {p}")
                return
            except Exception:
                continue
        
        if not self.connected:
            print("✗ GPS initialization failed on all ports")
    
    def update(self):
        """Non-blocking read of GPS data"""
        if not self.connected or not self.ser:
            return False
        
        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line and self.parser.parse(line):
                    return True
        except Exception as e:
            print(f"Error reading GPS: {e}")
        
        return False
    
    def get_data(self):
        """Return current GPS data as dictionary"""
        return {
            'Lat': round(self.parser.lat, 8),
            'gps:Lat': round(self.parser.lat, 8),
            'Lon': round(self.parser.lon, 8),
            'Alt': round(self.parser.alt, 2),
            'Sats': self.parser.sats,
            'Fix': self.parser.fix
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
    
    def publish_imu(self, imu_data, gps_data):
        """Publish combined IMU and GPS data to IMU/data topic"""
        if not self.connected:
            return False
        
        try:
            # Combine IMU and GPS data into single payload
            payload = {**imu_data, **gps_data}
            self.client.publish(MQTT_TOPIC_IMU, json.dumps(payload), qos=1)
            return True
        except Exception as e:
            print(f"Error publishing IMU data: {e}")
            return False
    
    def publish_gps(self, gps_data):
        """Publish GPS data to separate GPS/data topic"""
        if not self.connected:
            return False
        
        try:
            self.client.publish(MQTT_TOPIC_GPS, json.dumps(gps_data), qos=1)
            return True
        except Exception as e:
            print(f"Error publishing GPS data: {e}")
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
        self.gps = GPSSensor()
        self.mqtt = MQTTPublisher()
        
        self.last_publish = time.time()
        self.publish_count = 0
        self.error_count = 0
    
    def run(self):
        """Main loop - update sensors and publish"""
        print("\nStarting sensor tracking (5Hz)...")
        print("Press Ctrl+C to stop\n")
        
        try:
            while True:
                now = time.time()
                
                # Update sensors
                self.imu.update()
                self.gps.update()
                
                # Publish at configured interval
                if now - self.last_publish >= PUBLISH_INTERVAL:
                    imu_data = self.imu.get_data()
                    gps_data = self.gps.get_data()
                    
                    if self.mqtt.publish_imu(imu_data, gps_data):
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
        if self.gps.connected and self.gps.ser:
            self.gps.ser.close()
        print(f"Published: {self.publish_count} messages")
        print(f"Errors: {self.error_count}")
        print("Goodbye!")


def main():
    print("""
    ╔══════════════════════════════════════════╗
    ║  Jaguar Robot - Sensor Tracker v2.0      ║
    ║  IMU + GPS → MQTT Publisher              ║
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
