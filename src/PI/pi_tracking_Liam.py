#!/usr/bin/env python3
"""
Raspberry Pi IMU/GPS Position Tracker with TCP Socket Publisher
Sends IMU heading, acceleration, and GPS data via TCP to Flask controller
Simplified version without MQTT
"""

import json
import time
import math
import socket
import sys

try:
    from adafruit_bno055 import Adafruit_BNO055
    import busio
    import board
    BNO055_AVAILABLE = True
except ImportError:
    BNO055_AVAILABLE = False
    print("⚠ Warning: BNO055 not available (running in simulation mode)")

try:
    from gps3 import agps3
    GPSD_AVAILABLE = True
except ImportError:
    GPSD_AVAILABLE = False
    print("⚠ Warning: gps3 (GPSD) not available (running in simulation mode)")

# Constants
PUBLISH_INTERVAL = 0.2  # 5Hz publishing rate
FLASK_SERVER = "192.168.0.103"  # Flask server IP - CHANGE THIS TO YOUR JAGUAR IP
FLASK_PORT = 5015  # TCP port for sensor data


class GPSdaemon:
    """GPSD client for GPS data (non-blocking)"""
    def __init__(self, host='127.0.0.1', port=2947):
        self.host = host
        self.port = port
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.sats = 0
        self.fix = 0
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
        """Non-blocking read from GPSD"""
        if not self.connected:
            return False
        
        try:
            for new_data in self.socket:
                if new_data:
                    self.stream.unpack(new_data)
                    
                    if self.stream.TPV['mode'] >= 2:
                        self.lat = self.stream.TPV.get('lat', 0.0)
                        self.lon = self.stream.TPV.get('lon', 0.0)
                        self.alt = self.stream.TPV.get('alt', 0.0)
                        self.fix = self.stream.TPV['mode']
                        
                        if hasattr(self.stream, 'SKY'):
                            self.sats = self.stream.SKY.get('nSat', 0)
                        
                        return True
            
            return False
        except Exception as e:
            print(f"✗ Error reading GPSD: {e}")
            self.connected = False
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
        if not self.connected:
            return False
        
        try:
            euler = self.imu.read_euler()
            if euler:
                self.heading = float(euler[0])
                self.roll = float(euler[1])
                self.pitch = float(euler[2])
            
            accel = self.imu.read_linear_acceleration()
            if accel:
                self.acc_x = float(accel[0])
                self.acc_y = float(accel[1])
                self.acc_z = float(accel[2])
            
            return True
        except Exception as e:
            print(f"✗ Error reading IMU: {e}")
            self.connected = False
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


class TCPPublisher:
    """TCP Socket client for publishing sensor data"""
    def __init__(self, server=FLASK_SERVER, port=FLASK_PORT):
        self.server = server
        self.port = port
        self.socket = None
        self.connected = False
        self.reconnect_delay = 1
        self.max_reconnect_delay = 30
        self.connect()
    
    def connect(self):
        """Connect to Flask server via TCP"""
        try:
            if self.socket:
                self.socket.close()
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.server, self.port))
            self.socket.settimeout(None)
            self.connected = True
            self.reconnect_delay = 1
            print(f"✓ TCP connected to {self.server}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ TCP connection failed: {e}")
            self.connected = False
            self.socket = None
            return False
    
    def publish_data(self, imu_data, gps_data):
        """Publish combined IMU and GPS data via TCP socket"""
        if not self.connected:
            if not self.connect():
                return False
        
        try:
            payload = {**imu_data, **gps_data}
            message = json.dumps(payload) + "\n"
            self.socket.sendall(message.encode('utf-8'))
            return True
        except Exception as e:
            print(f"✗ Error sending data: {e}")
            self.connected = False
            
            # Try to reconnect with exponential backoff
            time.sleep(self.reconnect_delay)
            self.reconnect_delay = min(self.reconnect_delay * 2, self.max_reconnect_delay)
            return False
    
    def disconnect(self):
        """Gracefully disconnect from server"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.connected = False


class SensorTracker:
    """Main tracker coordinating IMU, GPS, and TCP publishing"""
    def __init__(self):
        print("Initializing Jaguar Sensor Tracker (TCP Mode)...")
        print("-" * 50)
        print(f"Target: {FLASK_SERVER}:{FLASK_PORT}")
        print("-" * 50)
        
        self.imu = IMUSensor()
        self.gps = GPSdaemon()
        self.tcp = TCPPublisher()
        
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
                
                # Update GPS at same rate as publish
                if now - self.last_gps_read >= PUBLISH_INTERVAL:
                    self.gps.update()
                    self.last_gps_read = now
                
                # Publish at configured interval
                if now - self.last_publish >= PUBLISH_INTERVAL:
                    imu_data = self.imu.get_data()
                    gps_data = self.gps.get_data()
                    
                    if self.tcp.publish_data(imu_data, gps_data):
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
                
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            self.shutdown()
    
    def shutdown(self):
        """Clean shutdown"""
        print("\n" + "-" * 50)
        print("Shutting down...")
        self.tcp.disconnect()
        print(f"Published: {self.publish_count} messages")
        print(f"Errors: {self.error_count}")
        print("Goodbye!")


def main():
    print("""
    ╔══════════════════════════════════════════╗
    ║  Jaguar Robot - Sensor Tracker v4.0      ║
    ║  IMU + GPSD → TCP Publisher              ║
    ║  Simplified & Fixed Edition              ║
    ╚══════════════════════════════════════════╝
    """)
    
    tracker = SensorTracker()
    
    # Run even if sensors aren't available (for testing)
    tracker.run()


if __name__ == "__main__":
    main()
