#!/usr/bin/env python3
"""
Optimized Raspberry Pi IMU/GPS Position Tracker
Efficient sensor fusion with dead reckoning at 5Hz
Minimal memory footprint and CPU usage
"""

import json
import time
import math
from datetime import datetime
import paho.mqtt.client as mqtt

try:
    from adafruit_bno055 import Adafruit_BNO055
    import busio
    import board
    BNO055_AVAILABLE = True
except ImportError:
    BNO055_AVAILABLE = False

try:
    import serial
    GPS_AVAILABLE = True
except ImportError:
    GPS_AVAILABLE = False

# Constants (precomputed)
EARTH_RADIUS = 6371000
DEG_TO_RAD = math.pi / 180
PUBLISH_INTERVAL = 0.2  # 5Hz
GPS_TIMEOUT = 5  # seconds without GPS fix before increasing filter trust in IMU

class KalmanFilter:
    """Minimal Kalman filter - only essential computations"""
    __slots__ = ('x', 'p', 'q', 'r', 'k')
    
    def __init__(self, q, r, x0=0):
        self.x = x0
        self.p = 1.0
        self.q = q
        self.r = r
        self.k = 0
    
    def update(self, z):
        self.p += self.q
        self.k = self.p / (self.p + self.r)
        self.x += self.k * (z - self.x)
        self.p *= (1 - self.k)
        return self.x


class GPSParser:
    """Fast NMEA parser - only processes GGA sentences"""
    __slots__ = ('lat', 'lon', 'alt', 'sats', 'fix', 'updated')
    
    def __init__(self):
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.sats = 0
        self.fix = 0
        self.updated = False
    
    def parse(self, line):
        """Parse GPGGA sentence only (most useful for position)"""
        if not line.startswith("$GPGGA"):
            return False
        
        try:
            parts = line.split(',')
            if len(parts) > 9 and parts[2] and parts[4] and parts[6]:
                # Parse latitude (DDMM.MMMMM format)
                lat_val = float(parts[2])
                self.lat = int(lat_val / 100) + (lat_val % 100) / 60
                if parts[3] == 'S':
                    self.lat = -self.lat
                
                # Parse longitude (DDDMM.MMMMM format)
                lon_val = float(parts[4])
                self.lon = int(lon_val / 100) + (lon_val % 100) / 60
                if parts[5] == 'W':
                    self.lon = -self.lon
                
                # Parse altitude
                self.alt = float(parts[9]) if parts[9] else 0
                
                # Parse fix quality and satellites
                self.fix = int(parts[6])
                self.sats = int(parts[7]) if parts[7] else 0
                
                self.updated = True
                return True
        except (ValueError, IndexError):
            pass
        
        return False


class IMUTracker:
    """Efficient IMU-based dead reckoning"""
    __slots__ = ('heading', 'roll', 'pitch', 'vel', 'ax', 'ay', 'dt', 'last_t', 'imu')
    
    def __init__(self, imu=None):
        self.heading = 0
        self.roll = 0
        self.pitch = 0
        self.vel = 0
        self.ax = 0
        self.ay = 0
        self.dt = 0
        self.last_t = time.time()
        self.imu = imu
    
    def update(self):
        """Update IMU state - called every iteration"""
        t = time.time()
        self.dt = t - self.last_t
        self.last_t = t
        
        if not self.imu:
            return False
        
        try:
            euler = self.imu.read_euler()
            if euler:
                self.heading = euler[0]
                self.roll = euler[1]
                self.pitch = euler[2]
            
            accel = self.imu.read_linear_acceleration()
            if accel:
                self.ax = accel[0]
                self.ay = accel[1]
                # Update velocity with exponential decay
                a_mag = math.sqrt(accel[0]**2 + accel[1]**2)
                self.vel = (self.vel + a_mag * self.dt) * 0.92
            
            return True
        except Exception:
            return False
    
    def get_dead_reckon(self, x, y):
        """Calculate dead reckoning displacement"""
        if self.vel < 0.01 or self.dt <= 0:
            return x, y
        
        h_rad = self.heading * DEG_TO_RAD
        dx = self.vel * math.sin(h_rad) * self.dt
        dy = self.vel * math.cos(h_rad) * self.dt
        
        return x + dx, y + dy


class Tracker:
    """Main position tracker - minimal state"""
    __slots__ = ('imu', 'gps_ser', 'gps_parser', 'kf_x', 'kf_y', 'kf_alt',
                 'ref_lat', 'ref_lon', 'last_gps_t', 'mqtt', 'imu_tracker')
    
    def __init__(self):
        self.imu = None
        self.gps_ser = None
        self.gps_parser = GPSParser()
        self.imu_tracker = None
        
        # Kalman filters (q=process noise, r=measurement noise)
        self.kf_x = KalmanFilter(0.005, 0.15)
        self.kf_y = KalmanFilter(0.005, 0.15)
        self.kf_alt = KalmanFilter(0.05, 2.0)
        
        self.ref_lat = 0
        self.ref_lon = 0
        self.last_gps_t = 0
        
        self._init_sensors()
        self.mqtt = MQTTClient()
        self.mqtt.connect()
    
    def _init_sensors(self):
        """Initialize sensors"""
        if BNO055_AVAILABLE:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.imu = Adafruit_BNO055.Adafruit_BNO055_I2C(i2c=i2c)
                self.imu_tracker = IMUTracker(self.imu)
                print("✓ IMU ready")
            except Exception as e:
                print(f"✗ IMU failed: {e}")
        
        if GPS_AVAILABLE:
            try:
                self.gps_ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)
                print("✓ GPS ready")
            except Exception:
                try:
                    self.gps_ser = serial.Serial("/dev/ttyAMA0", 9600, timeout=0.5)
                    print("✓ GPS ready (UART)")
                except Exception as e:
                    print(f"✗ GPS failed: {e}")
    
    def _gps_to_local(self, lat, lon):
        """Fast GPS to local coordinate conversion"""
        if self.ref_lat == 0:
            return 0, 0
        
        dlat = (lat - self.ref_lat) * DEG_TO_RAD
        dlon = (lon - self.ref_lon) * DEG_TO_RAD
        
        y = dlat * EARTH_RADIUS
        x = dlon * EARTH_RADIUS * math.cos(self.ref_lat * DEG_TO_RAD)
        
        return x, y
    
    def read_gps(self):
        """Non-blocking GPS read"""
        if not self.gps_ser:
            return
        
        try:
            if self.gps_ser.in_waiting:
                line = self.gps_ser.readline().decode('utf-8', errors='ignore').strip()
                if self.gps_parser.parse(line):
                    self.last_gps_t = time.time()
                    if self.ref_lat == 0:
                        self.ref_lat = self.gps_parser.lat
                        self.ref_lon = self.gps_parser.lon
        except Exception:
            pass
    
    def update_and_publish(self):
        """Single update-publish cycle"""
        # Update IMU (always)
        if self.imu_tracker:
            self.imu_tracker.update()
        
        # Read GPS (non-blocking)
        self.read_gps()
        
        # Perform sensor fusion
        x, y = 0, 0
        
        # Dead reckoning
        if self.imu_tracker:
            x, y = self.imu_tracker.get_dead_reckon(x, y)
            x = self.kf_x.update(x)
            y = self.kf_y.update(y)
        
        # GPS correction (when available)
        if self.gps_parser.fix > 0:
            gps_x, gps_y = self._gps_to_local(self.gps_parser.lat, self.gps_parser.lon)
            x = self.kf_x.update(gps_x)
            y = self.kf_y.update(gps_y)
        
        # Build minimal data packet
        data = {
            "t": datetime.now().isoformat(),
            "lat": round(self.gps_parser.lat, 8),
            "lon": round(self.gps_parser.lon, 8),
            "alt": round(self.gps_parser.alt, 2),
            "x": round(x, 3),
            "y": round(y, 3),
            "h": round(self.imu_tracker.heading if self.imu_tracker else 0, 1),
            "v": round(self.imu_tracker.vel if self.imu_tracker else 0, 3),
            "fix": self.gps_parser.fix,
            "sats": self.gps_parser.sats
        }
        
        self.mqtt.publish("jaguar/pos", data)
        
        # Console output
        status = "GPS" if self.gps_parser.fix > 0 else "IMU"
        print(f"[{status}] X:{data['x']:7.2f}m Y:{data['y']:7.2f}m H:{data['h']:6.1f}° V:{data['v']:5.3f}m/s")


class MQTTClient:
    """Minimal MQTT client"""
    __slots__ = ('client', 'connected')
    
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = lambda c, u, f, r: setattr(self, 'connected', r == 0)
        self.connected = False
    
    def connect(self):
        try:
            self.client.connect("localhost", 1883, keepalive=60)
            self.client.loop_start()
            time.sleep(1)
            if self.connected:
                print("✓ MQTT connected")
        except Exception as e:
            print(f"✗ MQTT failed: {e}")
    
    def publish(self, topic, data):
        if self.connected:
            self.client.publish(topic, json.dumps(data), qos=0)
    
    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()


def main():
    print("Jaguar Tracker v2 (Optimized)\n")
    
    tracker = Tracker()
    last_pub = time.time()
    
    try:
        while True:
            now = time.time()
            
            # Publish at 5Hz (200ms intervals)
            if now - last_pub >= PUBLISH_INTERVAL:
                tracker.update_and_publish()
                last_pub = now
            
            # Sleep only what's needed
            time.sleep(0.001)
    
    except KeyboardInterrupt:
        print("\nShutdown")
        tracker.mqtt.disconnect()
        if tracker.gps_ser:
            tracker.gps_ser.close()


if __name__ == "__main__":
    main()
