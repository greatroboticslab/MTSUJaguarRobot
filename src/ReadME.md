Jaguar Robot Control System
A comprehensive robot control system for a Jaguar robot platform featuring GPS/IMU sensor fusion, autonomous navigation, camera integration, and web-based control interface.
System Architecture
This system consists of multiple components that work together to provide real-time robot control and monitoring:
📁 Project Structure
├── PI/                          # Raspberry Pi sensor collection
│   ├── harry_SensorImproved.py # IMU/GPS data publisher
│   └── [sensor scripts]
│
├── appVersions/                 # Flask application versions
│   ├── app.py                  # Original version
│   ├── appUpdated.py           # Improved sensor fusion
│   ├── appAutoNav.py           # + Autonomous navigation
│   └── appAutoCapture.py       # + Auto photo capture
│
├── static/                      # Web assets
│   ├── robot.png               # Robot icon for map
│   └── [images]
│
└── templates/                   # HTML templates
    ├── index.html              # Main map & control interface
    └── camera.html             # Camera view page
Components
🔧 Raspberry Pi Sensor Module (PI/)
Purpose: Collects and publishes sensor data from the robot's Raspberry Pi
Key Features:

BNO055 IMU Integration: 9-DOF inertial measurement unit

Heading, roll, pitch (Euler angles)
Linear acceleration (gravity compensated)


GPS Module Support: NMEA sentence parsing (GPGGA)

Position (latitude, longitude, altitude)
Fix quality and satellite count


MQTT Publishing: Real-time data streaming at 5Hz

Topic: IMU/data (combined IMU + GPS)
Topic: GPS/data (GPS only)



Requirements:
pythonadafruit-circuitpython-bno055
paho-mqtt
pyserial
Configuration:
pythonMQTT_BROKER = "192.168.1.103"  # Update with your broker IP
GPS_PORT = "/dev/ttyUSB0"       # Update with your GPS port

🌐 Flask Application Versions (appVersions/)
All versions provide:

Real-time GPS/IMU data visualization on interactive map
Manual robot control (WASD keyboard controls)
PID parameter tuning interface
MQTT data integration
Socket-based robot communication

app.py - Base Version
Basic functionality with manual control and data display
appUpdated.py - Enhanced Sensor Fusion

Weighted fusion of GPS and IMU data
Dynamic heading offset correction
Friction detection and compensation
Static state detection
Improved accuracy over distance

Key Algorithms:

Haversine distance calculation
Bearing computation with offset correction
Weighted position/heading fusion
Dead reckoning fallback

appAutoNav.py - Autonomous Navigation
Previous features +

Automatic forward/backward cycling
Configurable cycle duration and speed
Enhanced motor command persistence
Movement monitoring and verification

Navigation Features:

Press 'N' or click button to toggle
20-second forward → 1-second pause → 20-second backward (configurable)
Smooth acceleration/deceleration
Automatic stop on navigation end

appAutoCapture.py - Auto Photo Capture
Previous features +

Automatic photo capture at intervals
Local storage of captured images
Photo management (list, delete)
Capture status indicators

Camera Features:

3-second auto-capture interval
Photos saved with timestamps
Browser-based photo gallery
Capture during navigation


🎮 Web Interface (templates/)
index.html - Main Control Interface
Features:

Interactive Map (Leaflet.js)

Real-time robot position tracking
GPS accuracy circle visualization
Robot heading indicator with rotation
Offline grid layer fallback
Heading lock mode (map rotates with robot)


Waypoint Navigation

Click to set waypoints
Distance calculation between points
1-meter radius route interpolation
PID-based autonomous following
Visual path display with labels


Control Panels

PID tuning (Kp, Ki, Kd)
Speed adjustment
Navigation duration config
Data visualization panel
Camera feed integration


Keyboard Controls

W - Forward
S - Backward
A - Turn left
D - Turn right
N - Toggle autonomous navigation
Space - Emergency stop


Compass Display

Visual heading indicator
360° rotation with IMU data



camera.html - Camera View
Simple full-screen camera feed with back navigation

Setup & Installation
1. Raspberry Pi Setup
bash# Install dependencies
pip3 install adafruit-circuitpython-bno055 paho-mqtt pyserial

# Run sensor publisher
cd PI/
python3 harry_SensorImproved.py
2. Server Setup (Computer/Main Controller)
bash# Install Flask dependencies
pip install flask paho-mqtt opencv-python numpy

# Choose your version and run
cd appVersions/
python3 appAutoCapture.py  # Latest version with all features
3. MQTT Broker
Ensure MQTT broker is running and accessible:
bash# On Ubuntu/Debian
sudo apt-get install mosquitto mosquitto-clients
sudo systemctl start mosquitto
4. Robot Configuration
Update IP addresses in code:
python# In app*.py
robot = RobotSocket('192.168.0.60', 10001)  # Robot IP
camera_url = "http://root:password@192.168.0.65:8081/..."  # Camera IP

# In harry_SensorImproved.py
MQTT_BROKER = "192.168.1.103"  # MQTT Broker IP

Usage
Basic Operation

Start the system:

Launch MQTT broker
Run harry_SensorImproved.py on Raspberry Pi
Run Flask app on main computer
Access web interface: http://localhost:5014


Connect Robot:

Click "Connect Robot" button
Button turns green when connected


Manual Control:

Use WASD keys or buttons to control robot
View real-time position on map



Autonomous Navigation

Set Waypoints:

Click "Point" button to enable waypoint mode
Click on map to place waypoints
Distance shown between points


Tune PID:

Click "PID" button
Adjust Kp, Ki, Kd values
Set desired speed and navigation duration


Start Navigation:

Click "Go" to begin autonomous following
Robot follows waypoints using PID control
Click "Stop" to halt



Photo Capture (appAutoCapture.py only)

Start Camera:

Click "Camera" button
Camera feed appears


Capture Photos:

Click "Photo" to start auto-capture
Photos saved every 3 seconds
Button shows "Stop Capturing" when active


Manage Photos:

View captured photos in captured_photos/ directory
Use API endpoints to list/delete photos




API Endpoints
Robot Control

POST /connect-robot - Connect to robot
POST /robot-cmd - Send movement command (w/a/s/d/stop/n)
POST /robot-autopilot - Autopilot actions

Data & Configuration

GET /data - Get current sensor data
GET /mqtt-data - Get recent MQTT messages
POST /pid-update - Update PID parameters
POST /navigation-config - Update navigation settings
GET /navigation-status - Get navigation state

Camera (appAutoCapture.py)

POST /toggle-camera - Start/stop camera
GET /camera-stream - MJPEG stream
POST /capture-photo - Capture single photo
GET /photos - List all photos
DELETE /delete-photo/<filename> - Delete photo


Technical Details
Sensor Fusion Algorithm
The system uses a weighted fusion approach combining:

GPS data (reliable position, slow update)
IMU data (fast heading, subject to drift)

Dynamic Correction:

Monitors consistent heading over 5+ meters
Calculates heading offset between GPS and IMU
Applies trust ratio based on travel distance (up to 100m)
Adjusts IMU heading with learned offset

Static Detection:

Position change < 2m → consider stationary
Acceleration < 100 → zero out acceleration values
Prevents drift accumulation when stopped

Navigation System
Enhanced Movement Control:

Command persistence every 500ms
Multiple stop commands for reliability
Speed ramping for smooth transitions
Movement verification via GPS

Waypoint Following:

1-meter route interpolation between points
PID control for heading correction
Dynamic speed adjustment based on error
Arrival detection within 1-meter radius


Troubleshooting
GPS Not Locking:

Ensure GPS module has clear sky view
Check serial port configuration
Wait 30-60 seconds for initial fix

IMU Calibration:

Move robot in figure-8 pattern
Calibration status in console output
May take 30 seconds after startup

MQTT Connection Failed:

Verify broker IP address
Check firewall settings
Ensure mosquitto service is running

Robot Not Responding:

Check IP address and port
Verify network connectivity
Look for socket timeout errors in console

Camera Stream Issues:

Verify camera URL and credentials
Check network bandwidth
Try stopping/starting camera feed


Safety Notes
⚠️ Important:

Always monitor robot during autonomous operation
Keep emergency stop (Space) ready
Test in open area first
Laser power set to 2W by default (safety)
Adjust speeds cautiously when tuning PID
