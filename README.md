# Robot Control System

A comprehensive web-based robot control application built with Flask that provides real-time control, navigation, and monitoring capabilities for a robotic system.

## 🚀 Quick Start (30 seconds)

**Option 1: Automated Setup (Recommended)**
```bash
cd /path/to/MTSUJaguarRobot
./setup/setup.sh          # Setup everything
./setup/setup.sh start     # Setup and start app automatically
```

**Option 2: Simple Start (If already set up)**
```bash
cd /path/to/MTSUJaguarRobot
./start.sh                 # Quick start script
```

**Option 3: Manual Setup**
```bash
# 1. Setup environment
cd /path/to/MTSUJaguarRobot
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# 2. Test setup (optional)
python3 setup/test_setup.py

# 3. Start application
python3 src/app.py

# 4. Open browser
# Navigate to: http://127.0.0.1:5014
```

✅ **No code changes required** - works out of the box on any machine with Python 3.10+

## Features

### 🚀 Core Functionality
- **Real-time Robot Control** - WASD keyboard controls for movement
- **Rate-Limited Commands** - Prevents command backlog and reduces latency
- **Auto-Stop Safety** - Automatically stops robot when no commands are received
- **Live Camera Feed** - Stream video from robot's IP camera
- **MQTT Integration** - Real-time data from IMU, GPS, and sensors
- **Navigation Mode** - Autonomous back-and-forth movement
- **Data Fusion** - Advanced GPS/IMU sensor fusion for accurate positioning

### 🎮 Control Interface
- **W/A/S/D** - Forward/Left/Backward/Right movement
- **Space/Stop** - Emergency stop
- **N** - Toggle navigation mode
- **Rate Limiting** - 100ms minimum delay between commands
- **Auto-Stop** - Stops robot 300ms after last movement command

### 📡 Data Processing
- **IMU Data** - Real-time heading, acceleration readings
- **GPS Tracking** - Latitude/longitude positioning
- **Sensor Fusion** - Weighted combination of GPS and IMU data
- **Dynamic Calibration** - Automatic heading offset correction
- **Friction Detection** - Adaptive speed adjustment for stuck conditions

### 📹 Camera System
- **Live Streaming** - Real-time MJPEG video feed
- **Camera Control** - Start/stop camera stream
- **IP Camera Support** - Axis camera integration (192.168.0.65)

## Installation

### Prerequisites
- Python 3.10+ 
- Virtual environment (recommended)

### Setup
1. Clone or download the project:
   ```bash
   cd /path/to/MTSUJaguarRobot
   ```

2. Create and activate a Python virtual environment (recommended):
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate  # On macOS/Linux
   # or on Windows: .venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Configure network settings (if needed):
   ```bash
   python3 setup/network_config.py robot    # Switch to robot network
   python3 setup/network_config.py internet # Switch to internet
   python3 setup/network_config.py status   # Check current network
   ```

## Getting Started - Step by Step

### Prerequisites Verification
Before starting, ensure you have:
- **Python 3.10 or higher** - Check with `python3 --version`
- **Network access** - Either internet or robot network
- **Web browser** - Any modern browser (Chrome, Firefox, Safari, Edge)

### Quick Start (3 Steps)

#### Step 1: Setup the Environment
```bash
# Navigate to project directory
cd /path/to/MTSUJaguarRobot

# Create virtual environment (recommended)
python3 -m venv .venv
source .venv/bin/activate

# Install required packages
pip install -r requirements.txt
```

**Expected output:** You should see successful installation messages for Flask, paho-mqtt, and opencv-python.

#### Step 2: Start the Application
```bash
python3 app.py
```

**Expected output:**
```
Auto-stop monitor started
 * Serving Flask app 'app'
 * Debug mode: on
 * Running on all addresses (0.0.0.0)
 * Running on http://127.0.0.1:5014
 * Running on http://[your-ip]:5014
 * Debugger is active!
MQTT connection error: timed out, retrying in 1 seconds...
```

**Note:** The MQTT connection errors are normal when not connected to the robot network.

#### Step 2 (Alternative): Test Your Setup First
Before starting the main application, you can run the setup verification script:

```bash
python3 test_setup.py
```

This will check:
- Python version compatibility
- All dependencies are installed
- Required files exist
- Port 5014 is available
- App can be imported successfully

**Expected output:**
```
🚀 Robot Control System Setup Test
========================================
Testing Python version...
✅ Python 3.13.2 is supported

Testing dependencies...
✅ flask is installed
✅ paho-mqtt is installed
✅ opencv-python is installed

...

🎉 ALL TESTS PASSED!
Your setup is ready. You can now run:
   python3 src/app.py
```

#### Step 3: Access the Web Interface
1. Open your web browser
2. Navigate to: `http://127.0.0.1:5014`
3. You should see the robot control interface with a map

### Testing Different Scenarios

#### Scenario A: Development/Testing Mode (No Robot Hardware)
This is the default mode - perfect for code development and UI testing.

**What works:**
- ✅ Web interface loads correctly
- ✅ All buttons and controls are functional
- ✅ Camera page accessible (will show no feed)
- ✅ API endpoints respond correctly

**What doesn't work:**
- ⚠️ MQTT connection (robot data) - expected timeout errors
- ⚠️ Robot movement commands - no physical robot
- ⚠️ Camera stream - no camera access

**This is perfect for:**
- Testing the web interface
- Developing new features
- Learning the codebase
- Demonstrating the UI to others

#### Scenario B: Connected to Robot Network
Only needed when controlling actual robot hardware.

1. **Connect to Robot WiFi Network:**
   - SSID: `DirJaguar`
   - Password: `drrobotdrrobot`

2. **Configure Network Settings:**
   ```bash
   python3 setup/network_config.py robot
   ```

3. **Start the Application:**
   ```bash
   python3 src/app.py
   ```

**Expected differences:**
- MQTT connections should succeed
- Robot movement commands work
- Camera feed available
- Real sensor data displayed

### Troubleshooting Setup Issues

#### Issue: "python3: command not found"
**Solution:**
```bash
# Try these alternatives:
python --version    # Check if Python is available as 'python'
py --version        # On Windows
# Install Python from python.org if needed
```

#### Issue: "Permission denied" when installing packages
**Solutions:**
```bash
# Option 1: Use virtual environment (recommended)
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# Option 2: Install with user flag
pip3 install --user -r requirements.txt

# Option 3: Use system package manager (macOS)
brew install python3
```

#### Issue: "Address already in use" (Port 5014)
**Solution:**
```bash
# Find and kill existing process
lsof -i :5014
kill -9 <PID_NUMBER>

# Or use a different port by editing app.py (last line)
# Change: app.run(host='0.0.0.0', port=5014, debug=True)
# To:     app.run(host='0.0.0.0', port=5015, debug=True)
```

#### Issue: "No module named 'cv2'" after installing opencv-python
**Solution:**
```bash
# Try alternative opencv installation
pip uninstall opencv-python
pip install opencv-python-headless

# Or for full opencv with GUI support
pip install opencv-contrib-python
```

### Verification Checklist

After completing setup, verify everything works:

- [ ] **Python Environment:** `python3 --version` shows 3.10+
- [ ] **Dependencies:** `pip list` shows flask, paho-mqtt, opencv-python
- [ ] **App Starts:** Running `python3 src/app.py` shows Flask server starting
- [ ] **Web Interface:** `http://127.0.0.1:5014` loads the control interface
- [ ] **No Critical Errors:** Only MQTT timeout messages (acceptable without robot)

### Success Indicators

**✅ Setup is successful when you see:**
1. Flask app running on http://127.0.0.1:5014
2. Web interface loads with map and controls
3. Console shows "Auto-stop monitor started"
4. Browser can access the control interface

**⚠️ These are normal/expected in development mode:**
- MQTT connection timeout errors
- "No camera feed" in camera view
- Robot commands not affecting physical hardware

### Next Steps After Setup

1. **Explore the Interface:** Try all the buttons and controls
2. **Read the Documentation:** Review the features and controls below
3. **Test API Endpoints:** Visit `/data` and `/mqtt-data` endpoints
4. **Camera View:** Check `/camera` page
5. **Network Configuration:** Test `network_config.py status`

## Usage

### Starting the Application
```bash
python3 app.py
```

The application will start on:
- **Local**: http://127.0.0.1:5014
- **Network**: http://[your-ip]:5014

### Web Interface
1. **Main Control Panel** (`/`) - Primary robot control interface
2. **Camera View** (`/camera`) - Live video feed
3. **Data API** (`/data`) - JSON endpoint for sensor data

### Getting Started with Robot Control

**Step 1: Open the Web Interface**
1. After starting the application, open your web browser
2. Navigate to the provided address (e.g., http://127.0.0.1:5014)
3. You'll see the main control panel interface

**Step 2: Connect to Robot**
1. Click the **"Connect"** button in the web UI
2. Wait for successful connection confirmation
3. The interface will indicate when the robot is ready

**Step 3: Control the Robot**
1. Use **WASD keys** to move the robot:
   - **W** - Move Forward
   - **A** - Turn Left  
   - **S** - Move Backward
   - **D** - Turn Right
   - **Space** - Emergency Stop
2. The robot will respond immediately to key presses
3. Auto-stop engages 300ms after releasing keys

**Step 4: Adjust Speed Settings**
1. Locate the PID speed controls in the web interface
2. Adjust speed from **0-500** to control wheel power
   - Lower values (50-150): Slow, precise movement
   - Medium values (150-250): Normal operation
   - Higher values (250-500): Fast movement
3. Changes apply immediately to movement commands

**Step 5: Access Camera Feed**
1. Click the **"Camera"** button in the GUI
2. This opens the live camera view in a new page
3. Start/stop camera streaming as needed
4. Camera shows real-time view from robot's perspective

### Robot Commands
| Key | Action |
|-----|--------|
| W | Move Forward |
| S | Move Backward |
| A | Turn Left |
| D | Turn Right |
| Space | Stop |
| N | Toggle Navigation Mode |

### Network Configuration
The robot system requires specific network settings:

**Robot Network Mode:**
- IP: 192.168.0.109
- Robot: 192.168.0.60:10001
- Camera: 192.168.0.65:8081

**Internet Mode:**
- DHCP configuration
- DNS: 8.8.8.8, 8.8.4.4

#### Manual Network Configuration
If you need to configure the network manually instead of using the `network_config.py` script:

**WiFi Network:**
- **SSID**: DirJaguar
- **Password**: drrobotdrrobot

**IPv4 Settings (Robot Mode):**
- **Configure IPv4**: Manually
- **IP Address**: 192.168.0.109
- **Subnet Mask**: 255.255.255.0
- **Router**: 192.168.0.1
- **DNS Servers**: 192.168.0.1

**macOS Network Setup:**
1. Go to System Preferences → Network
2. Select your Wi-Fi interface
3. Click "Advanced" → TCP/IP tab
4. Set "Configure IPv4" to "Manually"
5. Enter the IP settings above
6. Click "OK" and "Apply"

## Configuration

### Robot Settings
```python
# In app.py
robot = RobotSocket('192.168.0.60', 10001)  # Robot IP and port
pid_params.speed = 150                       # Default movement speed
```

### Camera Settings
```python
camera_url = "http://root:drrobot@192.168.0.65:8081/axis-cgi/mjpg/video.cgi"
```

### MQTT Settings
```python
mqtt_broker_ip = "192.168.1.103"
mqtt_broker_port = 1883
```

### Rate Limiting
```python
command_delay = 0.1    # 100ms minimum between commands
stop_timeout = 0.3     # Auto-stop after 300ms of no input
```

## API Endpoints

### Control Endpoints
- `POST /robot-cmd` - Send movement commands
- `POST /connect-robot` - Establish robot connection
- `POST /robot-autopilot` - Autopilot commands
- `GET /navigation-status` - Get navigation mode status

### Data Endpoints
- `GET /data` - Current sensor data (IMU, GPS, fused)
- `GET /mqtt-data` - Raw MQTT message history

### Camera Endpoints
- `GET /camera-stream` - Live MJPEG video stream
- `POST /toggle-camera` - Start/stop camera

### Configuration Endpoints
- `POST /pid-update` - Update PID parameters

## Safety Features

### Rate Limiting
- Minimum 100ms delay between commands
- Prevents command queue buildup
- Reduces system latency during continuous input

### Auto-Stop System
- Monitors for movement command gaps
- Automatically sends stop commands after 300ms
- Multiple stop commands ensure reliability
- Prevents runaway robot behavior

### Connection Management
- Automatic robot reconnection
- MQTT retry with exponential backoff
- Socket timeout handling

## Technical Details

### Architecture
- **Flask Web Server** - Main application framework
- **Threading** - Concurrent data processing and monitoring
- **Socket Communication** - Direct TCP connection to robot
- **MQTT Protocol** - Sensor data streaming
- **OpenCV** - Camera stream processing

### Data Fusion Algorithm
1. **GPS/IMU Weighting** - Dynamic weight adjustment based on movement
2. **Heading Correction** - Automatic offset calibration over distance
3. **Static Detection** - Noise filtering when stationary
4. **Friction Compensation** - Speed boost for stuck conditions

### Command Protocol
Robot commands use MMW (Motor Move Wheel) protocol:
```
MMW !MG           # Motor go/enable
MMW !M <L> <R>    # Set left/right motor speeds
```

## Troubleshooting

### Common Issues

**Connection Problems:**
```bash
# Check robot network
python3 network_config.py status

# Switch to robot network
python3 network_config.py robot
```

**Port Already in Use:**
```bash
# Kill existing processes
lsof -i :5014
kill -9 <PID>
```

**Camera Not Loading:**
- Verify camera IP (192.168.0.65)
- Check network connectivity
- Ensure camera credentials are correct

**MQTT Connection Issues:**
- Verify MQTT broker IP (192.168.1.103)
- Check network connectivity
- Monitor console for connection status

### Debug Mode
The application runs in debug mode by default. Check the console output for:
- MQTT connection status
- Robot command responses
- Camera stream status
- Auto-stop notifications

## Development

### File Structure
```
MTSUJaguarRobot/
├── README.md               # Main documentation
├── requirements.txt        # Python dependencies
├── start.sh               # Quick start launcher
├── .venv/                 # Virtual environment (created by setup)
├── setup/                 # Setup and configuration scripts
│   ├── setup.sh          # Automated setup script
│   ├── test_setup.py     # Setup verification script
│   └── network_config.py # Network switching utility
├── src/                   # Main application source code
│   ├── app.py            # Main Flask application
│   ├── templates/        # HTML templates
│   │   ├── index.html   # Main control interface
│   │   └── camera.html  # Camera view
│   └── static/          # Static assets
│       ├── myhouse.osm  # Map data
│       └── robot.png    # Robot icon
└── docs/                 # Documentation
    └── SETUP_COMPLETE.md # Setup completion guide
```

### Adding Features
1. **New Sensors** - Add MQTT subscriptions in `on_connect()`
2. **Control Commands** - Extend `/robot-cmd` endpoint
3. **UI Elements** - Modify HTML templates
4. **Data Processing** - Update `process_data()` function

## License

This project is for educational and research purposes.

## Hardware Requirements

- Robot with MMW command protocol support
- IP camera (Axis or compatible)
- IMU sensor with MQTT output
- GPS module with MQTT output
- Network connectivity (Wi-Fi recommended)

---

For support or questions, please check the console output for debugging information and verify all network connections are properly configured.
