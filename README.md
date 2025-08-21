# Robot Control System

Web-based robot control application for MTSU Jaguar Robot with real-time navigation, camera feed, and sensor data fusion.

> 🔄 **Returning users:** Jump to [Already Set Up? Quick Start!](#already-set-up-quick-start-️) below

### **Youtube Setup Tutorial Video Link: https://youtu.be/fRm_X7DfuXw?si=GvWGdrgBQc8AHm_P**

## 🚀 Quick Setup

### First-Time Setup (New Machine)

**For Development/Testing (no robot hardware):**
```bash
cd /path/to/MTSUJaguarRobot
./setup/setup.sh start     # One command - sets up everything and starts app
```

**For Robot Control (with robot hardware):**
```bash
# 1. First setup on internet connection
cd /path/to/MTSUJaguarRobot
./setup/setup.sh           # Download dependencies while on internet

# 2. Switch to robot WiFi: DirJaguar / drrobotdrrobot
# 3. Configure robot network
python3 setup/network_config.py robot

# 4. Start controlling robot
./start.sh
```

### Already Set Up? Quick Start! 🏃‍♂️

**If this machine already has the robot code installed:**

1. **Open terminal** in project folder:
   ```bash
   cd /path/to/MTSUJaguarRobot
   ```

2. **Connect to robot WiFi** (if controlling robot):
   - Use your system's WiFi settings to connect to: `DirJaguar` / `drrobotdrrobot`
   - If using MacOS, run
     ```bash
     python3 network_config.py robot
     ```
     If you want to connect back to wifi on MacOS, run
     ```bash
     python3 network_config.py internet
     ```

3. **Start the app** (one command):
   ```bash
   ./start.sh
   ```

4. **Control the robot**:
   - **Ctrl+click** the provided link: `http://127.0.0.1:5014`
   - Use WASD keys to control

**That's it! No setup required on previously configured machines.**

**Alternative start options:**
```bash
python3 src/app.py         # Direct start without launcher
```

Then open: **http://127.0.0.1:5014**

## 🎮 Controls

| Key | Action |
|-----|--------|
| W | Forward |
| S | Backward |
| A | Turn Left |
| D | Turn Right |
| Space | Stop |
| N | Auto Navigation |

## 🔧 Robot Network Setup

**⚠️ Important: Setup dependencies on internet first, then switch to robot WiFi**

**Complete Robot Setup Workflow:**

1. **Setup on Internet** (download dependencies):
   ```bash
   ./setup/setup.sh  # Must be done on internet connection
   ```

2. **Switch to Robot WiFi**: 
   - Connect to: `DirJaguar` / `drrobotdrrobot`

3. **Configure Robot Network**:
   ```bash
   python3 setup/network_config.py robot
   ```

4. **Start Robot Control**:
   ```bash
   ./start.sh
   ```

5. **Switch Back to Internet** (when done):
   ```bash
   python3 setup/network_config.py internet
   ```

### Manual Network Configuration

If the automated script doesn't work, configure manually:

**macOS/Linux:**
```bash
# Set static IP
sudo ifconfig en0 192.168.0.109 netmask 255.255.255.0
sudo route add default 192.168.0.1

# Set DNS
echo "nameserver 192.168.0.1" | sudo tee /etc/resolv.conf
```

**Windows:**
```cmd
# Open Network settings > Change adapter options
# Right-click WiFi > Properties > IPv4 > Use the following IP:
# IP: 192.168.0.109
# Subnet: 255.255.255.0
# Gateway: 192.168.0.1
# DNS: 192.168.0.1
```

**Required Internet Settings:**
- IP Address: `192.168.0.109`
- Subnet Mask: `255.255.255.0`
- Gateway: `192.168.0.1`
- DNS Server: `192.168.0.1`

**Switch back to internet:** `python3 setup/network_config.py internet`

**Robot Connection Details:**
- Robot Control: `192.168.0.60:10001`
- Camera Feed: `192.168.0.65:8081`
- MQTT Broker: `192.168.1.103:1883`

## 📋 Features

- **Real-time Control** - WASD keyboard controls
- **Live Camera** - Video feed from robot camera
- **Auto Navigation** - Back-and-forth autonomous movement
- **Sensor Fusion** - GPS + IMU data processing
- **Safety Features** - Auto-stop, rate limiting

## 🛠️ Development Mode

**On your regular internet connection**, the app works for:
- Testing web interface
- Developing new features
- UI demonstrations
- Learning the codebase

MQTT timeouts are normal/expected without robot connection.

**To control actual robot**: Follow "Robot Network Setup" above to switch networks.
- In interface click **Connect Robot**
- - Green = Connnected
- - Red = Not Connected
- Click **Camera** to see live camera feed
- Click **PID** and set speed from any range **0-500**

## 📁 Project Structure

```
├── src/app.py              # Main application
├── setup/                  # Setup scripts
├── start.sh               # Quick launcher
└── README.md              # This file
```

## 🔍 Troubleshooting

**Port in use:** `lsof -i :5014` then `kill -9 <PID>`
**Python issues:** Ensure Python 3.10+ installed
**Setup problems:** Run `python3 setup/test_setup.py`

---

**Need help?** Check console output for error details or contact lab supervisors.
