# Robot Control System

Web-based robot control application for MTSU Jaguar Robot with real-time navigation, camera feed, and sensor data fusion.

## 🚀 Quick Setup (30 seconds)

```bash
cd /path/to/MTSUJaguarRobot
./setup/setup.sh start     # One command - sets up everything and starts app
```

**Alternative options:**
```bash
./start.sh                 # If already set up
python3 src/app.py         # Manual start
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

**Only needed when controlling actual robot:**

1. Connect to WiFi: `DirJaguar` / `drrobotdrrobot`
2. Configure network: `python3 setup/network_config.py robot`
3. Start app: `./start.sh`

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

**Required Settings:**
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

Without robot hardware, the app still works for:
- Testing web interface
- Developing new features
- UI demonstrations
- Learning the codebase

MQTT timeouts are normal/expected without robot connection.

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
