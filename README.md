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
