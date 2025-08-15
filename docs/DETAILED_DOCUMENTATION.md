# Robot Control System - Detailed Documentation

## Architecture Overview

### System Components
- **Flask Web Server** - Main application framework
- **Threading** - Concurrent data processing and monitoring
- **Socket Communication** - Direct TCP connection to robot
- **MQTT Protocol** - Sensor data streaming
- **OpenCV** - Camera stream processing

### Data Flow
1. **Sensor Input** - IMU/GPS data via MQTT
2. **Data Fusion** - GPS/IMU sensor fusion algorithm
3. **Web Interface** - Real-time display and control
4. **Command Processing** - Rate-limited robot commands
5. **Safety Monitoring** - Auto-stop and connection management

## Detailed Configuration

### Robot Settings
```python
# Robot connection settings
robot = RobotSocket('192.168.0.60', 10001)  # Robot IP and port
pid_params.speed = 150                       # Default movement speed

# Rate limiting
command_delay = 0.1    # 100ms minimum between commands
stop_timeout = 0.3     # Auto-stop after 300ms of no input
```

### Camera Settings
```python
camera_url = "http://root:drrobot@192.168.0.65:8081/axis-cgi/mjpg/video.cgi"
```

### MQTT Settings
```python
mqtt_broker_ip = "192.168.1.103"
mqtt_broker_port = 1883

# MQTT Topics:
# - IMU/data - Raw IMU sensor data
# - CAMERA/tracking - Camera tracking data
# - camera/detections - Object detection results
# - camera/frame - Camera frame data
```

### Network Configuration Details

**Robot Network Mode:**
- IP: 192.168.0.109
- Subnet: 255.255.255.0
- Gateway: 192.168.0.1
- Robot: 192.168.0.60:10001
- Camera: 192.168.0.65:8081
- MQTT Broker: 192.168.1.103:1883

**Internet Mode:**
- DHCP configuration
- DNS: 8.8.8.8, 8.8.4.4

## API Reference

### Control Endpoints

#### POST /robot-cmd
Send movement commands to the robot.
```json
{
  "cmd": "w|s|a|d|stop|n"
}
```
Response:
```json
{
  "success": true,
  "throttled": false
}
```

#### POST /connect-robot
Establish TCP connection to robot.
```json
{}
```
Response:
```json
{
  "success": true
}
```

#### POST /robot-autopilot
Send autopilot commands.
```json
{
  "action": "rotateLeft|rotateRight|forward|stop",
  "speed": 150
}
```

#### GET /navigation-status
Get current navigation mode status.
Response:
```json
{
  "active": false,
  "direction": "forward",
  "cycle_time": 3
}
```

### Data Endpoints

#### GET /data
Current fused sensor data.
Response:
```json
{
  "imu": {
    "heading": 45.2,
    "accX": 0.1,
    "accY": -0.05
  },
  "gps": {
    "lat": 36.123456,
    "lon": -86.123456
  },
  "fused": {
    "lat": 36.123456,
    "lon": -86.123456,
    "heading": 45.3
  }
}
```

#### GET /mqtt-data
Raw MQTT message history (last 40 messages).
Response:
```json
[
  {
    "timestamp": "2025-08-15T14:30:00Z",
    "topic": "IMU/data",
    "Heading": 45.2,
    "accX": 0.1,
    "accY": -0.05,
    "gps:Lat": 36.123456,
    "Lon": -86.123456
  }
]
```

### Camera Endpoints

#### GET /camera-stream
Live MJPEG video stream.
Returns: `multipart/x-mixed-replace` MJPEG stream

#### POST /toggle-camera
Start or stop camera streaming.
```json
{
  "action": "start|stop"
}
```
Response:
```json
{
  "success": true,
  "status": "started"
}
```

### Configuration Endpoints

#### POST /pid-update
Update PID control parameters.
```json
{
  "kp": 1.0,
  "ki": 0.0,
  "kd": 0.0,
  "speed": 150
}
```
Response:
```json
{
  "kp": 1.0,
  "ki": 0.0,
  "kd": 0.0,
  "speed": 150
}
```

## Data Fusion Algorithm

### GPS/IMU Sensor Fusion
The system implements a weighted sensor fusion algorithm:

1. **Weighted Heading Calculation**
   ```python
   alpha_imu = 0.4  # IMU weight
   alpha_gps = 0.6  # GPS weight (higher for accuracy)
   
   # Adjust weights based on movement
   if distance < 3.0:
       alpha_imu = 0.6  # Trust IMU more when stationary
       alpha_gps = 0.4
   
   fused_heading = alpha_imu * imu_heading + alpha_gps * gps_heading
   ```

2. **Dynamic Offset Correction**
   - Tracks direction consistency over distance
   - Automatically corrects IMU heading drift
   - Applies corrections gradually based on travel distance

3. **Static Detection**
   - Filters noise when robot is stationary
   - Uses GPS distance threshold (2m) and acceleration threshold (100.0)

4. **Friction Compensation**
   - Detects when robot is stuck during turns
   - Automatically increases motor speed to overcome friction

### Command Protocol

The robot uses MMW (Motor Move Wheel) protocol:

```python
# Enable motors
robot.send_command("MMW !MG")

# Set motor speeds (left, right)
robot.send_command("MMW !M 150 -150")  # Forward
robot.send_command("MMW !M -150 150")  # Backward  
robot.send_command("MMW !M -150 -150") # Turn left
robot.send_command("MMW !M 150 150")   # Turn right
robot.send_command("MMW !M 0 0")       # Stop
```

## Safety Systems

### Rate Limiting
- Minimum 100ms delay between commands
- Prevents command queue buildup
- Maintains system responsiveness

### Auto-Stop System
- Monitors time since last movement command
- Sends stop commands after 300ms timeout
- Multiple stop commands ensure reliability
- Prevents runaway robot behavior

### Connection Management
- Automatic robot reconnection on socket errors
- MQTT retry with exponential backoff (1s to 60s max)
- Socket timeout handling (1 second)

## Development Guidelines

### Adding New Features

#### New Sensor Integration
1. Add MQTT subscription in `on_connect()`:
   ```python
   client.subscribe("NEW_SENSOR/data", 0)
   ```

2. Handle data in `on_message()`:
   ```python
   if msg.topic == "NEW_SENSOR/data":
       # Process sensor data
       current_data['new_sensor'] = processed_data
   ```

#### New Control Commands
1. Extend `/robot-cmd` endpoint:
   ```python
   elif cmd == 'new_command':
       # Handle new command
       robot.send_command("NEW_PROTOCOL_COMMAND")
   ```

2. Add to web interface:
   ```javascript
   case 'KeyX':  // X key
       sendCommand('new_command');
       break;
   ```

#### UI Modifications
- Templates in `src/templates/`
- Static assets in `src/static/`
- Use Leaflet.js for map interactions
- Bootstrap for responsive design

### Testing

#### Unit Testing
```bash
python3 setup/test_setup.py  # Verify setup
```

#### Integration Testing
1. Test without robot (development mode)
2. Test with robot network connection
3. Verify all control commands
4. Check camera feed functionality
5. Test auto-stop and safety features

#### Performance Testing
- Monitor command latency
- Check MQTT message processing rate
- Verify camera stream performance

### Debugging

#### Debug Mode Features
- Detailed console logging
- Flask debug mode with auto-reload
- MQTT connection status monitoring
- Command execution logging

#### Common Debug Steps
1. Check console output for errors
2. Verify network connectivity
3. Test MQTT broker connection
4. Validate robot socket connection
5. Monitor camera stream status

#### Log Analysis
```bash
# Monitor application logs
tail -f console_output.log

# Check network connectivity
ping 192.168.0.60  # Robot
ping 192.168.0.65  # Camera
ping 192.168.1.103 # MQTT broker
```

## Deployment

### Production Considerations
- Use production WSGI server (gunicorn, uwsgi)
- Implement proper logging
- Add authentication/authorization
- Configure firewall rules
- Set up process monitoring

### Environment Variables
```bash
export ROBOT_IP=192.168.0.60
export CAMERA_URL=http://192.168.0.65:8081/...
export MQTT_BROKER=192.168.1.103
export FLASK_ENV=production
```

### Systemd Service Example
```ini
[Unit]
Description=Robot Control System
After=network.target

[Service]
Type=simple
User=robot
WorkingDirectory=/opt/robot-control
ExecStart=/opt/robot-control/.venv/bin/python src/app.py
Restart=always

[Install]
WantedBy=multi-user.target
```

## Hardware Integration

### Required Hardware
- Robot with MMW command protocol support
- IP camera (Axis or compatible)
- IMU sensor with MQTT output
- GPS module with MQTT output
- WiFi network infrastructure

### Robot Communication
- TCP socket connection on port 10001
- MMW command protocol
- Automatic reconnection on failure

### Camera Integration
- HTTP MJPEG stream
- Authentication: root/drrobot
- Real-time video processing with OpenCV

### Sensor Integration
- MQTT-based sensor data
- JSON message format
- Real-time data fusion and processing
