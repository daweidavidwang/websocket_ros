# Mock Video Server for WebSocket Testing

This mock server simulates the video stream reception endpoint to help test WebSocket connection issues without needing the actual server infrastructure. The server automatically sends `request_video_start` commands to connected clients to initiate video streaming.

## Configuration

The WebSocket bridge client supports flexible URI configuration through:

- **Environment Variables**: Set `CONTROL_URI` and `VIDEO_URI` 
- **Command Line Arguments**: Use `--control-uri` and `--video-uri` flags
- **Default Values**: Falls back to `ws://localhost:8443/control` and `ws://localhost:8444/video`

This eliminates the need to manually edit source code when connecting to different servers or mock servers.

## Features

- **Control WebSocket**: Handles control messages and sends mock responses
- **Video WebSocket**: Receives and logs video frame data
- **Automatic Video Start**: Sends `request_video_start` to clients upon connection
- **Additional Test Commands**: Sends robot info, image capture, and video stop requests
- **Frame Statistics**: Tracks frame count, data rates, and connection statistics  
- **SSL/TLS Support**: Supports both secure (wss://) and insecure (ws://) connections
- **Multi-client**: Can handle multiple robot clients simultaneously

## Quick Start

### 1. Start the Mock Server

```bash
# Basic usage (without SSL for local testing)
python src/mock_video_server.py --no-ssl --log-level debug

# With SSL (using self-signed certificate)
python src/mock_video_server.py --log-level debug

# Custom ports
python src/mock_video_server.py --no-ssl --control-port 9443 --video-port 9444

# Disable automatic video start requests
python src/mock_video_server.py --no-ssl --no-auto-start-video
```

The server will start on:
- Control endpoint: `ws://localhost:8443/control` (or `wss://` with SSL)
- Video endpoint: `ws://localhost:8444/video` (or `wss://` with SSL)

### 2. Test with Your Existing Client

You can connect to the mock server using either environment variables or command line arguments:

**Option 1: Using Environment Variables**
```bash
# Set environment variables to point to mock server
export CONTROL_URI="ws://localhost:8443/control"
export VIDEO_URI="ws://localhost:8444/video"
export ROBOT_CODE="TRON1-104454MHz"

# Then run your existing bridge and fake robot:
# Terminal 1: Start the fake robot (ROS simulation)
python src/fake_robot.py --log-level debug

# Terminal 2: Start the bridge node (WebSocket client)
python src/bridge_node.py --log-level debug
```

**Option 2: Using Command Line Arguments**
```bash
# Terminal 1: Start the fake robot (ROS simulation)
python src/fake_robot.py --log-level debug

# Terminal 2: Start the bridge node with URI arguments
python src/bridge_node.py \
  --robot-code TRON1-104454MHz \
  --control-uri ws://localhost:8443/control \
  --video-uri ws://localhost:8444/video \
  --log-level debug
```

**Option 3: Mixed Configuration (Environment + CLI)**
```bash
# Set robot code as environment variable
export ROBOT_CODE="TRON1-104454MHz"

# Use CLI arguments for URIs
python src/bridge_node.py \
  --control-uri ws://localhost:8443/control \
  --video-uri ws://localhost:8444/video \
  --log-level debug
```

> **Note:** You no longer need to manually edit `websocket_client.py` to change the server URLs.

## Server Behavior

The mock server automatically performs the following sequence when a client connects:

1. **Connection Establishment**: Client connects to both control and video WebSocket endpoints
2. **Automatic Video Start**: Server sends `request_video_start` command to the client (after 1 second)
3. **Additional Test Commands** (if auto-start is enabled):
   - After 5 seconds: Sends `request_robot_info` 
   - After 8 seconds: Sends `request_image_capture`
   - After 38 seconds: Sends `request_video_stop`

This automated sequence helps test the complete video streaming workflow without manual intervention.

## Server Output

The mock server provides detailed logging:

```
[INFO] - Mock video server started:
[INFO] -   Control endpoint: ws://localhost:8443/control
[INFO] -   Video endpoint: ws://localhost:8444/video
[INFO] - Waiting for connections...

[INFO] - Control connection established - Client: TRON1-104454MHz, Type: robot
[INFO] - Sent request_video_start to client: TRON1-104454MHz
[INFO] - Video connection established - Client: TRON1-104454MHz, Type: robot

[INFO] - Video stats for TRON1-104454MHz: Frames: 30, Avg size: 15420.5 bytes, Total: 0.44 MB, FPS: 30.1
[DEBUG] - Received video frame from TRON1-104454MHz: Frame #31, Size: 15234 bytes

[INFO] - Sent request_robot_info to client: TRON1-104454MHz
[INFO] - Sent request_image_capture to client: TRON1-104454MHz
[INFO] - Sent request_video_stop to client: TRON1-104454MHz
```

## Command Line Options

### Mock Server Options
```
--host HOST               Server host (default: localhost)
--control-port PORT       Control port (default: 8443)  
--video-port PORT         Video port (default: 8444)
--no-ssl                 Disable SSL (use ws:// instead of wss://)
--cert PATH              Path to SSL certificate file
--key PATH               Path to SSL private key file
--log-level LEVEL        Set logging level (debug, info, warning, error, critical)
--auto-start-video       Automatically send request_video_start to clients (default: enabled)
--no-auto-start-video    Disable automatic video start requests
--standup-sitdown-test   Enable standup/sitdown test commands
```

### Bridge Client Options
```
--robot-code CODE        Robot identification code (or use ROBOT_CODE env var)
--control-uri URI        WebSocket URI for control messages (or use CONTROL_URI env var)
--video-uri URI          WebSocket URI for video streaming (or use VIDEO_URI env var)  
--log-level LEVEL        Set ROS logging level (debug, info, warn, error, fatal)
```

### Environment Variables for Bridge Client
```
ROBOT_CODE              Robot identification code (required)
CONTROL_URI             Control WebSocket URI (default: ws://localhost:8443/control)
VIDEO_URI               Video WebSocket URI (default: ws://localhost:8444/video)
```

## Testing Scenarios

### 1. Basic Connection Testing
Test WebSocket connection establishment, heartbeat functionality, and automatic video start requests.

### 2. Video Stream Testing  
- Monitor automatic video stream initiation
- Track frame reception statistics and data rates
- Test video stream stop functionality

### 3. Control Message Testing
Test various control commands sent automatically by the server:
- `request_video_start` (sent immediately after connection)
- `request_robot_info` (sent after 5 seconds)
- `request_image_capture` (sent after 8 seconds)
- `request_video_stop` (sent after 38 seconds)

### 4. Error Handling Testing
- Disconnect server while streaming to test reconnection
- Send malformed messages to test error handling
- Test client behavior with various server responses

## Troubleshooting

### SSL Certificate Issues
If you see SSL certificate errors, either:
1. Use `--no-ssl` flag for local testing
2. Provide valid certificate files with `--cert` and `--key`
3. Configure your client to ignore SSL certificate validation (for testing only)

### Connection Refused
- Make sure the server is running before starting your client
- Check that ports 8443 and 8444 are not already in use
- Verify firewall settings if connecting from different machines

### No Video Frames Received
- Check that your `fake_robot.py` is publishing video data to the expected ROS topic
- Verify that `bridge_node.py` is successfully reading from ROS and sending binary data
- Enable debug logging (`--log-level debug`) to see detailed frame information
- Ensure your `websocket_client.py` is pointing to the correct video port (8444)

### Client Connection Issues
- Make sure you've configured the correct URIs using either environment variables or CLI arguments
- Check that both control and video connections are established (look for connection logs)
- Verify the robot code matches between client and server logs
- Use `--log-level debug` to see detailed connection information
- Example debug command: `python src/bridge_node.py --robot-code TEST123 --control-uri ws://localhost:8443/control --video-uri ws://localhost:8444/video --log-level debug`

## Complete Test Example

### Method 1: Using Environment Variables
```bash
# Terminal 1: Start mock server
python src/mock_video_server.py --no-ssl --log-level debug

# Terminal 2: Start fake robot (ROS simulation)
python src/fake_robot.py --log-level debug

# Terminal 3: Start WebSocket client with environment variables
export ROBOT_CODE="TRON1-104454MHz"
export CONTROL_URI="ws://localhost:8443/control"
export VIDEO_URI="ws://localhost:8444/video"
python src/bridge_node.py --log-level debug
```

### Method 2: Using Command Line Arguments
```bash
# Terminal 1: Start mock server
python src/mock_video_server.py --no-ssl --log-level debug

# Terminal 2: Start fake robot (ROS simulation)
python src/fake_robot.py --log-level debug

# Terminal 3: Start WebSocket client with CLI arguments
python src/bridge_node.py \
  --robot-code TRON1-104454MHz \
  --control-uri ws://localhost:8443/control \
  --video-uri ws://localhost:8444/video \
  --log-level debug
```

### Method 3: Testing Different Server Addresses
```bash
# Terminal 1: Start mock server on custom ports
python src/mock_video_server.py --no-ssl --control-port 9001 --video-port 9002 --log-level debug

# Terminal 2: Start fake robot (ROS simulation)  
python src/fake_robot.py --log-level debug

# Terminal 3: Connect client to custom server ports
python src/bridge_node.py \
  --robot-code TRON1-104454MHz \
  --control-uri ws://localhost:9001/control \
  --video-uri ws://localhost:9002/video \
  --log-level debug
```

Expected flow:
1. Mock server starts and waits for connections
2. Fake robot starts publishing mock video/sensor data to ROS topics  
3. Bridge node connects to mock server
4. Mock server automatically sends `request_video_start`
5. Bridge node begins streaming video frames to mock server
6. Mock server logs frame statistics and sends additional test commands
