# ROS WebSocket Bridge Specification

## Overview
This program is a WebSocket client that bridges a cloud platform and the ROS system in the robot. It provides bidirectional communication between the cloud platform and robot hardware through WebSocket connections and ROS topics.

## System Architecture

### Components
- **WebSocket Bridge Node** (`bridge_node.py`): Main bridge handling WebSocket connections and ROS communication
- **Fake Robot Node** (`fake_robot.py`): Testing node that simulates robot sensors and responses
- **Package**: `ros_websocket_bridge` v0.0.1

### WebSocket Connections
The bridge maintains two WebSocket connections to `wss://gj-test.mnt-aihub.com:8443/`:

1. **Control Channel** (`/control`): Command and control messages
2. **Video Channel** (`/video`): Real-time video streaming

Both connections use headers:
- `client_type`: "robot"
- `client_id`: `ROBOT_CODE` ("robot3234324232")

## Functionality

### Core Features
- **Camera Image Capture**: Responds with camera image when requested
- **Point Cloud Data**: Responds with 3D point cloud data when requested  
- **Video Streaming**: Provides real-time camera video stream
- **Navigation Control**: Receives navigation commands from cloud
  - Go to position
  - Stop navigation
  - Set initial position
- **Robot Status**: Reports robot information and health status
- **Heartbeat**: Maintains connection with periodic heartbeat messages
- **Posture Control**: Stand up and sit down sequences via robot command service

## ROS Topics

### Subscribed Topics
- `/cameraF/camera/color/image_raw` (sensor_msgs/Image): Camera images for streaming and capture
- `/cloud_registered` (sensor_msgs/PointCloud2): Point cloud data from 3D sensors
- `/goal_reached` (std_msgs/Bool): Navigation goal completion status

### Published Topics
- `/navigation_goal` (geometry_msgs/PoseStamped): Navigation target position
- `/cancel_nav` (std_msgs/Bool): Cancel current navigation
- `/reset_robot_pos` (geometry_msgs/PoseStamped): Reset robot initial position

### ROS Services
- `robot_command` (highlevel_websocket_control/CommandService): High-level robot posture & motion commands (e.g., stand, walk, sit)

## WebSocket Message Protocol

### Message Structure
All messages use JSON format with common fields:
- `title`: Message type identifier
- `timestamp`: Unix timestamp in milliseconds
- `guid`: Unique message identifier
- `targetType`: "robot" or "client"
- `targetId`: Target identifier
- `clientId`: Client identifier (robot code)
- `taskHistoryCode`: Task tracking identifier
- `data`: Message payload

### Request Messages (Cloud → Robot)

#### Image Capture Request
```json
{
  "title": "request_image_capture",
  "timestamp": 1234567890000,
  "guid": "uuid-1234-req-img",
  "taskHistoryCode": "task_12345"
}
```

#### Point Cloud Capture Request
```json
{
  "title": "request_pointcloud_capture", 
  "timestamp": 1234567890000,
  "guid": "uuid-5678-req-pc",
  "taskHistoryCode": "task_12346"
}
```

#### Video Control Requests
```json
{
  "title": "request_video_start",
  "timestamp": 1234567890000,
  "taskHistoryCode": "task_12347"
}
```

```json
{
  "title": "request_video_stop",
  "timestamp": 1234567890000,
  "taskHistoryCode": "task_12348"
}
```

#### Navigation Control
```json
{
  "title": "request_start_navigation",
  "timestamp": 1234567890000,
  "taskHistoryCode": "task_12349",
  "requestType": "start",
  "data": {
    "position": {"x": 1.0, "y": 2.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  }
}
```

```json
{
  "title": "request_stop_navigation",
  "timestamp": 1234567890000,
  "taskHistoryCode": "task_12350"
}
```

#### Robot Info Request
```json
{
  "title": "request_robot_info",
  "timestamp": 1234567890000,
  "taskHistoryCode": "task_12351"
}
```

#### Set Initial Position
```json
{
  "title": "request_start_point",
  "timestamp": 1234567890000,
  "data": {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  }
}
```

#### Stand Up Request
```json
{
  "title": "request_standup_start",
  "timestamp": 1680000002000,
  "guid": "uuid-standup-req",
  "targetType": "robot",
  "targetId": "robot-code",
  "clientId": "client-id",
  "data": {
    "robotCode": "TRON1A-104454Mhz"
  }
}
```

#### Sit Down Request
```json
{
  "title": "request_sitdown_start",
  "timestamp": 1680000002000,
  "guid": "uuid-sitdown-req",
  "targetType": "robot",
  "targetId": "robot-code",
  "clientId": "client-id",
  "data": {
    "robotCode": "TRON1A-104454Mhz"
  }
}
```

### Response Messages (Robot → Cloud)

#### Image Capture Response
```json
{
  "title": "response_image_capture",
  "timestamp": 1234567890000,
  "guid": "uuid-1234-req-img",
  "targetType": "client",
  "targetId": "client_id",
  "clientId": "robot3234324232",
  "taskHistoryCode": "task_12345",
  "data": {
    "robotCode": "robot3234324232",
    "result": "ok",
    "height": 1080,
    "width": 1920,
    "encoding": "bgr8",
    "is_bigendian": false,
    "step": 5760,
    "data": "hex_encoded_image_data"
  }
}
```

#### Point Cloud Response
```json
{
  "title": "response_pointcloud_capture",
  "timestamp": 1234567890000,
  "guid": "uuid-5678-req-pc",
  "targetType": "robot",
  "targetId": "client_id",
  "clientId": "robot3234324232",
  "taskHistoryCode": "task_12346",
  "data": {
    "robotCode": "robot3234324232",
    "result": "ok",
    "height": 1,
    "width": 16200,
    "fields": {"x_size": 4, "y_size": 4, "z_size": 4, "rgb_size": 4},
    "is_bigendian": false,
    "point_step": 16,
    "row_step": 259200,
    "data": "hex_encoded_pointcloud_data",
    "is_dense": true
  }
}
```

#### Navigation Response
```json
{
  "title": "response_start_navigation",
  "timestamp": 1234567890000,
  "guid": "navigation_guid",
  "targetType": "client", 
  "targetId": "client_id",
  "clientId": "robot3234324232",
  "taskHistoryCode": "task_12349",
  "requestType": "start",
  "data": {
    "robotCode": "robot3234324232",
    "result": "ok"
  }
}
```

#### Goal Reached Notification
```json
{
  "title": "response_arrive_navigation",
  "timestamp": 1234567890000,
  "guid": "navigation_guid",
  "targetType": "client",
  "targetId": "client_id", 
  "clientId": "robot3234324232",
  "taskHistoryCode": "task_12349",
  "requestType": "start",
  "data": {
    "robotCode": "robot3234324232",
    "result": "ok"
  }
}
```

#### Robot Info Response
```json
{
  "title": "response_robot_info",
  "timestamp": 1234567890000,
  "guid": "info_guid",
  "targetType": "client",
  "targetId": "client_id",
  "clientId": "robot3234324232", 
  "taskHistoryCode": "task_12351",
  "data": {
    "robotCode": "robot3234324232",
    "accid": "PF_TRON1A_042",
    "sw_version": "robot-tron1-2.0.10.20241111103012",
    "imu": "OK",
    "camera": "OK", 
    "motor": "OK",
    "battery": 95,
    "status": "WALK",
    "gridPosition": {"x": 100, "y": 200, "z": 0},
    "result": "ok"
  }
}
```

#### Heartbeat Message
```json
{
  "title": "response_heartbeat",
  "timestamp": 1234567890000,
  "guid": "uuid-3456-req-vp",
  "targetType": "robot",
  "targetId": "heartbeat",
  "clientId": "heartbeat",
  "data": {
    "robotCode": "robot3234324232",
    "result": "ok"
  }
}
```

#### Stand Up Response
```json
{
  "title": "response_standup_start",
  "timestamp": 1680000002500,
  "guid": "uuid-standup-req",
  "targetType": "client",
  "targetId": "client-id",
  "clientId": "robot-code",
  "data": {
    "robotCode": "TRON1A-104454Mhz",
    "result": "ok"
  }
}
```

#### Sit Down Response
```json
{
  "title": "response_sitdown_start",
  "timestamp": 1680000002500,
  "guid": "uuid-sitdown-req",
  "targetType": "client",
  "targetId": "client-id",
  "clientId": "robot-code",
  "data": {
    "robotCode": "TRON1A-104454Mhz",
    "result": "ok"
  }
}
```

### Video Streaming
Video frames are sent as a binary JPEG blob to the video WebSocket.

## Implementation Details

### Dependencies
- Python 3
- ROS (rospy, std_msgs, sensor_msgs, geometry_msgs, nav_msgs)
- WebSockets (`websockets` library)
- OpenCV (`cv2`)
- PIL (Python Imaging Library)
- NumPy

### Configuration
- **Robot Code**: "robot3234324232" (hardcoded in `bridge_node.py:20`)
- **WebSocket URL**: "wss://gj-test.mnt-aihub.com:8443/"
- **Heartbeat Interval**: 5 seconds
- **Video Quality**: JPEG compression at 50% quality
- **Point Cloud Format**: XYZRGB with 16-byte point step
- **Posture Control Sequence**: Sit → Stand → (wait status=STAND) → Walk (wait status=WALK); Walk → Sit(wait status=SIT)

### Error Handling
- Automatic WebSocket reconnection with 5-second retry interval
- Exception handling for image processing and message parsing
- Graceful shutdown on ROS interruption
- Connection status logging

### Testing
The `fake_robot.py` node provides:
- 160x90 test images at 30Hz with animated patterns
- Realistic indoor point clouds at 10Hz (5500+ points)
- Simulated navigation goal responses (20-second delay)
- Navigation cancellation support
- (Planned) Simulated posture status transitions for stand/sit validation

## Posture Control Logic
The posture control feature leverages the `robot_command` service:

Stand Up Flow:
1. Send `stand` command via `robot_command`.
2. Poll / subscribe to robot status (implementation-specific) until status string == `STAND`.
3. Send `walk` command.
4. Wait until status string == `WALK`.
5. Emit `response_standup_start` with result.

Sit Down Flow:
1. Send `sit` command.
2. Wait until status string == `SIT`.
3. Emit `response_sitdown_start` with result.

Errors (timeouts, service failures) should respond with `result": "error"` and an error description field (future extension) while keeping the message schema stable.

## Launch Configuration

### Standard Launch
```bash
roslaunch ros_websocket_bridge bridge.launch
```

### With Fake Robot (Testing)
```bash
roslaunch ros_websocket_bridge solelaunch.launch
```
