# ROS WebSocket Bridge - Refactoring Implementation Plan

This document outlines the planned refactoring of the current `bridge_node.py` implementation to improve architecture, maintainability, and performance.

## Architecture Overview

**Target Architecture:**
- `WebsocketClient`: Handles WebSocket connections and message processing
  - Runs in dedicated asyncio event loop
  - Manages dual connections (control + video channels)
  - Processes incoming messages and routes to ROS bridge
- `RosBridge`: Handles ROS system integration and topic management
  - Manages ROS publishers/subscribers
  - Maintains navigation state machine
  - Provides async interface for WebSocket client
  - Note: ROS callbacks run in separate threads

**Implementation:** Classes in seperate files for readability.

## Heartbeat Implementation
- Use `asyncio.create_task()` with `asyncio.sleep(5)` in continuous loop
- Replace current ROS Timer-based approach
- Send heartbeat messages directly from WebSocket client
- Maintain connection health monitoring

## Inter-Class Communication

**Queue-Based Data Transfer:**
- Use `asyncio.Queue` or `Queue(std)` for async communication between classes
- Separate queues for different data types (images, point clouds, responses)
- Non-blocking queue operations with timeout handling

**Thread Safety:**
- All ROS-related function calls from `WebsocketClient` must use `asyncio.to_thread()`
- Ensures proper async/sync boundary management
- Prevents blocking the asyncio event loop

## Image and Point Cloud Request Processing

**Improved On-Demand Processing:**

### Image Capture Flow:
1. `WebsocketClient` receives image request
2. Calls `await asyncio.to_thread(RosBridge.getImage)` with 1-second timeout
3. `RosBridge.getImage()` creates size-1 `queue(std)` and subscribes to `/cameraF/camera/color/image_raw`
4. `image_callback` receives frame, puts data in queue, cancels subscription
5. `getImage()` returns image data to WebSocket client
6. Response sent via control channel

**Benefits over current implementation:**
- Eliminates persistent subscriptions for one-time requests
- Reduces resource usage when not actively capturing
- Better timeout handling and error recovery

## Video Stream Request Processing

### Start Video Stream:
1. **State Check**: `WebsocketClient` verifies no active stream exists
2. **Queue Creation**: Create size-1 `asyncio.Queue` for frame buffering
3. **ROS Setup**: Call `await asyncio.to_thread(RosBridge.setupVideoStream, putFrame_callback)`
4. **Streaming Task**: Create async task to continuously read from queue and send frames
5. **ROS Subscription**: `setupVideoStream()` subscribes to image topic with callback
6. **Frame Processing**: Callback uses `asyncio.run_coroutine_threadsafe(putFrame(frame))`
7. **Response**: Send acknowledgment to control channel

### Stop Video Stream:
1. **State Check**: Verify active stream exists
2. **Cleanup**: Call `await asyncio.to_thread(RosBridge.stopVideoStream)`
3. **Task Termination**: Cancel streaming task and destroy queue
4. **ROS Cleanup**: Cancel topic subscription
5. **Response**: Send stop confirmation

**Frame Buffering Logic:**
```python
async def putFrame(frame):
    if queue.full():
        queue.get_nowait()  # Drop oldest frame
    await queue.put(frame)  # Add new frame
```

## Navigation Request Processing

**Navigation State Machine:**
```python
class NavigationState(Enum):
    NOT_INIT = "not_init"    # Default state, start position not set
    IDLE = "idle"            # Ready to navigate
    NAVIGATING = "navigating" # Currently navigating to goal
```

### State Transitions:
- `NOT_INIT` → `IDLE`: When start position is set
- `IDLE` → `NAVIGATING`: When navigation goal is sent
- `NAVIGATING` → `IDLE`: When goal is reached or navigation is cancelled

### Start Navigation Flow:
1. **State Check**: Verify robot is in `IDLE` state
2. **Goal Processing**: Call `await asyncio.to_thread(RosBridge.startNavigation, goal_data, complete_callback)`
3. **ROS Publishing**: Publish `PoseStamped` to `/navigation_goal`
4. **Goal Monitoring**: Subscribe to `/goal_reached` with `goal_reached_callback`
5. **State Update**: Set state to `NAVIGATING`
6. **Response**: Send start acknowledgment
7. **Completion**: `goal_reached_callback` triggers `complete_callback` via `run_coroutine_threadsafe`

### Stop Navigation Flow:
1. **State Check**: Verify robot is in `NAVIGATING` state
2. **Cancellation**: Call `await asyncio.to_thread(RosBridge.stopNavigation)`
3. **ROS Publishing**: Publish to `/cancel_nav`
4. **Cleanup**: Cancel goal monitoring subscription
5. **State Update**: Set state to `IDLE`
6. **Response**: Send stop confirmation

### Set Start Position Flow:
1. **State Check**: Allow only in `NOT_INIT` or `IDLE` states
2. **Position Setting**: Call `await asyncio.to_thread(RosBridge.setStartPosition, pose_data)`
3. **ROS Publishing**: Publish `PoseStamped` to `/reset_robot_pos`
4. **State Update**: Set state to `IDLE` if was `NOT_INIT`
5. **Response**: Send position set confirmation

## Robot Status Request Processing

**Current Status**: Returns mock data (matches current implementation)

**Future Enhancement**: Integrate with actual robot sensors
- Battery level from hardware interface
- Component status from diagnostic topics
- Current position from odometry/localization
- Real-time status updates

## Robot Control Request Processing

**Status**: Not implemented in current version

**Planned Implementation**:
- Direct robot movement commands (velocity control)
- Emergency stop functionality
- Manual mode switching
- Integration with robot control topics

## Benefits of Refactored Architecture

### Performance Improvements:
- Reduced resource usage with on-demand subscriptions
- Better frame dropping for video streaming
- Proper async/await patterns throughout

### Maintainability:
- Clear separation of concerns between WebSocket and ROS handling
- State machine for navigation provides clearer logic flow
- Queue-based communication enables better testing and debugging

### Reliability:
- Improved error handling with proper timeout mechanisms
- State validation prevents invalid operation sequences
- Better resource cleanup on shutdown

## Migration Plan

1. **Phase 1**: Implement queue-based communication infrastructure
2. **Phase 2**: Refactor image/pointcloud handling to on-demand model
3. **Phase 3**: Implement navigation state machine
4. **Phase 4**: Migrate video streaming to new architecture
5. **Phase 5**: Testing and validation with fake_robot.py

## Backward Compatibility

- WebSocket message formats remain unchanged
- ROS topic interface stays identical
- External behavior preserved during refactoring
- Gradual migration possible with feature flags