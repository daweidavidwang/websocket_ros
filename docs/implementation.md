# Architectrue

- class WebsocketClient: handle websocket message from and to cloud.
 - running in an asyncio loop
- class RosBridge: handle communication bwtween WebsocketClient and ROS system.
 - Note: topic subscribe callback are running in other thread

class should be in a single file.

 ## Heartbeat
Use asyncio.sleep() to do heartbeat every 5s.

# communication between WebsocketClient and RosBridge
- transfer data via queue
- calling of functions that relative to ros from WebsocketClient should wrap into asyncio.to_thread()

## handle image and pointcloud request
Take handle image as example:
- `WebsocketClient` call RosBridge.getImage() in asyncio.to_thread() and await for data from queue, timeout 1s. If success, response the image data.
- `RosBridge.getImage` create a size 1 queue, subscribe the image topic. Then wait for data from queue and return.
- `image_subscribe_callback` get a frame of image and put it into queue, cancel the subscribe.

## handle video stream request
### When start video stream is requested
- `WebsocketClient` create a streamming task with a size 1 asyncio.queue, call RosBridge.setupVideoStream in asyncio.to_thread() with async putFrame(frame) function and response. If video is streamming, response error.
- `async putFrame` put the frame into queue. If queue is full, pop the first and then put in.
- `streamming task` await for data from queue and then send the frame as binary jpeg blob.
- `RosBridge.setupVideoStream` subscribe the image topic with stream callback. hold the subscribe object for later cancel.
- `stream callback` put frame using async putFrame in asyncio.run_coroutine_threadsafe.

### when stop video stream is requested
- `WebsocketClient` call RosBridge.stopVideoStream in asyncio.to_thread(), stop streamming task, destroy queue then response.  If video is not streamming, response error.
- `RosBridge.stopVideoStream` cancel topic subscription.

## handle navigation request
RosBridge should maintain the status of navigation
 - not_init: default status. start_position is not set. when set, goto idle
 - idle: robot ready to navigate
 - navigating: robot is navigating to the target pose. back to idle when complete

### when start navigation is requested
 - `WebsocketClient` If robot status is idle, call RosBridge.startNavigation() in asyncio.to_thread() with complete_callback. If not, response error.
 - `RosBridge.startNavigation` publish to /navigation_goal. subscribe /goal_reached with goal_reached_callback
 - `complete_callback` response navigation complete
 - `goal_reached_callback` call complete_callback in asyncio.run_coroutine_threadsafe. cancel subscription of /goal_reached

### when stop navigation is requsted
 - `WebsocketClient` If robot status is navigating, call RosBridge.stopNavigation() in asyncio.to_thread(). If not, response error.
 - `RosBridge.stopNavigation` publish to /cancel_nav. 
### when set start positon is requested
 - `WebsocketClient` If robot status is not_init or idle, call RosBridge.setStartPosition() in asyncio.to_thread(). If not, response error.
 - `RosBridge.setStartPosition` publish to /reset_robot_pos.

## handle robot status request
TODO. response mock data for now.

## handle robot control request
TODO. Not implement.