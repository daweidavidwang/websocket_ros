# websocket_ros

## Getting Started

1. **Install Docker**  
    Follow instructions at [docker.com](https://docs.docker.com/get-docker/) to install Docker on your system.
   
2. **Build Docker Image locally**  
    In the project directory, build the Docker image:
    ```sh
    docker build -t ros_websocket .
    ```
    
3. **Launch ROS Noetic Docker Environment**  
    Pull and run the ROS Noetic image:
    ```sh
    docker run -it --rm \
      -v $(pwd):/my_ws/src/websocket_ros \
      ros_websocket
    ```

4. **Test ROS-WebSocket Connection** 
    Compile the ROS workspace and source the setup file:
    ```sh
    cd /my_ws
    catkin_make
    source devel/setup.bash
    ```
    Inside the container, launch the test:
    ```sh
    ROBOT_CODE=robot3234324232 roslaunch ros_websocket_bridge solelaunch.launch
    ```

This will start the ROS-WebSocket connection for testing.

## Configuration

The WebSocket bridge can be configured using environment variables or command line arguments:

### Environment Variables
- `ROBOT_CODE`: The robot identification code (required)
- `CONTROL_URI`: WebSocket URI for control messages (default: `ws://localhost:8443/control`)
- `VIDEO_URI`: WebSocket URI for video streaming (default: `ws://localhost:8444/video`)

### Command Line Arguments
- `--robot-code`: The robot identification code
- `--control-uri`: WebSocket URI for control messages  
- `--video-uri`: WebSocket URI for video streaming
- `--log-level`: Set logging level (debug, info, warn, error, fatal)

### Example Usage
```sh
# Using environment variables
export ROBOT_CODE=robot123
export CONTROL_URI=ws://192.168.1.100:5001/control  
export VIDEO_URI=ws://192.168.1.100:5002/video-stream
roslaunch ros_websocket_bridge solelaunch.launch

# Using command line arguments
ROBOT_CODE=robot123 python3 src/bridge_node.py \
  --control-uri ws://192.168.1.100:5001/control \
  --video-uri ws://192.168.1.100:5002/video-stream \
  --log-level debug
```
