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
    roslaunch ros_websocket_bridge solelaunch.launch
    ```

This will start the ROS-WebSocket connection for testing.
