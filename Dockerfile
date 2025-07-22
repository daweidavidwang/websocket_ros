FROM ros:noetic

# Update package lists
RUN apt-get update

# Install Python3 and pip
RUN apt-get install -y python3-pip python3-dev

# Install ROS Python dependencies
RUN apt-get install -y \
    python3-rospy \
    python3-std-msgs \
    python3-sensor-msgs \
    python3-geometry-msgs \
    python3-nav-msgs \
    ros-noetic-cv-bridge \
    python3-opencv

# Install Python packages via pip
RUN pip3 install \
    numpy \
    pillow \
    websockets \
    asyncio

# Clean up apt cache to reduce image size
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /catkin_ws

# Source ROS setup
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc