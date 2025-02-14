FROM base_image_ros

# Set the working directory
WORKDIR /app

# Create ROS2 workspace
RUN mkdir -p /app/ros2_ws/src/opencv_tools

# Copy source code to the container
COPY . /app/ros2_ws/src/opencv_tools
RUN pip3 install -r /app/ros2_ws/src/opencv_tools/requirements.txt

# Build the ROS2 workspace and source the environment
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /app/ros2_ws && colcon build"

# Source the built workspace before running the command
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 launch opencv_tools camera.launch.py"]
