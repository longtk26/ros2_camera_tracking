FROM ros:humble-ros-base

# Set the working directory
WORKDIR /app

# Create ROS2 workspace
RUN mkdir -p /app/ros2_ws/src/opencv_tools

# Copy source code to the container
COPY . /app/ros2_ws/src/opencv_tools

# Install necessary packages including wget
RUN apt update && apt install -y \
    software-properties-common \
    wget

# Install Python 3.10 and pip3
RUN add-apt-repository ppa:deadsnakes/ppa \
    && apt update \
    && apt install -y python3.10 python3.10-distutils \
    && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1 \
    && wget https://bootstrap.pypa.io/get-pip.py \
    && python3 get-pip.py

# Install rviz2 for ROS2
RUN apt update && apt install -y ros-humble-rviz2

# Install colcon build tool and its extensions
RUN apt install -y python3-colcon-common-extensions

# Install PyTorch and torchvision, using CPU versions
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# Install OpenCV and NumPy
RUN pip3 install opencv-python==4.5.4.58
RUN pip3 install opencv-contrib-python==4.5.4.58
RUN pip3 install "numpy<2"
RUN pip3 install -r /app/ros2_ws/src/opencv_tools/requirements.txt


# Install dependencies for cv_bridge
RUN apt update && apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-opencv

# Build the ROS2 workspace and source the environment
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /app/ros2_ws && colcon build"

# Source the built workspace before running the command
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 launch opencv_tools camera.launch.py"]
