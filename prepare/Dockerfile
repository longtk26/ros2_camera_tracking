FROM ros:humble-ros-base
# FROM dustynv/ros:humble-ros-base-l4t-r32.7.1
# Install necessary packages including wget
RUN apt update && apt install -y \
    software-properties-common \
    wget

# Install Python 3.10 and pip3
# RUN add-apt-repository ppa:deadsnakes/ppa \
#     && apt update \
#     && apt install -y python3.10 python3.10-distutils \
#     && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1 \
#     && wget https://bootstrap.pypa.io/get-pip.py \
#     && python3 get-pip.py
RUN wget https://bootstrap.pypa.io/get-pip.py \
    && python3 get-pip.py

# Install colcon build tool and its extensions
RUN apt install -y python3-colcon-common-extensions

# Install PyTorch and torchvision, using CPU versions
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# Install OpenCV and NumPy
RUN pip3 install opencv-python==4.5.4.58
RUN pip3 install opencv-contrib-python==4.5.4.58
RUN pip3 install "numpy<2"

# Install dependencies for cv_bridge
RUN apt install -y \
ros-humble-cv-bridge \
ros-humble-image-transport \
ros-humble-vision-opencv \
ros-humble-rosbridge-server