# Use ROS 2 Foxy Docker image as the base
FROM ros:foxy
# Copy requirements file
COPY requirement.txt .
RUN echo "Cache invalidation" > /dev/null
RUN cat requirement.txt
#Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*
# # Uninstall existing NumPy
# RUN pip3 uninstall -y numpy



# Install packages from requirements file
RUN pip install -r requirement.txt

# Install additional Python packages
RUN pip3 install --no-cache-dir \
    torch \
    numpy \
    opencv-python
# Install cv_bridge
RUN apt-get update && apt-get install -y \
    ros-foxy-cv-bridge \
    && rm -rf /var/lib/apt/lists/*
# Install termcolor
RUN pip install --no-cache-dir \
    termcolor \
    torchvision \
    torch-scatter

    


# Set up environment variables for ROS2 Foxy
ENV ROS_DISTRO=foxy
ENV ROS_VERSION=2

# Source ROS2 setup script
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

COPY . /workspace/ros2_ws

#Set the working directory where we run the commands in shell
WORKDIR /workspace/ros2_ws

# Set up entry point
CMD ["bash"]
