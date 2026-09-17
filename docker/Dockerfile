# Base Image: Official ROS 2 Jazzy base
FROM ros:jazzy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WORKSPACE=/workspace

# Set working directory inside container
WORKDIR ${WORKSPACE}

# Install essential build tools, desktop/RViz packages, dependencies, and ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-tk \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # Desktop & RViz2
    ros-jazzy-desktop \
    ros-jazzy-rviz2 \
    # Navigation2 Packages
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-smac-planner \
    ros-jazzy-nav2-mppi-controller \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-nav2-bt-navigator \
    # SLAM & Localization
    ros-jazzy-robot-localization \
    ros-jazzy-rtabmap-ros \
    # Robot Description & Simulation (Gazebo / Ignition)
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    # Perception & OpenCV
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-vision-msgs \
    python3-opencv \
    python3-numpy \
    python3-scipy \
    python3-sklearn \
    # System Utility Tools
    mesa-utils \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt-get/lists/*

# Install Patchwork++ for fast 3D ground plane segmentation
RUN pip3 install --no-cache-dir --break-system-packages pypatchworkpp

# Initialize rosdep if not already initialized
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && rosdep update

# Automatically source ROS 2 setup and aliases in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc && \
    echo "if [ -f ~/.bash_aliases ]; then . ~/.bash_aliases; fi" >> ~/.bashrc

CMD ["bash"]
