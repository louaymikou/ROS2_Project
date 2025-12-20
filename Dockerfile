# =============================================================================
# Dockerfile for ROS2 Humble Mobile Manipulator
# Includes: Gazebo, Nav2, SLAM Toolbox, ros2_control, Joystick support
# =============================================================================

FROM osrf/ros:humble-desktop-full

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# =============================================================================
# Install ROS2 dependencies and tools
# =============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Navigation and SLAM
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    # ros2_control
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    # Gazebo packages
    ros-humble-gazebo-ros-pkgs \
    # Teleop and joystick
    ros-humble-teleop-twist-keyboard \
    ros-humble-joy \
    ros-humble-joy-linux \
    # URDF and Xacro
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    # Python tools
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    # Development tools
    git \
    vim \
    nano \
    htop \
    # Joystick tools
    joystick \
    jstest-gtk \
    # X11 for GUI
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    transforms3d \
    numpy

# =============================================================================
# Create workspace and copy source
# =============================================================================
WORKDIR /ros2_ws

# Copy source files
COPY src/ src/

# =============================================================================
# Build the workspace
# =============================================================================
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# =============================================================================
# Setup entrypoint
# =============================================================================
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
