FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-action-msgs \
    ros-humble-geometry-msgs \
    ros-humble-launch-ros \
    ros-humble-nav-msgs \
    ros-humble-nav2-msgs \
    ros-humble-rclpy \
    ros-humble-ros2launch \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

CMD ["bash"]
