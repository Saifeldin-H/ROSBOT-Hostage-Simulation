FROM husarion/rosbot-gazebo:humble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-humble-action-msgs \
    ros-humble-nav2-msgs \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY src/docker/simulation/prefetch_fuel_models.sh /usr/local/bin/prefetch_fuel_models
RUN bash /usr/local/bin/prefetch_fuel_models

WORKDIR /tmp/hostage_ws
COPY src/ros2_ws/src/hostage_sim src/hostage_sim
RUN . /opt/ros/humble/setup.sh && \
    colcon build --merge-install --packages-select hostage_sim

ENV IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/tmp/hostage_ws/install/lib
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/tmp/hostage_ws/install/lib
