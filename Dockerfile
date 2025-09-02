FROM arm64v8/ros:jazzy

ENV SHELL=/bin/bash

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

# Install realsense2-camera

RUN apt update && \
    apt install -y ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-realsense2-camera-dbgsym \
                ros-$ROS_DISTRO-realsense2-camera-msgs ros-$ROS_DISTRO-realsense2-camera-msgs-dbgsym \
    ros-$ROS_DISTRO-realsense2-description ros-$ROS_DISTRO-image-transport


# Install rplidar_ros2

WORKDIR /rplidar_ws/src

RUN git clone https://github.com/Slamtec/sllidar_ros2.git && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd .. && \
    colcon build --merge-install

RUN rm -rf /var/lib/apt/lists/*

COPY ./entrypoint.sh /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
