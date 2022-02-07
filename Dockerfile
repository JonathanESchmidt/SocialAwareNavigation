FROM osrf/ros:melodic-desktop

RUN apt-get -y update && apt-get install -y \
    iputils-ping \
    net-tools \
    wget \
    curl \
    nano \
    ros-melodic-joy \
    ros-melodic-teleop-twist-joy \
    ros-melodic-teleop-twist-keyboard \ 
    ros-melodic-laser-proc \
    ros-melodic-rgbd-launch \
    ros-melodic-depthimage-to-laserscan \
    ros-melodic-rosserial-arduino \ 
    ros-melodic-rosserial-python \
    ros-melodic-rosserial-server \ 
    ros-melodic-rosserial-client \ 
    ros-melodic-rosserial-msgs \
    ros-melodic-amcl \
    ros-melodic-map-server \
    ros-melodic-move-base \
    ros-melodic-urdf \
    ros-melodic-xacro \
    ros-melodic-compressed-image-transport \
    ros-melodic-rqt-image-view \
    ros-melodic-gmapping \
    ros-melodic-navigation \
    ros-melodic-interactive-markers

RUN curl -sSL http://get.gazebosim.org | sh

RUN apt-get -y update && apt-get install -y\
    ros-melodic-joint-state-publisher-gui \
    ros-melodic-gazebo11-ros \
    ros-melodic-gazebo11-msgs \
    ros-melodic-gazebo11-plugins \
    ros-melodic-gazebo11-ros-pkgs \
    ros-melodic-gazebo11-ros-control \
    ros-melodic-gazebo11-dev \
    ros-melodic-ros-controllers \
    ros-melodic-moveit \
    ros-melodic-trac-ik-kinematics-plugin \
    ros-melodic-tf2-sensor-msgs \
    ros-melodic-costmap-converter \
    ros-melodic-mbf-costmap-core \
    ros-melodic-mbf-msgs \
    ros-melodic-libg2o \
    libsuitesparse-dev \
    ros-melodic-people-tracking-filter \
    ros-melodic-face-detector \
    ros-melodic-leg-detector \
    ros-melodic-slam-karto \
    ros-melodic-people-velocity-tracker \
    libnetpbm10-dev

RUN mkdir -p /root/robot/src
WORKDIR /root/robot
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash"
RUN echo "source /opt/ros/melodic/setup.sh" >> /root/.bashrc
RUN echo "source /root/robot/devel/setup.bash" >> /root/.bashrc
WORKDIR /root/robot
