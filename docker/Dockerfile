FROM ros:noetic

ENV WORKSPACE=/SC2
ENV DEBIAN_FRONTEND noninteractive

# Install dependencies
RUN apt-get -qq update && \
    apt-get -qq upgrade && \
    apt-get -qq install \
        apt-utils \
        build-essential \
        ros-noetic-tf \
        ros-noetic-angles \
        ros-noetic-gazebo-msgs \
        ros-noetic-robot-localization \
        ros-noetic-stereo-image-proc \
        ros-noetic-gazebo-ros \
        ros-noetic-geometry2 \
        python3-catkin-tools \
        python3-osrf-pycommon

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Clear apt cache to save on space
RUN rm -rf /var/lib/apt/lists/*

# Copy in SC2 project
COPY . $WORKSPACE
RUN chmod 777 $WORKSPACE
RUN rm -rf $WORKSPACE/devel
RUN rm -rf $WORKSPACE/build

# Build project
RUN source /opt/ros/noetic/setup.bash && \
    source $WORKSPACE/srcp2-competitors/ros_workspace/install/setup.bash && \
	cd $WORKSPACE && \
    catkin_make && \
    catkin_make install

# Setup Entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]