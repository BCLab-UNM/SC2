FROM ros:noetic

# set up the ros-ws
RUN mkdir -p /SC2/src

# Install dependencies
RUN sudo apt update && apt upgrade -y && sudo apt install -y
RUN sudo apt install -y \
	ros-noetic-tf \
	ros-noetic-angles \
	ros-noetic-gazebo-msgs \
	ros-noetic-robot-localization \
	ros-noetic-stereo-image-proc \
	ros-noetic-gazebo-ros \
	ros-noetic-geometry2 

RUN pip install imutils
RUN pip install scipy==1.2.0

# Clear apt cache to save on space
RUN rm -rf /var/lib/apt/lists/*

# Copy in SC2 project
COPY . /SC2
RUN chmod 777 /SC2
RUN rm -rf /SC2/devel
RUN rm -rf /SC2/build

# Build project
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; source /SC2/srcp2-competitors/ros_workspace/install/setup.bash; cd /SC2; catkin_make"
