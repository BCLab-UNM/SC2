FROM ros:melodic

# set up the ros-ws
RUN mkdir -p /SC2/src

# Install dependencies
RUN sudo apt-get update && sudo apt-get install -y
RUN sudo apt-get install -y \
	ros-melodic-tf \
	ros-melodic-angles \
	ros-melodic-gazebo-msgs \
	ros-melodic-robot-localization \
	ros-melodic-stereo-image-proc \
	ros-melodic-gazebo-ros \
	ros-melodic-geometry2 \
	python-pip

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
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; source /SC2/srcp2-competitors/ros_workspace/install/setup.bash; cd /SC2; catkin_make"
