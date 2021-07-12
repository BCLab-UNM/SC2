#
# Space Robotics Challenge 2: NASA JSC
# Final Round
#
# Copyright (c), 2019-2022 NASA-JSC. All Rights Reserved
# Unauthorized Distribution Strictly Prohibited
#
ARG base_image="final_competitor"
FROM ${base_image}
# *******************************************
# ** COMPETITOR CUSTOMIZATION THIS LIST    **
# ** DELIMITED ITEMS WITH SPACE CHARACTER! **

# OS Level Apps (all will be installed root permissions)
# These boxes and figlet, flask are optional are just here as examples
ARG os_apps="apt-utils \
	build-essential \
	ros-noetic-tf \
	ros-noetic-angles \
	ros-noetic-gazebo-msgs \
	ros-noetic-robot-localization \
	ros-noetic-stereo-image-proc \
	ros-noetic-gazebo-ros \
	ros-noetic-geometry2 \
	python3-catkin-tools \
	python3-osrf-pycommon \
	python3 \
	python3-pip"

# Python Plugins
ARG py_apps="ipython future imutils scipy"
# ** COMPETITOR CUSTOMIZATION THIS LIST DONE **
# *********************************************

ARG enduser_name="srcp2"

USER root
# The apts
RUN apt-get update && apt-get install -y; \
    for app in $req_os_apps; do \
    apt-get install $app -y; \
    done;
# The python
RUN for py in $req_py_apps; do \
    pip3 install $py; \
    done;

# Make Home Folder Tree Proper Permissions ** THESE ARE REQUIRED **
# copy in the "binary" & src versions of all the ROS packages
# User Level 
USER ${enduser_name}
RUN mkdir -p -v /home/$enduser_name/cmp_workspace
COPY --chown=${enduser_name}:${enduser_name} ./srcp2-competitors/cmp_workspace/install ${HOME}/cmp_workspace/install/
COPY --chown=${enduser_name}:${enduser_name} . ${HOME}/cmp_workspace/src

# ******************************************
# **    COMPETITOR CUSTOMIZATION HERE!    **
# The Competitor Can Add More Docker Customization Commands Here
# Refer to Docker Documentation for this.
# RUN zzz
# COPY zzz
#
# ******************************************

# Root Level Permissions
USER root
RUN chown -R "${enduser_name}:${enduser_name}" "/home/${enduser_name}";\
    ls -la "/home/${enduser_name}"

# User Level Permissions
USER ${enduser_name}

# Entrypoint and Config Copies ** THESE ARE REQUIRED **
COPY docker/solution/container/solution-entrypoint.bash ${HOME}/scripts
COPY docker/solution/container/config_solution.yaml ${HOME}/config

# starting conditions (note: CMD not ENTRYPOINT for --interactive override)
ENV SOLUTION_ENTRYPOINT="${HOME}/scripts/solution-entrypoint.bash"
CMD ${SOLUTION_ENTRYPOINT}