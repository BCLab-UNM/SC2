#!/bin/bash

docker run -it \
    --network srcp2net \
    -v /SC2:$(pwd) \
    -v ~/.ipython/profile_default:/root/.ipython/profile_default \
    swarmathon:development \
    /bin/sh -c 'roslaunch scoot scoot.launch "mode:=man" &
                sleep 2 &&
                ROS_NAMESPACE="/small_scout_1" rosrun scoot repl.py'
