#!/bin/bash

docker run -it \
    --network srcp2net \
    -v /SC2:$(pwd) \
    swarmathon:development \
    /bin/sh -c 'roslaunch scoot scoot.launch "mode:=man" &
                sleep 2 &&
                ROS_NAMESPACE="/small_scout_1" rosrun scoot repl.py'
