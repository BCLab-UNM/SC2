#!/bin/bash

###### UNTESTED ######

if [ -f "$(dirname $0)/devel/setup.bash" ]; then
    source "$(dirname $0)/devel/setup.bash"
else
    # echo not built, building now
    # source "$(dirname $0)/srcp2-competitors/ros_workspace/install/setup.sh"
    # catkin build
    # source "$(dirname $0)/devel/setup.bash"
    # or 
    echo "ERROR: You must compile the code first before you run this script."
    echo "ERROR: Run the following command: catkin build"
    exit -1
fi
round=1

while [ $# -gt 0 ]
do
    case $1 in
        -r)     shift
                round="$1"
                ;;
        -b)     shift
                behavior="$1"
                ;;
        --help) echo "usage: ... "
                exit 1
                ;;
        *)      echo "invalid argument"
                echo "usage: ./run.sh [-r round] [-b behavior] [--help]"
                exit 2
                ;;
    esac
shift
done # end of while

# run the round in the background
# @TODO: might redirect the output so it dose not interupt
$(dirname $0)/srcp2-competitors/docker/scripts/launch/roslaunch_docker -c -n -e --run-round $round &

# wait for master to startup
watch --chgexit rostopic list 2>&1 >/dev/null
sleep 10 # wait abit more, might look for a particluar topic to show up insted

#$round
# @TODO: Need to add round args to the various launch files and make sure to read them in task
roslaunch ./launch/scoot.launch "name:=scout_1" "search:=searchRandomWalk"

# stop all docker containers
docker kill $(docker ps -q)
