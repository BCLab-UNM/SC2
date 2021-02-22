#!/bin/bash

SLEEP_INTERVAL=5
arg=""
while [ $# -gt 0 ]
do
    case $1 in
        -b)     rm "$(dirname $0)/devel/setup.bash" 2>/dev/null
                ;;
        -h | --help) echo "usage: ./run.sh [-b clean/build code] [--help]"
                exit 1
                ;;
        *)      echo "invalid argument"
                echo "usage: ./run.sh [-b clean/build code] [--help]"
                exit 2
                ;;
    esac
shift
done # end of while 

# Trap signals then stop all docker containers
trap "{ docker kill $(docker ps -q) ; echo q >> /tmp/srcp2RunSocket; exit 0 ; }" SIGINT SIGTERM EXIT
source /opt/ros/noetic/setup.bash || { echo "Is ROS noetic installed?" && exit 10 ; }

_CATKIN_SETUP_DIR="$(dirname $0)/srcp2-competitors/ros_workspace/install/"
source "$(dirname $0)/srcp2-competitors/ros_workspace/install/setup.bash" 

if [ -f "$(dirname $0)/devel/setup.bash" ]; then
    source "$(dirname $0)/devel/setup.bash"
else
    echo not built, building now
    catkin clean -y
    catkin build && source "$(dirname $0)/devel/setup.bash" || exit 5
    # or 
    #echo "ERROR: You must compile the code first before you run this script."
    #echo "ERROR: Run the following command: catkin build"
    #exit -1
fi


# run the round in the background
# -n is for no Gazebo GUI
# Create FIFO Pipe to send 'q' charter to and make their script not freak out
rm /tmp/srcp2RunSocket 2>/dev/null # kill the old one if it is there
mkfifo /tmp/srcp2RunSocket # Create FIFO Pipe
cat >/tmp/srcp2RunSocket & # this will leave it open
$(dirname $0)/srcp2-competitors/docker/scripts/run_srcp2_final $arg </tmp/srcp2RunSocket >/dev/null &

echo "Waiting for master to startup"
watch --chgexit rostopic list 2>&1 >/dev/null
echo "Sleeping for 10 secs"
sleep 10 # wait abit more, might look for a particluar topic to show up insted
echo "Unpausing Gazebo Physics"
rosservice call /gazebo/unpause_physics
echo "Launching our launch file"
roslaunch "$(dirname $0)"/launch/final_round.launch &
echo "Launched"
sleep $SLEEP_INTERVAL

# display score
while rostopic list /rosout; do
    clear
    score=`rostopic echo -n 1 /qual_"$round"_score/score | head -n1 | tr -d '"'`  # update the score
    rosnow=`rostopic echo -n 1 /clock/clock/secs | head -n 1` # get the current ros time # lookinto the --filter 
    echo "$rosnow : $score" # time_in_secs : score
    sleep $SLEEP_INTERVAL
done
echo "Hit the bottom"


