#!/bin/bash

if [[ -z $1 || $1 = S* || $1 = s* ]]
then
  name="small_scout_1"
elif [[ $1 = E* || $1 = e* ]]
then
  name="small_excavator_1"
elif [[ $1 = H* || $1 = h* ]]
then
  name="small_hauler_1"
else
  echo "Unable to start command with unmapped name $1"
  exit 1
fi

echo "Starting docker REPL with $name"

docker run -it \
    --network srcp2net \
    -v /SC2:$(pwd) \
    -v ~/.ipython/profile_default:/root/.ipython/profile_default \
    swarmathon:development \
    /bin/sh -c "roslaunch scoot scoot.launch \"mode:=man\" \"name:=$name\" &
                sleep 2 &&
                ROS_NAMESPACE=\"/$name\" rosrun scoot repl.py"
