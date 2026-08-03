#! /bin/bash

# echo ${ROS_DISTRO}
# echo ${ROS_MASTER}
echo "Setup ROS_MASTER_URI"


# echo ${CATKIN_WS}

echo "> Setting up ROS"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# if ! [[ -z "$CATKIN_WS" ]]; then
#     echo "> Setting up catkin workspace"
#     source "${CATKIN_WS}/devel/setup.bash"
# fi

if [[ $ROS_MASTER_URI ]]; then
    echo "ROS_MASTER_URI = ${ROS_MASTER_URI}"
    export ROS_MASTER_URI=${ROS_MASTER_URI}
fi
if [[ $ROS_IP ]]; then
    echo "ROS_IP = ${ROS_IP}"
    export ROS_IP=${ROS_IP}
fi

if [ "${ROS_MASTER:-true}" = true ]; then
    echo "> Setting up ROScore"
    /bin/bash -c "roscore || exit 0"   # run roscore in a new shell
    echo "> Started ROScore in a new process"
else
    if [[ -z "$ROS_MASTER_URI" ]]; then
        echo -e "\033[0;32mROS_MASTER_URL is not set\033[0m"
    fi
fi

# Run CMD from Dockerfile or the overriding command from docker compose yaml file
echo "> Running $@"
exec "$@"
