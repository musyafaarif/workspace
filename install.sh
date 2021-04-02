#!/bin/bash

source setup/ubuntu.sh
source setup/ros_gazebo.sh
source setup/sim.sh

DIR="$(dirname "$(realpath ${BASH_SOURCE[0]})")"
SETUP_FILE="$DIR/catkin_ws/devel/setup.bash"

if [[ ! -f "$SETUP_FILE" ]]; then
    cd catkin_ws
    catkin build
    if [[ ! -f "$SETUP_FILE" ]]; then
        echo "Build failed! Fix the issue and run this script again!"
        return
    fi
fi

WORKSPACE_SETUP_FILE="source $(dirname $(realpath ${BASH_SOURCE[0]}))/setup.bash"
if grep -Fxq "$WORKSPACE_SETUP_FILE" ~/.bashrc; then echo Workspace setup.bash already in .bashrc;
else echo "$WORKSPACE_SETUP_FILE" >> ~/.bashrc; fi;
eval $WORKSPACE_SETUP_FILE
