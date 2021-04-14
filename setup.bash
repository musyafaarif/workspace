#!/bin/bash

DIR="$(dirname "$(realpath ${BASH_SOURCE[0]})")"
SETUP_FILE="$DIR/catkin_ws/devel/setup.bash"

source "$SETUP_FILE"

# Set the plugin path so Gazebo finds our model and sim
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:"$DIR/simulator/build"
# Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:"$DIR/simulator/models":"$DIR/simulator_extras/models"
# Disable online model lookup since this is quite experimental and unstable
# export GAZEBO_MODEL_DATABASE_URI=""
# Add path to simulator_extras for getting world path
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:"$DIR/simulator_extras"
# Set path to sitl_gazebo repository
export SITL_GAZEBO_PATH="$DIR/simulator":"$DIR/simulator_extras"

# Kenjeran
export PX4_HOME_LAT=-7.2297
export PX4_HOME_LON=112.8298
export PX4_HOME_ALT=1.5         # Boat height: 1.5m

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$DIR/firmware
