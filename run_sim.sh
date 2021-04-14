#!/bin/bash

world_name="$1"
if [[ "$world_name" == "" ]]; then
    # TODO
    echo "Usage:"

    echo "Set world to default \"ocean\""
    world_name="ocean"
fi

world_name="$world_name.world"
# gazebo --verbose "./simulator_extras/worlds/$world_name"
roslaunch gazebo_ros empty_world.launch verbose:=true world_name:="worlds/$world_name"