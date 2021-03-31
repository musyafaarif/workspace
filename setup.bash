#!/bin/bash

DIR="$(dirname "$(realpath ${BASH_SOURCE[0]})")"
SETUP_FILE="$DIR/catkin_ws/devel/setup.bash"

source "$SETUP_FILE"
