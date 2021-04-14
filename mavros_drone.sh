#!/bin/bash

roslaunch mavros px4.launch node_name:="drone_mavros" fcu_url:="udp://:14540@127.0.0.1:14557"
