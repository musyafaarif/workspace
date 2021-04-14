#!/bin/bash

roslaunch mavros px4.launch node_name:="boat_mavros" fcu_url:="udp://:14541@127.0.0.1:14557" tgt_system:=2
