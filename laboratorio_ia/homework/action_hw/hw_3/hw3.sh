#!/bin/bash
source "/home/labaigi/workspaces/labaigi_ws/devel/setup.bash"
xterm -e roscore &
sleep 3
xterm -e rosrun android_interface android_interface_node.py _port:=9001 &
sleep 3
rosrun aiml_ros aiml_ros_node.py _aiml_path:=/home/labaigi/Desktop/laboratorio_ia/homework/action_hw/hw_2

