#!/bin/bash
source "/home/labaigi/workspaces/labaigi_ws/devel/setup.bash"

#xterm -e roscore &
#sleep 3

xterm -e roslaunch lucrezio_simulation_environments empty_world_with_apartment_and_robot.launch environment:=test_apartment_2 &
sleep 3

cd /home/labaigi/datasets/test_apartment_2
xterm -e rosrun map_server map_server map.yaml &
sleep 3

xterm -e rosrun lucrezio_simulation_environments pose_broadcaster_node &
sleep 3

cd /home/labaigi/datasets/test_apartment_2
xterm -e rosrun lucrezio_semantic_mapper semantic_mapper_node _environment:="test_apartment_2" &
sleep 3

xterm -e rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=lucrezio/cmd_vel &

cd /home/labaigi/datasets/
rosrun rviz rviz -d view.rviz
