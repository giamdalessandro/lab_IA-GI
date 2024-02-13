#!/bin/bash
source "/home/labaigi/workspaces/labaigi_ws/devel/setup.bash"
xterm -e roscore &
sleep 3
xterm -e rosrun my_counter counter_server_node &
sleep 3
xterm -e rosrun my_counter counter_client_node &

