#!/bin/bash
source "/home/labaigi/workspaces/labaigi_ws/devel/setup.bash"
cd "/home/labaigi/workspaces/labaigi_ws/src/rp_action/scripts/"
./start_demo.sh &
sleep 3
xterm -e rosrun barbablu barbablu_node

