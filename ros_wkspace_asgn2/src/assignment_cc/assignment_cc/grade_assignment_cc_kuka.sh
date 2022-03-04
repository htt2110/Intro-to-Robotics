#!/bin/bash  

echo "Executing assignment in Xterm windows"

roslaunch assignment_cc grade_asgn_cc.launch &

sleep 5 
xterm -hold -e "rosrun assignment_cc AutoGrade.py"

sleep 2 
#Kill all ros nodes
killall -9 rosmaster


