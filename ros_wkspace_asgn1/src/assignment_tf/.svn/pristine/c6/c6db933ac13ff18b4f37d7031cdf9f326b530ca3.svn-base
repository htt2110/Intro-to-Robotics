#!/bin/bash  

echo "Executing assignment in Xterm windows"

roslaunch assignment_tf grade_asgn_tf.launch &

sleep 2 
xterm -hold -e "rosrun assignment_tf AutoGrade.py"

sleep 2 
#Kill all ros nodes
killall -9 rosmaster


