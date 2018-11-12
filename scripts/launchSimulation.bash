#!/usr/bin/env bash


#Change directory to workspace. Example: cd ros_ws
echo "Launching in simulation mode with rosbag"

# build and run ros
#source ~/.bashrc
#catkin build

xterm -title "Roscore" -e roscore &
sleep 5
#Open new terminal and run transform file:
xterm -title "TF" -e "roslaunch tf.launch" &
sleep 1
#Run map server
xterm -title "Map_Server" -e "rosrun map_server map_server map.yaml" &
sleep 1
#Open rviz
rosrun rviz rviz -d rvizconfig.rviz
#Run bagfile
xterm -title "RosBag" -e "rosbag play testbag.bag" &
#Run python script
xterm -title "MCL" -hold -e "rosrun beginner_tutorials MCL.py" &
