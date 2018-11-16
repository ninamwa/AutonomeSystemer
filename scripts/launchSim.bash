#!/usr/bin/env bash


#Change directory to workspace. Example: cd ros_ws
echo "Launching in simulation mode with rosbag"

# build and run ros
#source ~/.bashrc
#catkin build

xterm -title "Roscore" -e roscore &
sleep 5
rosparam set use_sim_time true
#Open new terminal and run transform file:
#xterm -title "TF" -e "roslaunch tf.launch" &
#sleep 1
#Run map server in new terminal
xterm -title "Map_Server" -e "rosrun map_server map_server map.yaml" &
sleep 1
#Open rviz in new terminal
xterm -title "Rosrun" -e "rosrun rviz rviz -d rvizconfig.rviz" &
sleep 1
#Run bagfile in new terminal
xterm -title "RosBag" -e "rosbag play testbag.bag --clock --pause" &
#Run python script
rosrun beginner_tutorials MCL.py
