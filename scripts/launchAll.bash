#!/usr/bin/env bash


#Change directory to workspace. Example: cd ros_ws
echo "Launching with pioneer and kinect"

# build and run ros
#source ~/.bashrc
#catkin build

xterm -title "Roscore" -e roscore &
sleep 5
#Open new terminal and run RosAria
xterm -title "RosAria" -e "rosparam set RosAria/port /dev/pioneer/usb_to_serial_port && rosrun rosaria RosAria" &
sleep 2
#Open new terminal and run keyboard
xterm -title "Keyboard" -e "rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/RosAria/cmd_vel" &
sleep 1
#Open new terminal and run freenect node
xterm -title "Freenect" -hold -e "roslaunch freenect_launch freenect.launch " &
#Open new terminal and run depthimage to laserscan
xterm -title "DepthImage to LaserScan" -e "Rosrun depthimage_to_laserscan depthimage_to_laserscan image:=/camera/depth/image_raw" &
sleep 1
#Open new terminal and run transform file:
xterm -title "TF" -e "roslaunch tf.launch" &
sleep 1
#Run map server
xterm -title "Map_server" -e "rosrun map_server map_server map.yaml" &
sleep 2
#Open rviz
rosrun rviz rviz -d rvizconfig.rviz
#Run python script
xterm -title "MCL" -hold -e "rosrun beginner_tutorials MCL.py" &
