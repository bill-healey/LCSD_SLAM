#!/bin/bash
killall roscore
killall Mono
sleep 2
screen -dmS core bash -c 'roscore'
sleep 2
screen -dmS orb bash -c 'cd ORB_SLAM2; rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/TUMmonoVO_yaml/monoVO_ORB_SLAM_full_1.yaml'
sleep 2
#screen -dmS dso bash -c 'cd DSO_ROS/catkin_ws; . devel/setup.bash; roslaunch dso_ros monoVO_seong_01.launch'
cd DSO_ROS/catkin_ws;
. devel/setup.bash;
roslaunch dso_ros monoVO_seong_01.launch
