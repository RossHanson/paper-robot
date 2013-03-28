#! /bin/bash

std_sleep_interval=60s

echo "Starting Gazebo..."
roslaunch manipulation_worlds pr2_table_object.launch > gazeboOut.out & #Start Gazebo with the table
sleep ${std_sleep_interval}

echo "Looking down..."
rosrun manipulate_paper shake &
sleep ${std_sleep_interval}

echo "Starting arm tuck..."
rosrun pr2_tuckarm tuck_arms.py -l t -r u &
sleep ${std_sleep_interval}
#sleep ${std_sleep_interval}

echo "Taking picture..."
rosrun image_view image_view image:=/wide_stereo/left/image_rect_color
sleep 15s

echo "Starting lasers"
roslaunch pr2_tabletop_manipulation_launch pr2_tabletop_manipulation.launch > laser.out  #Start laser stuff

wait