#! /bin/bash

std_sleep_interval=60s

roslaunch manipulation_worlds pr2_table_object.launch > gazeboOut.txt & #Start Gazebo with the table
sleep ${std_sleep_interval}

roslaunch pr2_tabletop_manipulation_launch pr2_tabletop_manipulation.launch > laserOut.txt & #Start laser stuff
sleep ${std_sleep_interval}

rosrun pr2_tuckarm tuck_arms.py -l t -r u &
sleep ${std_sleep_interval}
sleep ${std_sleep_interval}

rosrun manipulate_paper shake
