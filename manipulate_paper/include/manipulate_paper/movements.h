#ifndef MOVEMENTS_H
#define MOVEMENTS_H

#include <arm_navigation_msgs/MoveArmAction.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>


using namespace std;

namespace Movements{

arm_navigation_msgs::MoveArmGoal make_gripper_goal(string group_name, string link_name,double x, double y, double z, double xOR, double yOR, double zOR);

void execute_arm_goal(ros::NodeHandle *nh, actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> *move_arm, arm_navigation_msgs::MoveArmGoal *goalA);

void move_gripper(ros::NodeHandle *nh, bool isLeft, double des_x, double des_y, double des_z, double xOR, double yOR, double zOR);

void move_right_gripper(ros::NodeHandle *nh, double des_x, double des_y, double des_z, double xOR=0.0, double yOR=0.0, double zOR=0.0);

void move_left_gripper(ros::NodeHandle *nh, double des_x, double des_y, double des_z, double xOR=0.0, double yOR=0.0, double zOR=0.0);
};

#endif
