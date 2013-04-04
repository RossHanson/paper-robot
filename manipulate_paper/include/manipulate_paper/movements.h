#ifndef MOVEMENTS_H
#define MOVEMENTS_H

#include <arm_navigation_msgs/MoveArmAction.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>


using namespace std;

namespace Movements{

arm_navigation_msgs::MoveArmGoal make_gripper_goal(string group_name, string link_name,double x, double y, double z, double xOR, double yOR, double zOR);

bool execute_arm_goal(ros::NodeHandle *nh, actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> *move_arm, arm_navigation_msgs::MoveArmGoal *goalA);

bool move_gripper(ros::NodeHandle *nh, bool isLeft, double des_x, double des_y, double des_z, double xOR, double yOR, double zOR);

bool move_right_gripper(ros::NodeHandle *nh, double des_x, double des_y, double des_z, double xOR=0.0, double yOR=0.0, double zOR=0.0);

bool move_left_gripper(ros::NodeHandle *nh, double des_x, double des_y, double des_z, double xOR=0.0, double yOR=0.0, double zOR=0.0);

void imped_left_arm_move(string ref_frame,double des_x, double des_y, double des_z, double xOR=0.0, double yOR=0.0, double zOR=0.0, double duration=5);

 void imped_right_arm_move(string ref_frame,double des_x, double des_y, double des_z, double xOR=0.0, double yOR=0.0, double zOR=0.0, double duration=5);

 void imped_reset_left_arm();
 void imped_reset_right_arm();

};

#endif
