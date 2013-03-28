#include <ros/ros.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/utils.h>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
using namespace boost;
using namespace std;


arm_navigation_msgs::MoveArmGoal moveGripper(double x, double y, double z, double xOR, double yOR, double zOR){
  arm_navigation_msgs::MoveArmGoal goalA;
  goalA.motion_plan_request.group_name = "left_arm";
  goalA.motion_plan_request.num_planning_attempts=1;
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  goalA.motion_plan_request.planner_id=std::string("");
  goalA.planner_service_name=std::string("ompl_planning/plan_kinematic_path");

  arm_navigation_msgs::SimplePoseConstraint desired_pose;

  desired_pose.header.frame_id = "torso_lift_link";
  desired_pose.link_name = "l_wrist_roll_link";
  desired_pose.pose.position.x = x;
  desired_pose.pose.position.y = y;
  desired_pose.pose.position.z = z;

  desired_pose.pose.orientation.x = xOR;
  desired_pose.pose.orientation.y = yOR;
  desired_pose.pose.orientation.z = zOR;
  desired_pose.pose.orientation.w = 1.0;

  desired_pose.absolute_position_tolerance.x = 0.02;
  desired_pose.absolute_position_tolerance.y = 0.02;
  desired_pose.absolute_position_tolerance.z = 0.02;

  desired_pose.absolute_roll_tolerance = 0.04;
  desired_pose.absolute_pitch_tolerance = 0.04;
  desired_pose.absolute_yaw_tolerance = 0.04;
  
  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

  return goalA;
}

void execute_arm_action(ros::NodeHandle *nh, actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> *move_arm, arm_navigation_msgs::MoveArmGoal *goalA){
  if ((*nh).ok())
    {
      bool finished_within_time = true;
      (*move_arm).sendGoal(*goalA);
      finished_within_time = (*move_arm).waitForResult(ros::Duration(200.0));
      if (!finished_within_time)
        {
          (*move_arm).cancelGoal();
          ROS_INFO("Timed out achieving goalA");
        }
      else
        {
          actionlib::SimpleClientGoalState state = (*move_arm).getState();
          bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
          if (success)
            ROS_INFO("Action finished: %s",state.toString().c_str());
          else
            ROS_INFO("Action failed: %s", state.toString().c_str());
        }
    }
}

int main1(int argc, char** argv){
  ros::init (argc, argv, "move_arm_pose_goal_test");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
  move_arm.waitForServer();
  ROS_INFO("Connected to server");
  arm_navigation_msgs::MoveArmGoal goalA;

  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  
  arm_navigation_msgs::SimplePoseConstraint desired_pose;
  desired_pose.header.frame_id = "torso_lift_link";
  desired_pose.link_name = "r_wrist_roll_link";
  desired_pose.pose.position.x = 0.75;
  desired_pose.pose.position.y = -0.188;
  desired_pose.pose.position.z = 0;

  desired_pose.pose.orientation.x = 0.0;
  desired_pose.pose.orientation.y = 0.0;
  desired_pose.pose.orientation.z = 0.0;
  desired_pose.pose.orientation.w = 1.0;

  desired_pose.absolute_position_tolerance.x = 0.005;
  desired_pose.absolute_position_tolerance.y = 0.005;
  desired_pose.absolute_position_tolerance.z = 0.005;

  desired_pose.absolute_roll_tolerance = 0.04;
  desired_pose.absolute_pitch_tolerance = 0.04;
  desired_pose.absolute_yaw_tolerance = 0.04;
  
  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

  if (nh.ok())
    {
      bool finished_within_time = false;
      move_arm.sendGoal(goalA);
      finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
      if (!finished_within_time)
        {
          move_arm.cancelGoal();
          ROS_INFO("Timed out achieving goal A");
        }
      else
        {
          actionlib::SimpleClientGoalState state = move_arm.getState();
          bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
          if(success)
            ROS_INFO("Action finished: %s",state.toString().c_str());
          else
            ROS_INFO("Action failed: %s",state.toString().c_str());
        }
    }
  ros::shutdown();
}

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_joint_goal_test");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_left_arm",true);

  move_arm.waitForServer();
  ROS_INFO("Connected to server");
  
  ifstream inputFile("data/in.txt");
  string line;
  char_separator<char> sep(",");
  
  double p [6] = {0};
  while (getline(inputFile,line))
    {       
      tokenizer< char_separator<char> > tokens(line, sep);
      int j = 0;
      BOOST_FOREACH (const string& t, tokens){ //Borrowed heavily from StackExchange question 53849
        p[j++] =atof(t.c_str());
      }
      arm_navigation_msgs::MoveArmGoal goalA = moveGripper(p[0],p[1],p[2],p[3],p[4],p[5]);
      execute_arm_action(&nh, &move_arm, &goalA);
    }

  ros::shutdown();
}
