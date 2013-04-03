#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_pose_goal_test");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm_right("move_right_arm",true);
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm_left("move_left_arm",true);
  move_arm_right.waitForServer();
  move_arm_left.waitForServer();
  ROS_INFO("Connected to server");
  arm_navigation_msgs::MoveArmGoal goalRight;
  arm_navigation_msgs::MoveArmGoal goalLeft;

  goalRight.motion_plan_request.group_name = "right_arm";
  goalRight.motion_plan_request.num_planning_attempts = 1;
  goalRight.motion_plan_request.planner_id = std::string("");
  goalRight.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalRight.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  goalLeft.motion_plan_request.group_name = "left_arm";
  goalLeft.motion_plan_request.num_planning_attempts = 1;
  goalLeft.motion_plan_request.planner_id = std::string("");
  goalLeft.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalLeft.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  
  arm_navigation_msgs::SimplePoseConstraint desired_pose;
  arm_navigation_msgs::SimplePoseConstraint desired_post_left;
  desired_pose.header.frame_id = "torso_lift_link";
  desired_pose.link_name = "r_wrist_roll_link";
  desired_pose.pose.position.x = 0;
  desired_pose.pose.position.y = -0.75;
  desired_pose.pose.position.z = 0;

  desired_pose.pose.orientation.x = 0.0;
  desired_pose.pose.orientation.y = 0.0;
  desired_pose.pose.orientation.z = 0.0;
  desired_pose.pose.orientation.w = 1.0;

  desired_pose.absolute_position_tolerance.x = 0.02;
  desired_pose.absolute_position_tolerance.y = 0.02;
  desired_pose.absolute_position_tolerance.z = 0.02;

  desired_pose.absolute_roll_tolerance = 0.04;
  desired_pose.absolute_pitch_tolerance = 0.04;
  desired_pose.absolute_yaw_tolerance = 0.04;
  
  arm_navigation_msgs::SimplePoseConstraint desired_post_left_left;
  desired_post_left.header.frame_id = "torso_lift_link";
  desired_post_left.link_name = "l_wrist_roll_link";
  desired_post_left.pose.position.x = 0;
  desired_post_left.pose.position.y = 0.75;
  desired_post_left.pose.position.z = 0;

  desired_post_left.pose.orientation.x = 0.0;
  desired_post_left.pose.orientation.y = 0.0;
  desired_post_left.pose.orientation.z = 0.0;
  desired_post_left.pose.orientation.w = 1.0;

  desired_post_left.absolute_position_tolerance.x = 0.02;
  desired_post_left.absolute_position_tolerance.y = 0.02;
  desired_post_left.absolute_position_tolerance.z = 0.02;

  desired_post_left.absolute_roll_tolerance = 0.04;
  desired_post_left.absolute_pitch_tolerance = 0.04;
  desired_post_left.absolute_yaw_tolerance = 0.04;
  
  

  if (nh.ok())
  {
    bool finished_within_time = false;
    arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalRight);
    arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_post_left,goalLeft);
   move_arm_right.sendGoal(goalRight);
   move_arm_left.sendGoal(goalLeft);
    finished_within_time = move_arm_left.waitForResult(ros::Duration(200.0));
    finished_within_time = move_arm_right.waitForResult(ros::Duration(200.0));    
    if (!finished_within_time)
    {
      move_arm_left.cancelGoal();
      move_arm_right.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state_right = move_arm_right.getState();
      actionlib::SimpleClientGoalState state_left = move_arm_left.getState();
      bool success = (state_right == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state_right.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state_right.toString().c_str());
      success = (state_left == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state_left.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state_left.toString().c_str());
    }
  }

  
  ros::shutdown();
}
