#include <ros/ros.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/utils.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

#include <manipulate_paper/movements.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ee_cart_imped_action/ee_cart_imped_arm.hh>
#include <ee_cart_imped_msgs/EECartImpedGoal.h>

using namespace std;

arm_navigation_msgs::MoveArmGoal Movements::make_gripper_goal(string group_name, string link_name,double x, double y, double z, double xOR=0.0, double yOR=0.0, double zOR=0.0){
  arm_navigation_msgs::MoveArmGoal goalA;
  goalA.motion_plan_request.group_name = group_name;
  goalA.motion_plan_request.num_planning_attempts=5;
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  goalA.motion_plan_request.planner_id=std::string("");
  goalA.planner_service_name=std::string("ompl_planning/plan_kinematic_path");

  arm_navigation_msgs::SimplePoseConstraint desired_pose;

  desired_pose.header.frame_id = "torso_lift_link";
  desired_pose.link_name = link_name;
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

bool Movements::execute_arm_goal(ros::NodeHandle *nh, actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> *move_arm, arm_navigation_msgs::MoveArmGoal *goalA){
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
          return success;
        }
    } else {
    ROS_INFO("Node handle not ok");
    return false;
  }
}

bool Movements::move_gripper(ros::NodeHandle *nh, bool isLeft, double des_x, double des_y, double des_z, double xOR=0.0, double yOR=0.0, double zOR=0.0){
  string group_name = "";
  string link_name = "";
  string client_name = "";
  if (isLeft)
    {
    group_name="left_arm";
    client_name = "move_left_arm";
    link_name = "l_wrist_roll_link";
    } else {
    group_name="right_arm";
    client_name="move_right_arm";
    link_name = "r_wrist_roll_link";
  }
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm(client_name,true);
  move_arm.waitForServer();
  cerr << "Connected to move arm server";
  arm_navigation_msgs::MoveArmGoal goal = Movements::make_gripper_goal(group_name,link_name,des_x,des_y,des_z,xOR,yOR,zOR);
  return Movements::execute_arm_goal(nh,&move_arm,&goal);
}
bool Movements::move_right_gripper(ros::NodeHandle *nh, double des_x, double des_y, double des_z, double xOR, double yOR, double zOR){
  return Movements::move_gripper(nh, false, des_x, des_y, des_z, xOR,yOR,zOR);
}
bool Movements::move_left_gripper(ros::NodeHandle *nh, double des_x, double des_y, double des_z, double xOR, double yOR, double zOR){
  return Movements::move_gripper(nh, true, des_x, des_y, des_z, xOR,yOR,zOR);  
}

void imped_arm_move(string ref_frame, string arm_ns,double *des_x, double *des_y, double *des_z, double *xOR, double *yOR, double *zOR, double duration)
{
  EECartImpedArm arm(arm_ns);

  ee_cart_imped_msgs::EECartImpedGoal traj;

  EECartImpedArm::addTrajectoryPoint(traj,*des_x,*des_y,*des_z,*xOR,*yOR,*zOR,1,
                                     1000.0,1000.0,1000.0,30.0,30.0,30.0,
                                     false,false,false,false,false,false,
                                     duration,ref_frame);
  arm.startTrajectory(traj);
}

void Movements::imped_left_arm_move(string ref_frame,double des_x, double des_y, double des_z, double xOR, double yOR, double zOR, double duration)
{
  imped_arm_move(ref_frame,"l_arm_cart_imped_controller",&des_x,&des_y,&des_z,&xOR,&yOR,&zOR,duration);
}

void Movements::imped_right_arm_move(string ref_frame,double des_x, double des_y, double des_z, double xOR, double yOR, double zOR, double duration)
{
  imped_arm_move(ref_frame,"r_arm_cart_imped_controller",&des_x,&des_y,&des_z,&xOR,&yOR,&zOR,duration);
}

void Movements::imped_reset_left_arm()
{
  Movements::imped_left_arm_move("/torso_lift_link",0,0.75,0.0);
}
void Movements::imped_reset_right_arm()
{
  Movements::imped_right_arm_move("/torso_lift_link",0,-0.75,0.0);
}

