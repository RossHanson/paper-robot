#include <ee_cart_imped_action/ee_cart_imped_arm.hh>
#include <ee_cart_imped_msgs/EECartImpedGoal.h>

int main(int argc, char **argv){
  ros::init(argc,argv,"test_force_application");
  
  EECartImpedArm arm("r_arm_cart_imped_controller");

  ee_cart_imped_msgs::EECartImpedGoal traj;

  EECartImpedArm::addTrajectoryPoint(traj, 0.5, 0, 0, 0, 0, 0, 1,
                                     1000, 1000, 1000, 30, 30, 30,
                                     false, false, false, false, false,
                                     false, 4, "/torso_lift_link");

  EECartImpedArm::addTrajectoryPoint(traj, 0.75, 0, 0, 0, 0, 0, 1,
                                     50, 1000, 1000, 30, 30, 30,
                                     false, false, false, false, false,
                                     false, 6, "/torso_lift_link");

  arm.startTrajectory(traj);
}
