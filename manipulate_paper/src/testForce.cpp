#include <ee_cart_imped_action/ee_cart_imped_arm.hh>
#include <ee_cart_imped_msgs/EECartImpedGoal.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


ros::Publisher pub;


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud){

  sensor_msgs::PointCloud2 cloud_filtered;

    // Create the filtering object
  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered);
  
  pub.publish(cloud_filtered);
}


void find_table_edge(){
  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe("input",1,cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2>("output",1);
  ros::spin();
}


int main(int argc, char **argv){
  ros::init(argc,argv,"test_force_application");
  find_table_edge();
}



