#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/ModelCoefficients.h>
ros::Publisher pub;
ros::Subscriber sub;

using namespace std;

bool cmp(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2){
return point1.x < point2.x;
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{

sensor_msgs::PointCloud2 cloud_filtered_blob, out_blob, tmp_out_blob;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> best_out;

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new plc::PointCloud<pcl::PointXYZ>), cloud_p (new plc::PointCloud<pcl::PointXYZ>), cloud_f (new plc::PointCloud<pcl::PointXYZ>);
  // Perform the actual filtering

pcl::fromROSMsg(*cloud,*cloud_filtered);

pcl::PCDWriter writer;

pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

pcl::SACSegmentation<pcl::PointXYZ> seg;

seg.setOptimizeCoefficients (true);

seg.setModelType(pcl::SACMODEL_PLANE);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setMaxIterations(1000);
seg.setDistanceThreshold(0.01);

pcl::ExtractIndices<pcl::PointXYZ> extract;
float best_val = -100;

int i = 0, nr_points = (int) cloud_filtered->points.size();
while (cloud_filtered->points.size() > .04 * nr_points && i<10)
  {
seg.setInputCloud(cloud_filtered);
seg.segment(*inliers, *coefficients);
if (inliers->indices.size() == 0)
  {std::cerr << "Could not estimate a planar model for the given data set" << std::endl;
break;
}

extract.setInputCloud(cloud_filtered);
extract.setIndices(inliers);
extract.setNegative(false);
extract.filter(*cloud_p);
std:cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << " | Size: "
<< cloud_p->width * cloud_p->height << "data points" << std::endl;
std::stringstream ss;
ss << "table_scene_lms_" << i << ".pcd";
writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
if (coefficients->values[3]>best_val){
best_val = coefficients->values[3];
best_out = *cloud_p;
}
extract.setNegative(true);
extract.filter(*cloud_f);
cloud_filtered.swap(cloud_f);
i++;
}


std::cerr << "Best model width: " << best_out.width << " and height " << best_out.height;
  pcl::toROSMsg(best_out,tmp_out_blob);

pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
sensor_msgs::PointCloud2::ConstPtr tmp_out_blob_ptr(new sensor_msgs::PointCloud2(tmp_out_blob));
sor.setInputCloud (tmp_out_blob_ptr);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (out_blob);
std::cerr << " Downsampled model width: " << out_blob.width << " and height " << out_blob.height << std::endl;
pub.publish(out_blob);

pcl::fromROSMsg(out_blob,best_out);
std::sort(best_out.begin(),best_out.end(),cmp);
std::cerr << "Lowest x position: " << best_out.points[0].x << std::endl;


ROS_INFO("FINISHED RUNNING");



sub.shutdown();
}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
sub  = nh.subscribe ("input", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
