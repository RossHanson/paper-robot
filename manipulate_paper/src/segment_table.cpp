#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
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
#include <pcl_ros/transforms.h>
//Rviz marker
#include <visualization_msgs/Marker.h>
//Transform
#include <tf/transform_listener.h>
//Math
#include <cmath>
//My stuff
#include <manipulate_paper/movements.h>

ros::Publisher pub;
ros::Publisher marker_pub;
ros::Publisher move_pub;
ros::Subscriber sub;
ros::NodeHandle* nh;
std::string target_id = "/torso_lift_link";

using namespace std;

bool cmp(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2){
return point1.x < point2.x;
}

void add_rviz_marker(double x, double left_y, double right_y, double z, string frame_id){
  ros::Rate r(1);


  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker, line;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = frame_id; line.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now(); line.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes"; line.ns="basic_shapes";
  marker.id = 0; line.id = 1;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;  line.type = visualization_msgs::Marker::LINE_STRIP;


  line.scale.x = .05;
  
    // Set the marker action.  Options are ADD and DELETE
    marker.action = line.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point point1;
    geometry_msgs::Point point2;
    point1.z = point2.z = z;
    point1.x = point2.x = x;
    point1.y = left_y; point2.y = right_y;
    line.points.push_back(point1);
    line.points.push_back(point2);
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = z;
    line.pose.orientation.x = marker.pose.orientation.x = 0.0;
    line.pose.orientation.y = marker.pose.orientation.y = 0.0;
    line.pose.orientation.z = marker.pose.orientation.z = 0.0;
    line.pose.orientation.w = marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = .05;
    marker.scale.y = .05;
    marker.scale.z = .05;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f; line.color.r = 1.0f;
    marker.color.g = 1.0f; line.color.g = 1.0f;
    marker.color.b = 0.0f; line.color.b = 0.0f;
    marker.color.a = 1.0; line.color.a = 1.0;

    marker.lifetime = ros::Duration(); line.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker); marker_pub.publish(line);
  }


void perform_arm_motion(float table_edge_x, float table_edge_z){
  float x_offset = 0.065;
  float z_offset = 0.05;
    cerr << "Shooting for: " << table_edge_x << " and " << table_edge_z <<endl;
    
    cerr << "Offset: " << x_offset;
    stringstream args;
    args <<"move_to_table_edge " << table_edge_x << " " << table_edge_z;
    std_msgs::String str;
    str.data = args.str();
    //Movements::imped_left_arm_move(target_id,table_edge_x - x_offset,0.0, table_edge_z - z_offset,0.1,0.0,-1.0);
    x_offset = x_offset + .01;
  
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    cerr << "Frame id " <<cloud->header.frame_id << endl;
sensor_msgs::PointCloud2 cloud_filtered_blob, out_blob, tmp_out_blob;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> best_out;

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new plc::PointCloud<pcl::PointXYZ>), cloud_p (new plc::PointCloud<pcl::PointXYZ>), cloud_f (new plc::PointCloud<pcl::PointXYZ>);
  // Perform the actual filtering


tf::TransformListener listener;

 ros::Time now = ros::Time::now();
 listener.waitForTransform(target_id,cloud->header.frame_id,now,ros::Duration(3.0));
  tf::StampedTransform transform;
  listener.lookupTransform(target_id,cloud->header.frame_id,ros::Time::now()-ros::Duration(1),transform);
 pcl_ros::transformPointCloud(target_id, transform, *cloud,tmp_out_blob);
pcl::fromROSMsg(tmp_out_blob,*cloud_filtered); 

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
float best_val = 100;

int i = 0, nr_points = (int) cloud_filtered->points.size();
while (cloud_filtered->points.size() > .01 * nr_points && i<10)
  {
    std::cout << "Looking at cloud " << std::endl;
seg.setInputCloud(cloud_filtered);
seg.segment(*inliers, *coefficients);
if (inliers->indices.size() == 0)
  {std::cerr << "Could not estimate a planar model for the given data set" << std::endl;
break;
}
 std::cout << "About to filter stuff out" << std::endl;
extract.setInputCloud(cloud_filtered);
extract.setIndices(inliers);
extract.setNegative(false);
extract.filter(*cloud_p);
 std::cout << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << " | Size: "
<< cloud_p->width * cloud_p->height << "data points" << std::endl;
 if (coefficients->values[2]>0.9&&coefficients->values[3]<best_val && cloud_p->width>10000){
   best_val = coefficients->values[3];
   std::cout << "Updated best " << std::endl;
   best_out = *cloud_p;
}
 std::cout << "Made it past loop" << std::endl;
 pcl::toROSMsg(*cloud_p,tmp_out_blob); 
pub.publish(tmp_out_blob);
ros::Duration(3.0).sleep();
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
double threshold = best_out.points[0].x + .005;
double left_bound=0.0;
double right_bound=0.0;
double z_ave = 0.0;
pcl::PointCloud<pcl::PointXYZ>::VectorType::iterator point_it = best_out.begin();
double count = 0;
while((point_it!=best_out.end())&&point_it->x<threshold){
    right_bound = std::min(right_bound,double(point_it->y));
    left_bound = std::max(left_bound,double(point_it->y));
    z_ave += double(point_it->z);
    point_it++;
    count++;
}
z_ave = z_ave / count;

add_rviz_marker(best_out.points[0].x,left_bound,right_bound,z_ave,target_id);
ROS_INFO("FINISHED RUNNING");
 perform_arm_motion(best_out.points[0].x,best_out.points[0].z);
sub.shutdown();
}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  nh = new ros::NodeHandle;
  Movements::imped_reset_left_arm();
  Movements::imped_reset_right_arm();
  // Create a ROS subscriber for the input point cloud
sub  = nh->subscribe ("input", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub = nh->advertise<sensor_msgs::PointCloud2> ("output", 1);
  marker_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);
  move_pub = nh ->advertise<std_msgs::String>("force_control_commands",1);
  // Spin
  ros::spin ();
}
