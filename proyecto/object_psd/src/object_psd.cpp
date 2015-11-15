#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
// VoxelGrid Config
#include <dynamic_reconfigure/server.h>
#include <object_psd/VoxelGridConfig.h>

ros::Publisher pub;
double leaf_size;

void config_cb (object_psd::VoxelGridConfig &config, uint32_t level) {
  leaf_size = config.leaf_size;
  ROS_INFO("Reconfigure Leaf Size: %f", leaf_size);
}

void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2::Ptr cloudPtr (new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr cloudFilteredPtr (new pcl::PCLPointCloud2);

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloudPtr);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (leaf_size, leaf_size, leaf_size);
  sor.filter (*cloudFilteredPtr);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(*cloudFilteredPtr, output);

  // Publish the data
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "object_psd");
  ros::NodeHandle nh;

  // Initialize dynamic config server
  dynamic_reconfigure::Server<object_psd::VoxelGridConfig> server;
  dynamic_reconfigure::Server<object_psd::VoxelGridConfig>::CallbackType cb;
  cb = boost::bind(&config_cb, _1, _2);
  server.setCallback(cb);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/bender/head_sensor/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  // Spin
  ros::spin ();
  return 0;
}