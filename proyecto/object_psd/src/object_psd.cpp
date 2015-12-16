#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL specific includes
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilteredPtr (new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCL data type
  pcl::fromROSMsg(*cloud_msg, *cloudPtr);

  // Perform voxel grid filtering
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (leaf_size, leaf_size, leaf_size);
  sor.filter (*cloudFilteredPtr);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = static_cast<int>(cloudFilteredPtr->points.size());
  while (cloudPlane->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloudFilteredPtr);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      ROS_WARN("Could not estimate a planar model for the given dataset.");
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloudFilteredPtr);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloudPlane);
    ROS_INFO_STREAM_THROTTLE(1, "PointCloud representing the planar component: " << cloudPlane->points.size () << " data points.");

    // Remove the planar inliers, extract the rest
/*    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;*/
  }
  std::cout.flush();
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloudPlane, output);

  // Publish the data
  output.header.stamp = ros::Time::now();
  output.header.frame_id = cloud_msg->header.frame_id;
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
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  // Spin
  ros::spin ();
  return 0;
}