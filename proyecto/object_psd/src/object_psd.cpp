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
// VoxelGrid configuration
#include <dynamic_reconfigure/server.h>
#include <object_psd/VoxelGridConfig.h>

ros::Publisher pub;
double leaf_size;

void config_cb(object_psd::VoxelGridConfig &config, uint32_t level)
{
  leaf_size = config.leaf_size;
  ROS_INFO("Reconfigure Leaf Size: %f", leaf_size);
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilteredPtr(new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCL data type
  pcl::fromROSMsg(*cloud_msg, *cloudPtr);

  // Perform voxel grid filtering
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudFilteredPtr);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudObj(new pcl::PointCloud<pcl::PointXYZ>());
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02);

  int i = 0, nr_points = static_cast<int>(cloudFilteredPtr->points.size());
  while (i < 5) // 0.3 * nr_points @TODO: umbral puntos minimos en el plano
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloudFilteredPtr);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      ROS_WARN("Could not estimate a planar model for the given dataset.");
      break;
    }
    else
    {
      ROS_INFO_STREAM_THROTTLE(1, "Segmentation: " << inliers->indices.size());
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudFilteredPtr);
    extract.setIndices(inliers);
    extract.setNegative(true);

    // Get the points associated with objects
    extract.filter(*cloudObj);
    ROS_INFO_STREAM_THROTTLE(
        1, "PointCloud representing the planar component: " << cloudObj->points.size () << " data points. It: " << i);
    ++i;
  }
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloudObj);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.02); // 2cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloudObj);
  ec.extract(cluster_indices);

  // Colors
  uint32_t red = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
  uint32_t green = ((uint32_t)0 << 16 | (uint32_t)255 << 8 | (uint32_t)0);
  uint32_t blue = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)255);

  float colors[] = { *reinterpret_cast<float*>(&red), *reinterpret_cast<float*>(&green), *reinterpret_cast<float*>(&blue)};

  // Colored point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectColored(new pcl::PointCloud<pcl::PointXYZRGB>());

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      pcl::PointXYZRGB p;
      p.x = cloudObj->points[*pit].x;
      p.y = cloudObj->points[*pit].y;
      p.z = cloudObj->points[*pit].z;
      p.rgb = colors[j%3];
      objectColored->points.push_back(p);
    }
    ++j;
  }
  objectColored->width = objectColored->points.size();
  objectColored->height = 1;
  objectColored->is_dense = true;

  ROS_INFO_STREAM_THROTTLE(
      1, "Object clusters: " << cluster_indices.size () << " data points " << objectColored->points.size() << "\n");

  // Convert to ROS data type
  sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*objectColored, *output);

  // Publish the data
  output->header.stamp = ros::Time::now();
  output->header.frame_id = cloud_msg->header.frame_id;
  pub.publish(output);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "object_psd");
  ros::NodeHandle nh;

  // Initialize dynamic config server
  dynamic_reconfigure::Server<object_psd::VoxelGridConfig> server;
  dynamic_reconfigure::Server<object_psd::VoxelGridConfig>::CallbackType cb;
  cb = boost::bind(&config_cb, _1, _2);
  server.setCallback(cb);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  // Spin
  ros::spin();
  return 0;
}
