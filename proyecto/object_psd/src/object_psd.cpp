#include <ros/ros.h>
#include <math.h>
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
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
// VoxelGrid configuration
#include <dynamic_reconfigure/server.h>
#include <object_psd/VoxelGridConfig.h>
// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
// Rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <cmath>

ros::Publisher pub;
double leaf_size;
rviz_visual_tools::RvizVisualToolsPtr visual_tools;

void config_cb(object_psd::VoxelGridConfig &config, uint32_t level)
{
  leaf_size = config.leaf_size;
  ROS_INFO("Reconfigure Leaf Size: %f", leaf_size);
}

void cylinder_matcher(pcl::PointCloud<pcl::PointXYZ>::Ptr object_pc)
{
  // Create the normal estimation class
  pcl::NormalEstimation < pcl::PointXYZ, pcl::Normal > ne;
  // Pass object to it
  ne.setInputCloud(object_pc);

  // Create kd-tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  // Pass kd-tree to the normal estimation object
  ne.setSearchMethod(tree);

  // Output normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch(0.03);

  // Compute normals
  ne.compute(*cloud_normals);

  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(0.05);
  seg.setRadiusLimits(0, 0.1);
  seg.setInputCloud(object_pc);
  seg.setInputNormals(cloud_normals);

  // Obtain the cylinder inliers and coefficients
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  seg.segment(*inliers_cylinder, *coefficients_cylinder);

  //std::cout << *coefficients_cylinder << std::endl;
  Eigen::Quaterniond q;

  Eigen::Vector3d axis_vector(coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5]);
  axis_vector.normalize();

  Eigen::Vector3d up_vector(0.0, 0.0, 1.0);
  Eigen::Vector3d right_axis_vector = axis_vector.cross(up_vector);
  right_axis_vector.normalized();
  double theta = axis_vector.dot(up_vector);
  double angle_rotation = -1.0 * acos(theta);

  tf::Vector3 tf_right_axis_vector;
  tf::vectorEigenToTF(right_axis_vector, tf_right_axis_vector);

  // Create TF quaternion
  tf::Quaternion tf_q(tf_right_axis_vector, angle_rotation);

  // Convert back to Eigen
  tf::quaternionTFToEigen(tf_q, q);

  // Rotate so that vector points along line
  Eigen::Affine3d pose;
  q.normalize();
  pose = q;
  pose.translation() = Eigen::Vector3d(coefficients_cylinder->values[0],
                                       coefficients_cylinder->values[1],
                                       coefficients_cylinder->values[2]);

  double radius = 2*static_cast<double>(coefficients_cylinder->values[6]);

  visual_tools->publishCylinder(pose, rviz_visual_tools::RAND, 0.2, radius);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(object_pc);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ> ());
  extract.filter(*cloud_cylinder);
  if (cloud_cylinder->points.empty ())
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
    // Colors
    uint32_t color = ((uint32_t)50 << 16 | (uint32_t)255 << 8 | (uint32_t)200);
    float colorf = *reinterpret_cast<float*>(&color);

    // Colored point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectColored(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (std::vector<int>::const_iterator it = inliers_cylinder->indices.begin(); it != inliers_cylinder->indices.end(); ++it)
    {
      pcl::PointXYZRGB p;
      p.x = object_pc->points[*it].x;
      p.y = object_pc->points[*it].y;
      p.z = object_pc->points[*it].z;
      p.rgb = colorf;
      objectColored->points.push_back(p);
    }

    objectColored->width = objectColored->points.size();
    objectColored->height = 1;
    objectColored->is_dense = true;

    // Convert to ROS data type
    sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*objectColored, *output);

    // Publish the data
    output->header.stamp = ros::Time::now();
    output->header.frame_id = "bender/sensors/rgbd_head_depth_optical_frame";
    pub.publish(output);
  }
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
      ROS_DEBUG_STREAM_THROTTLE(1, "Segmentation: " << inliers->indices.size());
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudFilteredPtr);
    extract.setIndices(inliers);
    extract.setNegative(true);

    // Get the points associated with objects
    extract.filter(*cloudObj);
    ROS_DEBUG_STREAM_THROTTLE(
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

  // Put clusters in a pointcloud vector
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> objCluster(cluster_indices.size());
  for (std::size_t i = 0; i < cluster_indices.size(); ++i)
  {
    objCluster[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
    objCluster[i]->points.reserve(cluster_indices[i].indices.size());
    for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin(); pit != cluster_indices[i].indices.end(); ++pit)
    {
      objCluster[i]->points.push_back(cloudObj->points[*pit]);
    }
    objCluster[i]->width = objCluster[i]->points.size();
    objCluster[i]->height = 1;
    objCluster[i]->is_dense = true;
    ROS_DEBUG_STREAM("Cluster " << i << " with " << objCluster[i]->points.size());
  }

  // Sphere model
/*  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(
      new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(objCluster[0]));*/


/*  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
  ransac.setDistanceThreshold(.01);
  ransac.computeModel();
  ransac.getInliers(ransac_inliers);*/

  cylinder_matcher(objCluster[1]);

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
  //pub.publish(output);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "object_psd");
  ros::NodeHandle nh;

  // Visual tools
  visual_tools.reset(new rviz_visual_tools::RvizVisualTools("bender/sensors/rgbd_head_depth_optical_frame","/object_marker"));
  visual_tools->setLifetime(0.1);

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
