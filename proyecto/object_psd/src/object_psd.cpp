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
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

// Configuration
#include <dynamic_reconfigure/server.h>
#include <object_psd/PSDConfig.h>
// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
// Rviz
#include <rviz_visual_tools/rviz_visual_tools.h>
// Shape messages
#include <object_psd/ObjectShape.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include <object_psd/pcl_util.h>

#include <object_pose_estimation/SQTypes.h>
#include <object_pose_estimation/ObjectPoseEstimator.h>
#include <boost/thread.hpp>
#include <object_psd/sample_superquadric.h>

ros::Publisher pub, superquadricPub, shapePub;
// Parameters
double leaf_size; // Leaf size for voxel grid
int ransac_it; // RANSAC Max iter
// Threshold
double sphere_th;
double cylinder_th;
double match_th;
// Pose factors
double phi_factor;
double theta_factor;
double psi_factor;

double x_pt_filter, y_pt_filter, z_pt_filter;

object_psd::ObjectShape::Ptr objects;

rviz_visual_tools::RvizVisualToolsPtr visual_tools;

void config_cb(object_psd::PSDConfig &config, uint32_t level)
{
  leaf_size = config.leaf_size;
  ransac_it = config.ransac_it;
  sphere_th = config.sphere_th;
  cylinder_th = config.cylinder_th;
  match_th = config.match_th;
  phi_factor = config.phi_factor;
  theta_factor = config.theta_factor;
  psi_factor = config.psi_factor;
  x_pt_filter = config.x;
  y_pt_filter = config.y;
  z_pt_filter = config.z;
  ROS_INFO("Reconfigure Leaf Size: %f", leaf_size);
}

float sphere_matcher(pcl::PointCloud<pcl::PointXYZ>::Ptr object_pc)
{
  pcl::ModelCoefficients::Ptr sphere_coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_SPHERE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(ransac_it);
  seg.setDistanceThreshold(sphere_th);
  seg.setRadiusLimits(0.01, 0.15);
  seg.setInputCloud(object_pc);
  seg.segment(*inliers, *sphere_coefficients);

  double match = static_cast<double>(inliers->indices.size()) / object_pc->points.size();

  if (match < match_th)
  {
    return match;
  }
  Eigen::Vector3d center(sphere_coefficients->values[0], sphere_coefficients->values[1],
                         sphere_coefficients->values[2]);
  double radius = 2 * static_cast<double>(sphere_coefficients->values[3]);
  visual_tools->publishSphere(center, rviz_visual_tools::GREY, radius);

  geometry_msgs::Pose pose_msg;
  Eigen::Affine3d pose(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  pose.translation() = center;
  tf::poseEigenToMsg(pose, pose_msg);

  shape_msgs::SolidPrimitive sphere_msg;
  sphere_msg.type = shape_msgs::SolidPrimitive::SPHERE;
  sphere_msg.dimensions.resize(1);
  sphere_msg.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = radius;

  objects->primitives.push_back(sphere_msg);
  objects->primitive_poses.push_back(pose_msg);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(object_pc);
  extract.setIndices(inliers);
  extract.setNegative(false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  extract.filter(*cloud);
  if (cloud->points.empty())
  {
    std::cerr << "Can't find the sphere component." << std::endl;
    return 0.0;
  }
  else
  {
    // Colors
    float colorf = pcl_util::getColor(pcl_util::CYAN);

    // Colored point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectColored(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (std::vector<int>::const_iterator it = inliers->indices.begin(); it != inliers->indices.end(); ++it)
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
  return match;
}

double cylinder_matcher(pcl::PointCloud<pcl::PointXYZ>::Ptr object_pc)
{
  // Create the normal estimation class
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
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
  seg.setMaxIterations(ransac_it);
  seg.setDistanceThreshold(cylinder_th);
  seg.setRadiusLimits(0.01, 0.1);
  seg.setInputCloud(object_pc);
  seg.setInputNormals(cloud_normals);

  // Obtain the cylinder inliers and coefficients
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  seg.segment(*inliers_cylinder, *coefficients_cylinder);

  double match = static_cast<double>(inliers_cylinder->indices.size()) / object_pc->points.size();

  if (match < match_th)
  {
    return match;
  }

  Eigen::Vector3d axis_vector(coefficients_cylinder->values[3], coefficients_cylinder->values[4],
                              coefficients_cylinder->values[5]);
  axis_vector.normalize();

  Eigen::Vector3d up_vector = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d right_axis_vector = axis_vector.cross(up_vector);
  right_axis_vector.normalized();
  double theta = axis_vector.dot(up_vector);
  double angle_rotation = acos(theta);

  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(angle_rotation, right_axis_vector);
  q.normalize();

  // Rotate so that vector points along line
  Eigen::Affine3d pose;
  pose = q * Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitX());
  pose.translation() = Eigen::Vector3d(coefficients_cylinder->values[0], coefficients_cylinder->values[1],
                                       coefficients_cylinder->values[2]);

  double radius = 2.0 * static_cast<double>(coefficients_cylinder->values[6]);

  visual_tools->publishCylinder(pose, rviz_visual_tools::RAND, 0.2, radius);

  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(pose, pose_msg);

  shape_msgs::SolidPrimitive cylinder_msg;
  cylinder_msg.type = shape_msgs::SolidPrimitive::CYLINDER;
  cylinder_msg.dimensions.resize(2);
  cylinder_msg.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.2;
  cylinder_msg.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;

  objects->primitives.push_back(cylinder_msg);
  objects->primitive_poses.push_back(pose_msg);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(object_pc);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
  extract.filter(*cloud_cylinder);
  if (cloud_cylinder->points.empty())
  {
    std::cerr << "Can't find the cylindrical component." << std::endl;
    return 0.0;
  }
  else
  {
    // Colors
    uint32_t color = ((uint32_t)50 << 16 | (uint32_t)255 << 8 | (uint32_t)200);
    float colorf = *reinterpret_cast<float*>(&color);

    // Colored point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectColored(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (std::vector<int>::const_iterator it = inliers_cylinder->indices.begin(); it != inliers_cylinder->indices.end();
        ++it)
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
  return match;
}

bool expectNear(double value, double expectedValue, double eps = 0.1)
{
  return std::abs(value - expectedValue) < eps;
}

bool
quadraticSphereMatcher(const ope::SQParameters& param,
                       const std::string& frame_id = "camera_depth_optical_frame")
{
  if (expectNear(param.e1, 1.0) && expectNear(param.e2, 1.0))
  {
    // Calc diameter from parameters
    double diameter = 2.0/3.0 * (param.a1 + param.a3 + param.a3);
    ROS_INFO("Sphere detected (d = %.3f) [%.3f, %.3f, %.3f]", diameter, param.px, param.py, -param.pz);
    // Calc pose
    Eigen::Vector3d center(param.px, param.py, param.pz); // Center

    visual_tools->setBaseFrame(frame_id);
    visual_tools->publishSphere(center, rviz_visual_tools::RAND, diameter);
    return true;
  }
  return false;
}

bool
quadraticCuboidMatcher(const ope::SQParameters& param,
                       const std::string& frame_id = "camera_depth_optical_frame")
{
  if (expectNear(param.e1, 0.1) && expectNear(param.e2, 0.1))
  {
    using namespace Eigen;
    // Calc dimensions from parameters
    double depth = 2 * param.a1, width = 2 * param.a2, height = 2 * param.a3;
    // Calc pose from parameters
    Affine3d pose = Translation3d(param.px, param.py, param.pz)
        * Eigen::AngleAxisd(phi_factor * param.phi, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(theta_factor * param.theta, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(psi_factor * param.psi, Eigen::Vector3d::UnitZ());

    // Publish shape
    visual_tools->setBaseFrame(frame_id);
    visual_tools->publishCuboid(pose, depth, width, height, rviz_visual_tools::RAND);
    visual_tools->triggerBatchPublish();
    return true;
  }
  return false;
}

bool
quadraticCylinderMatcher(const ope::SQParameters& param,
                         const std::string& frame_id = "camera_depth_optical_frame")
{
  using namespace Eigen;
  // Calc height and radius from parameters
  ROS_INFO_STREAM(param);
  double height = 2 * param.a3;
  double radius = param.a1 + param.a2;

  // Calc pose from parameters
  Affine3d pose = Translation3d(param.px, param.py, param.pz)
      * Eigen::AngleAxisd(phi_factor * param.phi, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(theta_factor * param.theta, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(psi_factor * param.psi, Eigen::Vector3d::UnitZ());
  // Publish shape
  visual_tools->setBaseFrame(frame_id);
  visual_tools->publishCylinder(pose, rviz_visual_tools::RAND, height, radius);
  visual_tools->triggerBatchPublish();
  return true;
}

void
quadraticMatcher(pcl::PointCloud<pcl::PointXYZ>::Ptr& obj,
		   const std::string& frame_id = "camera_depth_optical_frame")
{
  ope::OPESettings settings;
  settings.minTgtDepth = 0.4f;
  settings.maxObjHeight = 2.5f;

  ope::ObjectPoseEstimator estimator(settings);
  ope::SQParameters sqParams = estimator.calculateObjectPose(*obj);

  object_psd::SuperquadricParameters<double> visual_params;
  visual_params.a = sqParams.a1;
  visual_params.b = sqParams.a2;
  visual_params.c = sqParams.a3;
  visual_params.e1 = sqParams.e1;
  visual_params.e2 = sqParams.e2;
  Eigen::Affine3d pose = Eigen::Translation3d(sqParams.px, sqParams.py, sqParams.pz)
        * Eigen::AngleAxisd(sqParams.phi, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(sqParams.theta, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(sqParams.psi, Eigen::Vector3d::UnitZ());
  visual_params.transform = pose.matrix();

  // Colored point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr supercuadric_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  object_psd::SuperquadricSampling<pcl::PointXYZRGB, double> sampling;
  sampling.setParameters(visual_params);
  sampling.generatePointCloud(*supercuadric_pointcloud);
  // Set point cloud color
  float color = pcl_util::getRandomColor();
  for (std::size_t i = 0; i < supercuadric_pointcloud->points.size(); ++i)
  {
	supercuadric_pointcloud->points[i].rgb = color;
  }
  // Convert to ROS data type
  sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
  pcl::toROSMsg<pcl::PointXYZRGB>(*supercuadric_pointcloud, *output);
  output->header.stamp = ros::Time::now();
  output->header.frame_id = frame_id;
  superquadricPub.publish(output);

  bool match = quadraticSphereMatcher(sqParams, frame_id);
  if (!match)
    match = quadraticCuboidMatcher(sqParams, frame_id);
  if (!match)
    match = quadraticCylinderMatcher(sqParams, frame_id);
  // Print parameters
  ROS_DEBUG_STREAM(sqParams);
}

void
publishCluster(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& object_cluster,
			   const std::string& frame_id = "camera_depth_optical_frame")
{
	// Colored point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_colored(new pcl::PointCloud<pcl::PointXYZRGB>());

	for (std::size_t i = 0; i < object_cluster.size(); ++i)
	{
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster = object_cluster[i];
	  float selected_color = pcl_util::getRandomColor();
	  for (std::size_t j = 0; j < cluster->points.size(); ++j)
	  {
		pcl::PointXYZRGB p;
		p.x = cluster->points[j].x;
		p.y = cluster->points[j].y;
		p.z = cluster->points[j].z;
		p.rgb = selected_color;
		object_colored->points.push_back(p);
	  }
	}
	object_colored->width = object_colored->points.size();
	object_colored->height = 1;
	object_colored->is_dense = true;

	// Convert to ROS data type
	sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
	pcl::toROSMsg<pcl::PointXYZRGB>(*object_colored, *output);

	// Publish the data
	output->header.stamp = ros::Time::now();
	output->header.frame_id = frame_id;
	pub.publish(output);
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilteredPtr(new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCL data type
  pcl::fromROSMsg<pcl::PointXYZ>(*cloud_msg, *cloudPtr);

  // Perform voxel grid filtering
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudFilteredPtr);

  // PassThrough filter (x axis)
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(-x_pt_filter, x_pt_filter);

  // PassThrough filter (y axis)
  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-y_pt_filter, y_pt_filter);

  // PassThrough filter (z axis)
  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(-z_pt_filter, z_pt_filter);

  // Filter on x axis
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_x (new pcl::PointCloud<pcl::PointXYZ>);
  pass_x.setInputCloud(cloudFilteredPtr);
  pass_x.filter(*cloud_after_x);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_y (new pcl::PointCloud<pcl::PointXYZ>);
  pass_y.setInputCloud(cloud_after_x);
  pass_y.filter(*cloud_after_y);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_z (new pcl::PointCloud<pcl::PointXYZ>);
  pass_z.setInputCloud (cloud_after_y);
  pass_z.filter(*cloud_after_z);

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

  int i = 0, nr_points = static_cast<int>(cloud_after_z->points.size());
  // TODO: umbral puntos minimos en el plano
  while (i < 5) // 0.3 * nr_points
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_after_z);
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
    extract.setInputCloud(cloud_after_z);
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
    for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin();
        pit != cluster_indices[i].indices.end(); ++pit)
    {
      objCluster[i]->points.push_back(cloudObj->points[*pit]);
    }
    objCluster[i]->width = objCluster[i]->points.size();
    objCluster[i]->height = 1;
    objCluster[i]->is_dense = true;
    objCluster[i]->header =  cloudPtr->header;
    ROS_INFO_STREAM("Cluster " << i << " with " << objCluster[i]->points.size());
  }

  boost::thread_group thread_group;
  for (std::size_t i = 0; i < objCluster.size(); ++i)
  {
    if (objCluster[i]->size() < 13)
    {
      ROS_WARN("Cluster with less than 13 points!");
      continue;
    }
    //quadraticMatcher(objCluster[i]);
    thread_group.create_thread(boost::bind(&quadraticMatcher, objCluster[i],cloud_msg->header.frame_id));
  }
  thread_group.join_all();

  // Publish cluster point cloud
  publishCluster(objCluster, cloud_msg->header.frame_id);
}



void publishShape(const ros::TimerEvent& event)
{
  if (objects->primitives.size())
  {
    shapePub.publish(objects);
    objects->primitives.clear();
    objects->primitive_poses.clear();
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "object_psd");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // Get camera frame
  std::string camera_frame;
  nh_priv.param<std::string>("camera_frame", camera_frame, "camera_depth_optical_frame");
  ROS_INFO("Using frame '%s' as camera_frame.", camera_frame.c_str());

  // Visual tools
  visual_tools.reset(new rviz_visual_tools::RvizVisualTools(camera_frame, "object_marker"));
  visual_tools->setLifetime(0.2);

  // Shape msg
  objects.reset(new object_psd::ObjectShape());
  objects->header.frame_id = camera_frame;

  ros::Timer timer = nh.createTimer(ros::Duration(0.1), publishShape);
  shapePub = nh.advertise<object_psd::ObjectShape>("shape", 1);

  // Initialize dynamic config server
  dynamic_reconfigure::Server<object_psd::PSDConfig> server;
  dynamic_reconfigure::Server<object_psd::PSDConfig>::CallbackType cb;
  cb = boost::bind(&config_cb, _1, _2);
  server.setCallback(cb);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Create a ROS publisher for the output point cloud
  superquadricPub = nh.advertise<sensor_msgs::PointCloud2>("superquadric", 1);

  // Spin
  ros::spin();
  return 0;
}
