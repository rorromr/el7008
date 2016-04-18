cmake_minimum_required(VERSION 2.8.3)
project(object_pose_estimation)

find_package(catkin)

#Eigen
include( FindPkgConfig )
pkg_check_modules( EIGEN eigen3 )

# PCL
find_package( PCL )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES object_pose_estimation
)

include_directories( 
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

# Setting stuff
set( fitter_src
  src/InertiaCalculations.cpp
  src/Minimization.cpp
  src/OPESettings.cpp
  src/OPEMain.cpp
  src/ObjectPoseEstimator.cpp
  src/Plane.cpp
  src/PointCloudCapture.cpp
  src/OPEUtils.cpp
  src/SQFitting.cpp
)

set( fitter_hdr
  include/object_pose_estimation/OPECommon.h 
  include/object_pose_estimation/OPESettings.h 
  include/object_pose_estimation/EigenSolver.h 
  include/object_pose_estimation/InertiaCalculations.h 
  include/object_pose_estimation/Minimization.h 
  include/object_pose_estimation/ObjectPoseEstimator.h
  include/object_pose_estimation/Plane.h 
  include/object_pose_estimation/PointCloudCapture.h 
  include/object_pose_estimation/SQFitting.h 
  include/object_pose_estimation/SQTypes.h 
  include/object_pose_estimation/TableObjectDetector.h
  include/object_pose_estimation/TableObjectDetector.hpp
  include/object_pose_estimation/TableObjectModeler.h 
  include/object_pose_estimation/OPEUtils.h 
)

# Add library
add_library( object_pose_estimation ${fitter_src} ${fitter_hdr} )
target_link_libraries( object_pose_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ) 

# Add test
add_subdirectory( tests )

