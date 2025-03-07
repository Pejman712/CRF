#pragma once



#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/impl/instantiate.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/approximate_voxel_grid.hpp>
#include <pcl/filters/impl/radius_outlier_removal.hpp>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/impl/passthrough.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "unitree_lidar_sdk.h"

// Define the Point3D struct with the necessary fields
struct Point3D
{
  PCL_ADD_POINT4D;               // Quad-word XYZ
  //PCL_ADD_INTENSITY; 
  float intensity;             // Intensity field
  std::uint16_t ring;            // Ring field for multi-beam LIDARs
  float time;
  float sharpness;                         // Time field for timestamping
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Ensure proper alignment for Eigen
} EIGEN_ALIGN16;

//PCL_INSTANTIATE(VoxelGrid, Point3D)
//PCL_INSTANTIATE(KdTree, Point3D)
//PCL_INSTANTIATE(RadiusOutlierRemoval, Point3D)

// Register the custom point structure with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(
    Point3D,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (float, sharpness, sharpness)
    (std::uint16_t, ring, ring)
)

