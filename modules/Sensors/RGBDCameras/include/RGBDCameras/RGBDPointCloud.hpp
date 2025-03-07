/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alvaro Garcia Gonzalez BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/rgbd/depth.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace crf::sensors::rgbdcameras {

/**
 * @ingroup group_rgbdcamera
 * @brief Struct to pair an rgbd frame to a pointcloud. It's an utility for
 * representing this combination.
 *
 * @{
 */
struct RGBDPointCloud {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud;
    cv::rgbd::RgbdFrame frame;
};

/**@}*/

}  // namespace crf::sensors::rgbdcameras
