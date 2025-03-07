#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <utility>
#include <functional>
#include <cmath>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/extract_clusters.h>

#include <nlohmann/json.hpp>

#include "Types/TaskTypes/TaskPose.hpp"
#include "EventLogger/EventLogger.hpp"
#include "WallDetector/IWallDetector.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"

namespace crf {
namespace applications {
namespace walldetector {

class WallDetector : public IWallDetector {
 public:
    WallDetector(
      std::shared_ptr<robots::robotbase::RobotBaseConfiguration> robotBaseConfig,
      const nlohmann::json &WallDetectorParams);
    WallDetector() = delete;
    WallDetector(const WallDetector& other) = delete;
    WallDetector(WallDetector&& other) = delete;
    ~WallDetector() override = default;
    std::vector<WallParameter> detectWall(
      const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudData,
      const utility::types::TaskPose& cameraPose) override;

 private:
    nlohmann::json jConfig_;
    robots::robotbase::RobotParameters robotParameters_;
    utility::logger::EventLogger logger_;
    float clusterDistSeparationTolerance_, minClusterSize_,
      maxClusterSize_, ransacAccuracyThreshold_, inlierShareInPointCloud_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusterLidarData(
      const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& lidarCloud);
    std::vector<float> getLineApproximation(
      const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& wallCloud);
    WallParameter getWallParameters(
      const std::vector<float> taskLineCoefficients,
      const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& wallCloud,
      const utility::types::TaskPose& cameraPose);
    void filterRobotSelfReflections(
      const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudData,
      const utility::types::TaskPose& cameraPose);
};

}  // namespace walldetector
}  // namespace applications
}  // namespace crf
