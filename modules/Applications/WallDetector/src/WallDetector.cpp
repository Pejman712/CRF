/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 *
 * The main task of this application is to detect and verify presence of a wall. Initial purpose was
 * to get robot distance to the wall, which would enable robot follow the wall while keeping constant
 * distance to it. The algorithm can be explained like this:
 * 1. clusterize 2D point cloud from lidar (PCL).
 * 2. extract the biggest cluster and verify if its a wall (PCL, RANSAC algorithm).
 * 3. if TRUE (more than certain share of points (INLIER_SHARE_IN_POINTCLOUD) are aligned in a line),
 *    then provide distance to the point, which belongs to wall and is the closest to robot.
 *
 * CLARIFICATION - theta is the angle between line and y-axis, which is assumed to be always
 * parallel to the robot trajectory.
*/

#include <vector>
#include <utility>
#include <exception>
#include <memory>

#include <Eigen/Core>

#include "WallDetector/WallDetector.hpp"

#define DEFAULT_ROBOT_HEIGHT 0.25

namespace crf {
namespace applications {
namespace walldetector {

WallDetector::WallDetector(
  std::shared_ptr<robots::robotbase::RobotBaseConfiguration> robotBaseConfig,
  const nlohmann::json &WallDetectorParams):
    jConfig_(WallDetectorParams),
    robotParameters_(robotBaseConfig->getRobotParameters()),
    filteredCloud_(new pcl::PointCloud<pcl::PointXYZRGBA>),
    logger_("WallDetector") {
    logger_->debug("CTor");
    try {
        clusterDistSeparationTolerance_ = jConfig_.at(
          "CLUSTER_DIST_SEPARATION_TOLERANCE").get<float>();
        minClusterSize_ = jConfig_.at("MIN_CLUSTER_SIZE").get<float>();
        maxClusterSize_ = jConfig_.at("MAX_CLUSTER_SIZE").get<float>();
        ransacAccuracyThreshold_ = jConfig_.at("RANSAC_ACCURACY_THRESHOLD").get<float>();
        inlierShareInPointCloud_ = jConfig_.at("INLIER_SHARE_IN_POINTCLOUD").get<float>();
    } catch (const nlohmann::json::exception& e) {
        logger_->warn("Failed to read config because: {}", e.what());
        throw std::invalid_argument("Could not read config");
    }
}

std::vector<WallParameter> WallDetector::detectWall(
  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudData,
  const utility::types::TaskPose& cameraPose) {
    logger_->debug("detectWall");
    std::vector<WallParameter> parameters;
    if (pointCloudData->points.empty()) {
        logger_->warn("Received empty point cloud");
        return parameters;
    }
    filterRobotSelfReflections(pointCloudData, cameraPose);
    if (filteredCloud_->points.empty()) {
         logger_->warn("filtered cloud empty. Something is wrong.");
         return parameters;
    }
    auto wallClouds = clusterLidarData(filteredCloud_);
    for (size_t ix = 0; ix < wallClouds.size(); ix++) {
         auto lineCoefficients = getLineApproximation(wallClouds[ix]);
         if (lineCoefficients.size() > 0) {
             parameters.push_back(getWallParameters(lineCoefficients, wallClouds[ix], cameraPose));
         }
    }
    return parameters;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> WallDetector::clusterLidarData(
  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& lidarCloud) {
    logger_->debug("clusterLidarData");
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr pclTree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    pclTree->setInputCloud(lidarCloud);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> euclideanClusterExtraction;
    euclideanClusterExtraction.setClusterTolerance(clusterDistSeparationTolerance_);
    euclideanClusterExtraction.setMinClusterSize(minClusterSize_);
    euclideanClusterExtraction.setMaxClusterSize(maxClusterSize_);
    euclideanClusterExtraction.setSearchMethod(pclTree);
    euclideanClusterExtraction.setInputCloud(lidarCloud);
    euclideanClusterExtraction.extract(clusterIndices);
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> wallClusters;
    if (clusterIndices.empty()) {
        logger_->warn("No clusters have been identified - points are too spread.");
        return wallClusters;
    }
    for (size_t i = 0; i < clusterIndices.size(); i++) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr wallClusterCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
        for (size_t j = 0; j < clusterIndices[i].indices.size(); j++) {
            wallClusterCloud->points.push_back(lidarCloud->points[clusterIndices[i].indices[j]]);
        }
        if (!wallClusterCloud->points.empty()) {
            wallClusterCloud->width = wallClusterCloud->points.size();
            wallClusterCloud->height = 1;
            wallClusterCloud->is_dense = true;
            wallClusters.push_back(wallClusterCloud);
        }
    }
    return wallClusters;
}

std::vector<float> WallDetector::getLineApproximation(
  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& wallCloud) {
    logger_->debug("getLineApproximation");
    std::vector<float> taskLineCoefficients;
    if (wallCloud->points.empty()) {
        return taskLineCoefficients;
    }
    std::vector<int> inliers;
    int clusterPointSize = wallCloud->points.size();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr validLinePts (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::SampleConsensusModelLine<pcl::PointXYZRGBA>::Ptr lineModel(
      new pcl::SampleConsensusModelLine<pcl::PointXYZRGBA>(wallCloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac(lineModel);
    ransac.setDistanceThreshold(ransacAccuracyThreshold_);
    ransac.computeModel();
    ransac.getInliers(inliers);
    if (static_cast<float>(inliers.size())/static_cast<float>(clusterPointSize)
        < inlierShareInPointCloud_) {
        logger_->warn("Valid line point share below acceptable: {}% - wall couldn't be identified",
          static_cast<float>(inliers.size())/static_cast<float>(clusterPointSize)*100);
        return taskLineCoefficients;
    }
    Eigen::VectorXf lineCoefficients;
    ransac.getModelCoefficients(lineCoefficients);
    if (lineCoefficients.size() != 6) {
        logger_->error("Unexpected behaviour: line coefficient amount is wrong.");
        return taskLineCoefficients;
    }
    taskLineCoefficients.push_back(lineCoefficients[4]/lineCoefficients[3]);  // get line slope
    taskLineCoefficients.push_back(
      lineCoefficients[1] - taskLineCoefficients[0]*lineCoefficients[0]);  // get line offset
    return taskLineCoefficients;
}

WallParameter WallDetector::getWallParameters(
  const std::vector<float> taskLineCoefficients,
  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& wallCloud,
  const utility::types::TaskPose& cameraPose) {
    logger_->debug("getWallParameters");
    WallParameter wallParameter;
    std::array<float, 12> pointCloudGeneratorPose = cameraPose.getPosRotMatrix();
    pcl::PointXYZRGBA referencePoint;
    referencePoint.x = pointCloudGeneratorPose[0];
    referencePoint.y = pointCloudGeneratorPose[1];
    referencePoint.z = pointCloudGeneratorPose[2];
    wallParameter.distance = std::abs(taskLineCoefficients[1] +
        (taskLineCoefficients[0]*referencePoint.x) - referencePoint.y)/
        sqrt(pow(taskLineCoefficients[0], 2)+1);
    wallParameter.theta = atan(1/taskLineCoefficients[0]);
    wallParameter.length  = pcl::geometry::distance(wallCloud->points[0],
        wallCloud->points[wallCloud->points.size()-1]);
    return wallParameter;
}

void WallDetector::filterRobotSelfReflections(
  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudData,
  const utility::types::TaskPose& cameraPose) {
    logger_->debug("filterRobotSelfReflections");
    filteredCloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    std::array<float, 12> pointCloudGeneratorPose = cameraPose.getPosRotMatrix();
    pcl::PointXYZRGBA referencePoint;
    referencePoint.x = pointCloudGeneratorPose[0];
    referencePoint.y = pointCloudGeneratorPose[1];
    referencePoint.z = pointCloudGeneratorPose[2];
    pcl::PointIndices::Ptr selfRelfectionInliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    for (size_t i = 0; i < pointCloudData->points.size(); i++) {
        float euclideanDistance =
          pcl::geometry::distance(pointCloudData->points[i], referencePoint);
        if (euclideanDistance <
            sqrt(pow(robotParameters_.wheelsDistanceX/2, 2) +
                pow(robotParameters_.wheelsDistanceY/2, 2))) {
                selfRelfectionInliers->indices.push_back(i);
        }
    }
    extract.setInputCloud(pointCloudData);
    extract.setIndices(selfRelfectionInliers);
    extract.setNegative(true);
    extract.filter(*filteredCloud_);
    return;
}

}  // namespace walldetector
}  // namespace applications
}  // namespace crf
