#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */
#include "EventLogger/EventLogger.hpp"
#include "ObjectPoseEstimator/IObjectPoseEstimator.hpp"
#include "ObjectPoseEstimator/PoseEstimationData.hpp"
#include "ObjectPoseEstimator/ObjectPoseEstimatorConfiguration.hpp"

#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/impl/point_types.hpp>
#include <boost/optional.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <array>


namespace crf {
namespace applications {
namespace objectposeestimator {

class ObjectPoseEstimator: public IObjectPoseEstimator {
 public:
    ObjectPoseEstimator() = delete;
    ObjectPoseEstimator(const ObjectPoseEstimator& other) = delete;
    ObjectPoseEstimator(ObjectPoseEstimator&& other) = delete;
    explicit ObjectPoseEstimator(const std::string& configFileName);
    ~ObjectPoseEstimator() override;
    boost::optional<PoseEstimationData> computeTargetPose(
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetPointCloud) override;
    float getBestFitnessScore() override;
    bool clearBestFitnessScore() override;
    float getBestInverseFitnessScore() override;
    bool clearBestInverseFitnessScore() override;
    float getBestComposeFitnessScore() override;
    bool clearBestComposeFitnessScore() override;
    std::array<float, 12> getBestPose() override;
    bool clearBestPose() override;
    boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>
        getCurrentObjectCluster() override;
    bool clearCurrentObjectCluster() override;
    boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> getObjectModel() override;
    Object3DBoundingBox getBestObject3DBoundingBox() override;
    bool clearBest3DObjectBoundingBox() override;
    bool setICPRefinement(bool icpFlag);

 private:
    crf::utility::logger::EventLogger log_;
    ObjectPoseEstimatorConfiguration configuration_;
    float bestFitnessScore_;
    float bestInverseFitnessScore_;
    float bestComposeFitnessScore_;
    std::array<float, 12> bestPose_;
    Object3DBoundingBox best3DBoundingBox_;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sourcePointCloud_;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr currentObjectCluster_;

    boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> extractCluster(
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetPointCloud);
    boost::optional<Eigen::Matrix4f> estimateMassCenter(
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetPointCloud);
    boost::optional<Object3DBoundingBox> estimate3DBoundingBox(
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetPointCloud);
    float computeFitnessScore(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetPointCloud,
        const std::vector<int> &inliers);
    boost::optional<Eigen::Matrix4f> icpAligment(
        const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &tgtFullCloud);
    boost::optional<Eigen::Matrix4f> initialAligment(
        const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr srcPointCloud,
        const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tgtPointCloud);
    float getFitness(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &targetPointCloud,
        float inlierTreshold);
    float getInverseFitness(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &targetPointCloud,
        float inlierTreshold);
    float getComposeFitness(float fitnessScore, float inverseFitnessScore);
};

}  // namespace objectposeestimator
}  // namespace applications
}  // namespace crf

