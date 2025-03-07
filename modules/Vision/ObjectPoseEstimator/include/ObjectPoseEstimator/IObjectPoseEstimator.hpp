#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include "ObjectPoseEstimator/PoseEstimationData.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/optional.hpp>
#include <vector>

namespace crf {
namespace applications {
namespace objectposeestimator {
class IObjectPoseEstimator {
 public:
    virtual ~IObjectPoseEstimator() = default;
    /**
     * @brief compute targetPointcloud current position in its own coordinate system using the
     * prescan object model
     * Both pointclouds should not be previously aligned
     * Returns:
     * -current PoseEstimationData for the targetPointcloud if it arrives to a solution
     * (don't mix up with the best estimated pose)
     * -boost::none if it is not possible to compute a pose for the targetPointcloud
     **/
    virtual boost::optional<PoseEstimationData> computeTargetPose(
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetPointCloud) = 0;
    /**
     * @brief best fitness score getter 
     * fitness score values: [0,1]
     **/
    virtual float getBestFitnessScore() = 0;
    /**
     * @brief reset best fitness score current value to 0
     * Returns:
     * -true if the last best fitness score value was not 0.0 and the reset is done
     * -false if the best fitness score value was 0.0 which means not initialized and
     * not able to be reset
    **/
    virtual bool clearBestFitnessScore() = 0;
    /**
     * @brief best fitness score getter 
     * fitness score values: [0,1]
     **/
    virtual float getBestInverseFitnessScore() = 0;
    /**
     * @brief reset best inverse fitness score current value to 0
     * Returns:
     * -true if the last best fitness score value was not 0.0 and the reset is done
     * -false if the best fitness score value was 0.0 which means not initialized and
     * not able to be reset
    **/
    virtual bool clearBestInverseFitnessScore() = 0;
    /**
     * @brief best compose fitness score getter 
     * fitness score values: [0,2]
     * computed as: bestFitnessScore^2 + bestInverseFitnessScore^2
     **/
    virtual float getBestComposeFitnessScore() = 0;
    /**
     * @brief reset best compose fitness score current value to 0
     * Returns:
     * -true if the last best fitness score value was not 0.0 and the reset is done
     * -false if the best fitness score value was 0.0 which means not initialized and
     * not able to be reset
    **/
    virtual bool clearBestComposeFitnessScore() = 0;
    /**
     * @brief best estimated pose getter
    **/
    virtual std::array<float, 12> getBestPose() = 0;
    /**
     * @brief reset best pose current value to the identity
     * Returns:
     * -true if the last best pose value was not the identity and the reset is done
     * -false if the best pose value was the identity which means just initialized and
     * not able to be reset
    **/
    virtual bool clearBestPose() = 0;
    /**
     * @brief obtain current main cluster extracted from the targetPointcloud feed into
     * computeTargetPose
     * Returns:
     * -
     * -boost::none if the current object cluster is empty
     * -current object cluster as a pointcloud if it is not empty
    **/
    virtual boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>
        getCurrentObjectCluster() = 0;
    /**
     * @brief reset current object cluster to an empty pointcloud
     * Returns:
     * -true if current object cluster is not empty and the reset could be done
     * -false if the current object cluster is empty and the reset could not be done
    **/
    virtual bool clearCurrentObjectCluster() = 0;
    /**
     * @brief obtain prescan object model currently being used to compute target pose
     * Returns:
     * -object model pointcloud if preScanObjectPath is initialized
     * -boost::none if preScanObjectPath is not initialized
    **/
    virtual boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>
        getObjectModel() = 0;
    /**
     * @brief best oriented bounding box getter
    **/
    virtual Object3DBoundingBox getBestObject3DBoundingBox() = 0;
    /**
     * @brief reset best oriented bounding box values to the initialization ones
     * Returns:
     * -true if the last best oriented bounding box was different from a cleared one and
     * the reset is done
     * -false if the best oriented bounding box values were equal to a cleared one which means
     * just initialized and not able to be reset
    **/
    virtual bool clearBest3DObjectBoundingBox() = 0;
    /**
     * @brief set icpRefinement flag value during run time
     * Returns:
     * -true if the icpFlag value has been successfully set
     * -false if the icpFlag value was already set and thus not set again
    **/
    virtual bool setICPRefinement(bool icpFlag) = 0;
};
}  // namespace objectposeestimator
}  // namespace applications
}  // namespace crf
