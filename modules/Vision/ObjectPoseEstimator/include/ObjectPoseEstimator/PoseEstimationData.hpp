#pragma once
/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include <array>
#include <pcl/point_types.h>

namespace crf {
namespace applications {
namespace objectposeestimator {


struct PoseEstimationData {
    /**
     * @brief pose containis the rotation and translation matrices values of the object
     * array 12x1 containing rotation and translation matrices values
     * first 9 values containing rotation matrix in row major order and 
     * last 3 values containing translation matrix in column major order
     */
    std::array<float, 12> pose;
    /**
     * @brief percentage of overlapping points in between target and source clouds
     */
    float fitnessScore;
    /**
    * @brief empty constructor
    */
    PoseEstimationData() {
        pose = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
        fitnessScore = 0.0f;
    }
    /**
    * @brief parameterized constructor
    */
    PoseEstimationData(std::array<float, 12> pose_, float fitnessScore_): pose(pose_),
        fitnessScore(fitnessScore_) {}
};


/**
 * @brief struct containing necessary information to draw an oriented bounding box with respect
 * to the world coordinate system
 * Pay attention to the fact that this is not the minimal possible bounding box
 * This is the bounding box which is oriented in accordance with the eigen vectors
*/
struct Object3DBoundingBox {
    /**
     * @brief min point of the OBB
     */
    pcl::PointXYZRGBNormal minPointOBB;
    /**
     * @brief max point of the OBB
     */
    pcl::PointXYZRGBNormal maxPointOBB;
    /**
     * @brief position of the OBB center of mass
     */
    pcl::PointXYZRGBNormal positionOBB;
    /**
     * @brief matrix representing the rotation transform  from the world coordinate system
     * to the coordinate system comprised of Eigen values 
     */
    Eigen::Matrix3f rotationalMatrixOBB;
    /**
    * @brief empty constructor
    */
    Object3DBoundingBox() {
        pcl::PointXYZRGBNormal zeroPoint;
        zeroPoint.x = 0.0;
        zeroPoint.y = 0.0;
        zeroPoint.z = 0.0;
        Eigen::Matrix3f rotationalMatrixOBB_ = Eigen::Matrix3f::Identity();
        // rotationalMatrixOBB_.setIdentity();
        minPointOBB = zeroPoint;
        maxPointOBB = zeroPoint;
        positionOBB = zeroPoint;
        rotationalMatrixOBB = rotationalMatrixOBB_;
    }
    /**
    * @brief parameterized constructor
    */
    Object3DBoundingBox(pcl::PointXYZRGBNormal minPointOBB_, pcl::PointXYZRGBNormal maxPointOBB_,
        pcl::PointXYZRGBNormal positionOBB_, Eigen::Matrix3f rotationalMatrixOBB_):
        minPointOBB(minPointOBB_), maxPointOBB(maxPointOBB_), positionOBB(positionOBB_),
        rotationalMatrixOBB(rotationalMatrixOBB_) {}
};

}  // namespace objectposeestimator
}  // namespace applications
}  // namespace crf
