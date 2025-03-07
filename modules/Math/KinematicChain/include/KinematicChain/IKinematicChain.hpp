/* © Copyright CERN 2022. All rights reserved. This software is released under a
 * CERN proprietary software license. Any permission to use it shall be granted
 * in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Ante Marić CERN BE/CEM/MRO 2022
 *          Bartosz Sójka CERN BE/CEM/MRO 2023
 *  ================================================================================================================
 */

#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>
#include "Types/JointTypes/JointPositions.hpp"

#include "KinematicChain/DescriptionTypes.hpp"
#include "KinematicChain/GeometricData.hpp"

namespace crf::math::kinematicchain {

/**
 * @ingroup group_i_kinematic_chain
 * @brief Parses kinematic chain from XML file (either URDF or custom JSON
 * description)
 *
 */
class IKinematicChain {
 public:
    virtual ~IKinematicChain() = default;

    /**
     * @brief returns the type of robot description (Tool, Platform, Arm, or
     * Combined)
     * @return descriptionType_ - type of robot description
     */
    virtual DescriptionType getType() = 0;

    /**
     * @brief returns the type of joint at specified index
     * @param jointIndex represents index of specified joint
     * @return jointType - type of specified joint
     * 0 for UNKNOWN, 1 for REVOLUTE, 2 for CONTINUOUS, 3 for PRISMATIC
     */
    virtual int getJointType(const int& jointIndex) = 0;

    /**
     * @brief returns the size of the kinematic chain
     * @return chainSize_ - number of joints in the kinematic chain
     */
    virtual size_t getChainSize() const = 0;

    /**
     * @brief returns the number of platform wheels
     * @return numWheels_ - number of platform wheels (0 if platform is not
     * present)
     */
    virtual size_t getNumWheels() const = 0;

    /**
     * @brief Returns the wheel radius of the mobile platform; Throws a runtime
     * error if a platform is not present
     * @return wheelRadius_ - wheel radius in meters
     */
    virtual double getWheelRadius() = 0;

    /**
     * @brief Returns longitudinal wheel distance; i.e, distance from the shaft of
     * the front left wheel to the back left wheel; Throws a runtime error if a
     * platform is not present
     * @return platformL_ - longitudinal wheel distance in meters
     */
    virtual double getPlatformL() = 0;

    /**
     * @brief Returns lateral wheel distance; i.e, distance from the shaft of the
     * front left wheel to the front right wheel; Throws a runtime error if a
     * platform is not present
     * @return platformW_ - lateral wheel distance in meters
     */
    virtual double getPlatformW() = 0;

    /**
     * @brief Sets joints to positions from the argument
     * and calculates accordingly all parameters such as rotations and
     * translations.
     * @param jointPositions Configuration containing all joint angles
     */
    virtual void setJointPositions(
        const crf::utility::types::JointPositions& jointPositions) = 0;

    /**
     * @brief Returns requested axis.
     * @param axis Code for a particular axis.
     * @param jointOrWheelN Joint or Wheel number, indexed from 0.
     * Joint are numbered from the base to the end effector,
     * Wheels are numbered couterclockwise, starting from the front right.
     * @return Axis specified by the parameter "axis".
     * It is returned as Eigen::Vector3d.
     */
    virtual Eigen::Vector3d getAxis(const Axes axis,
                                    const int& jointOrWheelN) = 0;

    /**
     * @brief Returns requested translation.
     * @param translation Code for a particular translation.
     * @param jointN Joint number, indexed from 0
     * @return Translation described by the parameter "translation". It is
     * returned as Eigen::Vector3d.
     */
    virtual Eigen::Vector3d getTranslation(const Translations translation,
                                           const int& jointN) = 0;

    /**
     * @brief Returns requested rotation.
     * @param rotation Code for a particular rotation.
     * @param jointN Joint number, indexed from 0
     * @return Rotation described by the parameter "rotation". It is returned as
     * Eigen::Quaterniond.
     */
    virtual Eigen::Quaterniond getRotation(const Rotations rotation,
                                           const int& jointN) = 0;

    /**
     * @brief Returns requested axis.
     * @param axis Code for a particular axis.
     * @param jointOrWheelN Joint or Wheel number, indexed from 0.
     * Joint are numbered from the base to the end effector,
     * Wheels are numbered couterclockwise, starting from the front right.
     * @param jointPositions Configuration containing all joint angles
     * @return Axis specified by the parameter "axis".
     * It is returned as Eigen::Vector3d.
     */
    virtual Eigen::Vector3d computeAxis(
        const Axes axis,
        const int& jointOrWheelN,
        const crf::utility::types::JointPositions& jointPositions) = 0;

    /**
     * @brief Computes geometric data based on the jointPositions
     * and returns requested translation.
     * @param translation Code for a particular translation.
     * @param jointN Joint number, indexed from 0
     * @param jointPositions Configuration containing all joint angles
     * @return Translation described by the parameter translation. It is returned
     * as Eigen::Vector3d.
     */
    virtual Eigen::Vector3d computeTranslation(
        const Translations translation,
        const int& jointN,
        const crf::utility::types::JointPositions& jointPositions) = 0;

    /**
     * @brief Computes geometric data based on the jointPositions
     * and returns requested rotation.
     * @param rotation Code for a particular rotation.
     * @param jointN Joint number, indexed from 0
     * @param jointPositions Configuration containing all joint angles
     * @return Rotation described by the parameter rotation. It is returned as
     * Eigen::Quaterniond.
     */
    virtual Eigen::Quaterniond computeRotation(
        const Rotations rotation,
        const int& jointN,
        const crf::utility::types::JointPositions& jointPositions) = 0;
};

}  // namespace crf::math::kinematicchain
