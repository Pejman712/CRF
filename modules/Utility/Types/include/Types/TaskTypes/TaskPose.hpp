/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Carlos Prados Sesmero CERN EN/SMM/MRO 2020
 *         Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#pragma once

#include <Eigen/Dense>
#include <array>
#include <iomanip>

#include "Rotation/Rotation.hpp"

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;
using crf::math::rotation::Orientation;
using crf::math::rotation::OrientationRepresentation;
using crf::math::rotation::operator<<;

namespace crf::utility::types {

/**
 * @ingroup group_task_types
 * @brief Class representing spacial pose -- position and orientation.
 */
class TaskPose {
 public:
    /**
     * @brief Construct a new Task Pose object.
     * It creates an identity TaskPose -- position (0, 0, 0)
     * and identity orientation. Orientation is constructed in the quaternion representation.
     */
    TaskPose();

    /**
     * @throws Accuracy -- parameter 'homogeneousTransformationMatrix' is checked if it is
     * a homogeneous transformation matrix up to the parameter 'accuracy',
     * otherwise, a constructor throws a 'std::invalid_argument' exception.
     */
    explicit TaskPose(
        const Eigen::Matrix4d& homogeneousTransformationMatrix,
        const double& accuracy = 1e-12);

    explicit TaskPose(const Eigen::Vector3d& position, const Orientation& orientation);

    /**
     * @throws Accuracy -- parameter 'quaternion' is checked if it is
     * an unitary quaternion up to the parameter 'accuracy',
     * otherwise, constructor throws a 'std::invalid_argument' exception.
     */
    explicit TaskPose(
        const Eigen::Vector3d& position,
        const Eigen::Quaternion<double>& quaternion,
        const double& accuracy = 1e-12);

    /**
     * @throws Accuracy -- parameter 'rotationmatrix' is checked if it is
     * a rotation matrix up to the parameter 'accuracy',
     * otherwise, constructor throws a 'std::invalid_argument' exception.
     */
    explicit TaskPose(
        const Eigen::Vector3d& position,
        const Eigen::Matrix3d& rotationMatrix,
        const double& accuracy = 1e-12);

    explicit TaskPose(const Eigen::Vector3d& position, const Eigen::AngleAxisd& angleAxis);

    explicit TaskPose(const Eigen::Vector3d& position, const CardanXYZ& cardanXYZ);

    explicit TaskPose(const Eigen::Vector3d& position, const EulerZXZ& eulerZXZ);

    TaskPose(const TaskPose&) = default;

    ~TaskPose() = default;

    TaskPose& operator=(const TaskPose& other) = default;

    /**
     * @brief Sets the value of the orientation in TaskPose object, by specifying
     * homogeneous transformation matrix representation.
     * @throw Accuracy -- parameter 'homogeneousTransformationMatrix'
     * is checked if it is a homogeneous transformation matrix up to the parameter 'accuracy',
     * otherwise, setter throws a 'std::invalid_argument' exception.
     */
    void setHomogeneousTransformationMatrix(
        const Eigen::Matrix4d& homogeneousTransformationMatrix,
        const double& accuracy = 1e-12);

    /**
     * @brief Sets the value of the position in TaskPose object, using position vector.
     */
    void setPosition(const Eigen::Vector3d& position);

    /**
     * @brief Sets the value of the orientation in TaskPose object, using Orientation object.
     */
    void setOrientation(const Orientation& orientation);

    /**
     * @brief Sets the value of the orientation in TaskPose object, by specifying
     * quaternion representation.
     * @throw Accuracy -- parameter 'quaternion' is checked if it is
     * an unitary quaternion up to the parameter 'accuracy',
     * otherwise, setter throws a 'std::invalid_argument' exception..
     */
    void setQuaternion(const Eigen::Quaterniond& quaternion, const double& accuracy = 1e-12);

    /**
     * @brief Sets the value of the orientation in TaskPose object, by specifying
     * rotation matrix representation.
     * @throw Accuracy -- parameter 'rotationMatrix' is checked if it is
     * a rotation matrix up to the parameter 'accuracy',
     * otherwise, setter throws a 'std::invalid_argument' exception.
     */
    void setRotationMatrix(const Eigen::Matrix3d& rotationMatrix, const double& accuracy = 1e-12);

    /**
     * @brief Sets the value of the orientation in TaskPose object, by specifying
     * angle-axis representation.
     */
    void setAngleAxis(const Eigen::AngleAxisd& angleAxis);

    /**
     * @brief Sets the value of the orientation in TaskPose object, by specifying
     * XYZ Cardan angles representation.
     */
    void setCardanXYZ(const CardanXYZ& cardanXYZ);

    /**
     * @brief Sets the value of the orientation in TaskPose object, by specifying
     * ZXZ euler angles representation.
     */
    void setEulerZXZ(const EulerZXZ& eulerZXZ);

    /**
     * @brief  Gets the value of the TaskPose in homogeneous transformation matrix representation.
     */
    Eigen::Matrix4d getHomogeneousTransformationMatrix() const;

    /**
     * @brief Gets the value of position from TaskPose object as position vector.
     */
    Eigen::Vector3d getPosition() const;

    /**
     * @brief Gets the value of orientation from TaskPose object as Orientation object.
     */
    Orientation getOrientation() const;

    /**
     * @brief Gets the value of orientation from TaskPose object in quaternion representation.
     */
    Eigen::Quaterniond getQuaternion() const;

    /**
     * @brief Gets the value of orientation from TaskPose object in matrix representation.
     */
    Eigen::Matrix3d getRotationMatrix() const;

    /**
     * @brief Gets the value of orientation from TaskPose object in angle-axis representation.
     */
    Eigen::AngleAxisd getAngleAxis() const;

    /**
     * @brief Gets the value of orientation from TaskPose object
     * in XYZ Cardan angles representation.
     */
    CardanXYZ getCardanXYZ() const;

    /**
     * @brief Gets the value of orientation from TaskPose object
     * in ZXZ Euler angles representation.
     */
    EulerZXZ getEulerZXZ() const;

    /**
     * @brief Gets current OrientationRepresentation of the Orientation object in TaskPose object.
     */
    OrientationRepresentation getOrientationRepresentation() const;

 private:
    Eigen::Vector3d position_;
    Orientation orientation_;
};

/**
 * @ingroup group_types_task_pose
 * @brief Creates display string of the parameter 'taskPose' and writes it to the parameter 'os'.
 * Display string of parameter 'taskPose' will be created
 * accordingly to the current OrientationRepresentation of the Orientation object in
 * parameter 'taskPose'.
 */
std::ostream& operator<<(std::ostream& os, const TaskPose& taskPose);

}  // namespace crf::utility::types
