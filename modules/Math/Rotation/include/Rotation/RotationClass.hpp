/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#pragma once

#include "Rotation/Conversions.hpp"
#include "Rotation/IsValidRotation.hpp"
#include "Rotation/Representations.hpp"

namespace crf::math::rotation {

/**
 * @ingroup group_rotation_rotation_class
 * @brief Class representing spacial rotation. Supports representations in:
 * quaternion, matrix, angle axis, XYZ Cardan angles, ZXZ Euler angles.
 */
class Rotation {
 public:
    /**
     * @brief Constructs a new Rotation object.
     * It creates an identity rotation in the quaternion representation.
     */
    Rotation();

    /**
     * @throws Accuracy -- parameter 'quaternion' is checked if it is an unitary quaternion
     * up to the parameter 'accuracy', otherwise, constructor
     * throws a 'std::invalid_argument' exception.
     */
    explicit Rotation(const Eigen::Quaternion<double>& quaternion, const double& accuracy = 1e-12);

    /**
     * @throws Accuracy -- parameter 'matrix' is checked if it is an rotation matrix
     * up to the parameter 'accuracy', otherwise, constructor
     * throws a 'std::invalid_argument' exception.
     */
    explicit Rotation(const Eigen::Matrix3d& matrix, const double& accuracy = 1e-12);

    explicit Rotation(const Eigen::AngleAxisd& angleAxis);

    explicit Rotation(const CardanXYZ& cardanXYZ);

    explicit Rotation(const EulerZXZ& eulerZXZ);

    Rotation(const Rotation&) = default;

    ~Rotation() = default;

    Rotation& operator=(const Rotation& other) = default;

    /**
     * @brief Sets the value of Rotation object, by specifying quaternion representation.
     * @throw Accuracy -- parameter 'quaternion' is checked if it is an unitary quaternion
     * up to the parameter 'accuracy', otherwise, setter
     * throws a 'std::invalid_argument' exception.
     */
    void setQuaternion(const Eigen::Quaterniond& quaternion, const double& accuracy = 1e-12);

    /**
     * @brief Sets the value of Rotation object, by specifying matrix representation.
     * @throw Accuracy -- parameter 'matrix' is checked if it is an rotation matrix
     * up to the parameter 'accuracy', otherwise, setter
     * throws a 'std::invalid_argument' exception.
     */
    void setMatrix(const Eigen::Matrix3d& matrix, const double& accuracy = 1e-12);

    /**
     * @brief Sets the value of Rotation object, by specifying angle-axis representation.
     */
    void setAngleAxis(const Eigen::AngleAxisd& angleAxis);

    /**
     * @brief Sets the value of Rotation object, by specifying XYZ Cardan angles representation.
     */
    void setCardanXYZ(const CardanXYZ& cardanXYZ);

    /**
     * @brief Sets the value of Rotation object, by specifying ZXZ Euler angles representation.
     */
    void setEulerZXZ(const EulerZXZ& eulerZXZ);

    /**
     * @brief Gets the value of Rotation object in quaternion representation.
     */
    Eigen::Quaterniond getQuaternion() const;

    /**
     * @brief Gets the value of Rotation object in matrix representation.
     */
    Eigen::Matrix3d getMatrix() const;

    /**
     * @brief Gets the value of Rotation object in angle-axis representation.
     */
    Eigen::AngleAxisd getAngleAxis() const;

    /**
     * @brief Gets the value of Rotation object in XYZ Cardan angles representation.
     */
    CardanXYZ getCardanXYZ() const;

    /**
     * @brief Gets the value of Rotation object in ZXZ Euler angles representation.
     */
    EulerZXZ getEulerZXZ() const;

    /**
     * @brief Gets current RotationRepresentation of the Rotation object.
     */
    RotationRepresentation getRepresentation() const;

 private:
    Eigen::Quaterniond quaternion_;
    Eigen::Matrix3d matrix_;
    Eigen::AngleAxisd angleAxis_;
    CardanXYZ cardanXYZ_;
    EulerZXZ eulerZXZ_;
    RotationRepresentation representation_;
};

using Orientation = Rotation;

/**
 * @ingroup group_rotation_rotation_class
 * @brief Creates display string of the parameter 'rotation' and writes it to the parameter 'os'.
 * Display string of parameter 'rotation' will be created
 * accordingly to the current RotationRepresentation of the parameter 'rotation'.
 */
std::ostream& operator<<(std::ostream& os, const Rotation& rotation);

}  // namespace crf::math::rotation
