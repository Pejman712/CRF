/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <iomanip>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

namespace crf::utility::types {

/**
 * @ingroup group_joint_types
 * @brief Composition from Eigen::VectorXd for strongly typing vectors of arbitrary length.
 * Base class for types:
 *     - JointPositions
 *     - JointVelocities
 *     - JointAccelerations
 *     - JointForceTorques.
 *
 */
class VectorXd {
 public:
    /**
     * @brief Construct a VectorXd object with no coordinates.
     */
    VectorXd();

    VectorXd(const VectorXd&) = default;

    /**
     * @brief Construct a VectorXd object with the number of coordinates
     * equal to the parameter 'dimension' with 0.0 on all coordinates.
     * @throw Dimension -- Parameter 'dimension' needs to be >= 0, otherwise, constructor
     * throws a std::invalid_argument exception.
     */
    explicit VectorXd(const std::size_t dimension);

    explicit VectorXd(const Eigen::VectorXd& coordinates);

    explicit VectorXd(const std::vector<double>& coordinates);

    explicit VectorXd(const std::initializer_list<double>& coordinates);

    ~VectorXd() = default;

    VectorXd& operator=(const VectorXd& other) = default;

    VectorXd& operator=(const Eigen::VectorXd& coordinates);

    VectorXd& operator=(const std::vector<double>& coordinates);

    VectorXd& operator=(const std::initializer_list<double>& coordinates);

    /**
     * @brief Returns number of coordinates.
     */
    std::size_t size() const;

    /**
     * @brief VectorXd does not support direct arithmetical operations.
     * This function returns correponding Eigen::VectorXd object,
     * on which such operations can be performed.
     */
    Eigen::VectorXd raw() const;

    /**
     * @brief Non-Const version of the index operator.
     * @throw Range -- parameter 'index' is checked if it is within the range [0, size()],
     * otherwise, operator throws a 'std::out_of_range' exception.
     */
    double& operator[](const std::size_t index);

    /**
     * @brief Const version of the index operator.
     * @throw Range -- parameter 'index' is checked if it is within the range [0, size()],
     * otherwise, operator throws a 'std::out_of_range' exception.
     */
    double operator[](const std::size_t index) const;

 protected:
    Eigen::VectorXd coordinates_;
};

/**
 * @ingroup group_joint_types
 * @brief Creates display string of the parameter 'vector' and writes it
 * to the parameter 'os'.
*/
std::ostream& operator<<(std::ostream& os, const VectorXd& vector);

}  // namespace crf::utility::types
