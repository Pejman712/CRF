/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Zsolt Pasztori CERN EN/SMM/MRO
 *         Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#pragma once

#include <iomanip>
#include <iostream>
#include <array>
#include <Eigen/Dense>

namespace crf::utility::types {

/**
 * @ingroup group_task_types
 * @brief Composition from Eigen::Vectr6d for strongly typing vectors of length 6.
 * Base class for types:
 *     - TaskVelocity
 *     - TaskAcceleration
 *     - TaskForceTorque.
 */
class Vector6d {
 public:
    /**
     * @brief Construct a new Vector6d object with all six coordinates equal to 0.0.
     */
    Vector6d();

    Vector6d(const Vector6d&) = default;

    explicit Vector6d(const Eigen::Vector<double, 6>& coordinates);

    explicit Vector6d(const std::array<double, 6>& coordinates);

    /**
     * @throws Size -- Parameter 'coordinates' is checked if it has the size equal to 6, otherwise,
     * constructor throws a 'std::invalid_argument' exception.
     */
    explicit Vector6d(const std::initializer_list<double>& coordinates);

    ~Vector6d() = default;

    Vector6d& operator=(const Vector6d& other) = default;

    Vector6d& operator=(const Eigen::Vector<double, 6>& coordinates);

    Vector6d& operator=(const std::array<double, 6>& coordinates);

    /**
     * @throws Size -- Parameter 'coordinates' is checked if it is of the same size equal to 6,
     * otherwise, operator throws a 'std::invalid_argument' exception.
     */
    Vector6d& operator=(const std::initializer_list<double>& coordinates);

    /**
    * @brief Returns number of coordinates. Always equal to 6.
    */
    std::size_t size() const;

    /**
     * @brief Vector6d does not support direct arithmetical operations.
     * This function returns correponding Eigen::Vector<double, 6> object,
     * on which such operations can be performed.
     */
    Eigen::Vector<double, 6> raw() const;

    /**
     * @brief Non-Const version of the index operator.
     * @throw Range -- parameter 'index' is checked if it is within the range [0, 6], otherwise,
     * operator throws a 'std::out_of_range' exception.
     */
    double& operator[](const std::size_t index);

    /**
     * @brief Const version of the index operator.
     * @throw Range -- parameter 'index' is checked if it is within the range [0, 6], otherwise,
     * operator throws a 'std::out_of_range' exception.
     */
    double operator[](const std::size_t index) const;

 protected:
    Eigen::Vector<double, 6> coordinates_;
};

/**
 * @brief Printable version of Vector6d
 */
std::ostream& operator<<(std::ostream& os, const Vector6d& vector6d);

}  // namespace crf::utility::types
