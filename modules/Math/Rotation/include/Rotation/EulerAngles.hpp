/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#pragma once

#include <array>
#include <iomanip>
#include <iostream>
#include <vector>

namespace crf::math::rotation {

/**
 * @ingroup group_rotation_euler_angles
 * @brief Base class for different Euler angles representations of rotation.
 */
class EulerAngles {
 public:
    /**
     * @brief Constructs a new EulerAngles object with [0.0, 0.0, 0.0] angles.
    */
    EulerAngles();

    EulerAngles(const EulerAngles&) = default;

    explicit EulerAngles(const std::array<double, 3>& coordinates);

    explicit EulerAngles(const std::initializer_list<double>& coordinates);

    ~EulerAngles() = default;

    EulerAngles& operator=(const EulerAngles& other) = default;

    EulerAngles& operator=(const std::array<double, 3>& coordinates);

    EulerAngles& operator=(const std::initializer_list<double>& coordinates);

    /**
     * @brief Non-const version of the index operator.
     * @throw Range -- parameter 'index' is checked if it is within the range [0, 3], otherwise,
     * operator throws a 'std::out_of_range' exception.
    */
    double& operator[](const std::size_t index);

    /**
     * @brief Non-const version of the index operator.
     * @throw Range -- parameter 'index' is checked if it is within the range [0, 3], otherwise,
     * operator throws a 'std::out_of_range' exception.
    */
    double operator[](const std::size_t index) const;

    /**
    * @brief Returns number of coordinates. Always equal to 3.
    */
    std::size_t size() const;

    /**
     * @brief Returns std::array<double, 3> of angles from EulerAngles class.
     * Useful when serialising.
    */
    std::array<double, 3> rawArray() const;

 protected:
    std::array<double, 3> angles_;
};

/**
 * @ingroup group_rotation_euler_angles
 * @brief Creates display string of the parameter 'eulerAngles' and writes it
 * to the parameter 'os'.
*/
std::ostream& operator<<(std::ostream& os, const EulerAngles& eulerAngles);

/**
 * @ingroup group_rotation_euler_angles
 * @brief Class for XYZ Cardan angles.
 */
class CardanXYZ : public EulerAngles {
    using EulerAngles::EulerAngles;

 public:
    using EulerAngles::operator=;
};

/**
 * @ingroup group_rotation_euler_angles
 * @brief Creates display string of the parameter 'cardanXYZ' and writes it
 * to the parameter 'os'.
*/
std::ostream& operator<<(std::ostream& os, const CardanXYZ& cardanXYZ);

/**
 * @ingroup group_rotation_euler_angles
 * @brief Class for ZXZ Euler angles.
 */
class EulerZXZ : public EulerAngles {
    using EulerAngles::EulerAngles;

 public:
    using EulerAngles::operator=;
};

/**
 * @ingroup group_rotation_euler_angles
 * @brief Creates display string of the parameter 'eulerZXZ' and writes it
 * to the parameter 'os'.
*/
std::ostream& operator<<(std::ostream& os, const EulerZXZ& eulerZXZ);

}  // namespace crf::math::rotation
