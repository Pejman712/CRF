/*
 * © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Julia Kabalar EN/SMM/MRO 2018
 *          Alejandro Diaz Rosales BE/CEM/MRO 2023
 *          Sebastien Collomb BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */
#pragma once

#include <array>

#include "CommonInterfaces/IInitializable.hpp"
#include "crf/expected.hpp"

namespace crf::sensors::imu {

    /**
     * @ingroup group_imu
     * @brief This struct contains expected values for different IMU signals, such as position,
     *        orientation, linear velocity, angular velocity, linear acceleration, angular
     *        acceleration, and magnetic field. Each signal is represented as an array of three or four
     *        double values. These values are encapsulated in a crf::expected object indicating whether
     *        the data is available or not.
     *
     *        Usage example:
     *        @code
     *        IMUSignals imuData;
     *        crf::expected<std::array<double, 3>> position = imuData.position;
     *        if (position) {
     *            // Access the position data
     *            std::array<double, 3> pos = position.value();
     *        } else {
     *            // Handle the case where position data is not available
     *            crf::Code error = position.error();
     *        }
     *        @endcode
     */
struct IMUSignals{
    /**
     * @brief This field represents the position data as an array of three double values.
     *
     */
    crf::expected<std::array<double, 3>> position = crf::Code::NotImplemented;
    /**
     * @brief This field represents the orientation data as an array of four double values.
     *
     */
    crf::expected<std::array<double, 4>> quaternion = crf::Code::NotImplemented;
    /**
     * @brief This field represents the orientation as an array of three double values
     *        with a Euler angles (ZYX) representation.
     *
     */
    crf::expected<std::array<double, 3>> eulerZYX = crf::Code::NotImplemented;
    /**
     * @brief This field represents the linear velocity data as an array of three double values.
     *
     */
    crf::expected<std::array<double, 3>> linearVelocity = crf::Code::NotImplemented;
    /**
     * @brief This field represents the angular velocity data as an array of three double values.
     *
     */
    crf::expected<std::array<double, 3>> angularVelocity = crf::Code::NotImplemented;
    /**
     * @brief This field represents the linear acceleration data as an array of three double
     *        values.
     *
     */
    crf::expected<std::array<double, 3>> linearAcceleration = crf::Code::NotImplemented;
    /**
     * @brief This field represents the angular acceleration data as an array of three double
     *        values.
     *
     */
    crf::expected<std::array<double, 3>> angularAcceleration = crf::Code::NotImplemented;
    /**
     * @brief This field represents the magnetic field data as an array of three double
     *        values.
     *
     */
    crf::expected<std::array<double, 3>> magneticField = crf::Code::NotImplemented;
};

}  // namespace crf::sensors::imu
