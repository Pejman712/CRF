/*
 * © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Julia Kabalar EN/SMM/MRO 2018
 *          Alejandro Diaz Rosales BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <array>

#include "CommonInterfaces/IInitializable.hpp"
#include "crf/expected.hpp"

#include "IMU/IMUSignals.hpp"

namespace crf::sensors::imu {

/**
 * @ingroup group_imu
 * @brief Interface class for all IMU devices.
 *
 */
class IIMU : public crf::utility::commoninterfaces::IInitializable{
 public:
    virtual ~IIMU() = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief Get the different signals of the IMU, although not all them might be avaiable, since some of the
     *         values have to be estimated, or the sensor might not have the appropiate equipment to measure it.
     *
     * @return IMUSignals structure that containes the different values.
     */
    virtual IMUSignals getSignal() = 0;
    /**
     * @brief The sensor must be still, in a flat sureface before starting the calibration.
     *
     * @return true if the calibration was succesfull.
     */
    virtual crf::expected<bool> calibrate() = 0;
};

}  // namespace crf::sensors::imu
