/*
 * © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Julia Kabalar EN/SMM/MRO 2018
 *          Alejandro Diaz Rosales BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include "IMU/IIMU.hpp"
#include "crf/expected.hpp"

using testing::_;
using testing::Invoke;

namespace crf::sensors::imu {

class IMUMock : public IIMU{
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));
    MOCK_METHOD(crf::expected<bool>, calibrate, (), (override));
    MOCK_METHOD(IMUSignals, getSignal, (), (override));
};

}  // namespace crf::sensors::imu
