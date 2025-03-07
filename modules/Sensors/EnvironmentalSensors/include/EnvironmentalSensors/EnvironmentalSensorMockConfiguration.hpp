/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/
#pragma once

#include <memory>
#include <optional>
#include <random>

#include <gtest/gtest.h>

#include "EnvironmentalSensors/EnvironmentalSensorMock.hpp"

namespace crf {
namespace sensors {
namespace environmentalsensors {

using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::_;

class EnvironmentalSensorMockConfiguration {
 public:
    EnvironmentalSensorMockConfiguration():
        initialized_(false),
        maxValue_(0),
        minValue_(0) {
            environmentalSensorMock_.reset(new NiceMock<EnvironmentalSensorMock>());
        }

    ~EnvironmentalSensorMockConfiguration() {
    }

    std::shared_ptr<NiceMock<EnvironmentalSensorMock>> getMock() {
        return environmentalSensorMock_;
    }

    void setMeasurementRange(float maxValue, float minValue) {
        maxValue_ = maxValue;
        minValue_ = minValue;
    }

    void configureEnvironmentalSensorMock() {
        ON_CALL(*environmentalSensorMock_, initialize()).WillByDefault(Invoke(
            [this] {
                if (initialized_) return false;
                initialized_ = true;
                return true;
            }));
        ON_CALL(*environmentalSensorMock_, deinitialize()).WillByDefault(Invoke(
            [this] {
                if (!initialized_) return false;
                initialized_ = false;
                return true;
            }));

        ON_CALL(*environmentalSensorMock_, getMeasurement()).WillByDefault(Invoke(
            [this] () -> std::optional<float> {
                if (!initialized_) return std::nullopt;
                std::random_device ran;
                std::mt19937 mt(ran());
                if (maxValue_ == 0.0 && minValue_ == 0.0) {
                    std::uniform_real_distribution<float> dist(-1, 1);
                    return dist(mt);
                }
                std::uniform_real_distribution<float> dist(minValue_, maxValue_);
                return dist(mt);
            }));
    }

 private:
    bool initialized_;
    std::shared_ptr<NiceMock<EnvironmentalSensorMock>> environmentalSensorMock_;

    float maxValue_;
    float minValue_;
};

}  // namespace environmentalsensors
}  // namespace sensors
}  // namespace crf
