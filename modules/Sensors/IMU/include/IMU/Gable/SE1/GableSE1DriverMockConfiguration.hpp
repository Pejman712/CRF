/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <vector>
#include <memory>

#include "SOEMAPI/SOEMAPIMockConfiguration.hpp"
#include "EventLogger/EventLogger.hpp"
#include "IMU/Gable/GableTxPDO.hpp"
#include "IMU/Gable/SE1/GableSE1RxPDO.hpp"

namespace crf::sensors::imu {

class GableSE1DriverMockConfiguration :
    public crf::communication::soemapi::SOEMAPIMockConfiguration {
 public:
    GableSE1DriverMockConfiguration() = delete;
    GableSE1DriverMockConfiguration(const uint8_t& numberOfSlaves, const int& ioMapSize):
        SOEMAPIMockConfiguration(numberOfSlaves, ioMapSize),
        logger_("GableSE1DriverMockConfiguration") {
            gableInputs_.resize(numberOfSlaves_);
            gableOutputs_.resize(numberOfSlaves_);
    }

    void configure() override {
        SOEMAPIMockConfiguration::configure();

        ON_CALL(*this, config_overlap_map(_)).WillByDefault(Invoke(
            [this](void* pIOmap) {
                // Map slaves to their respective inputs and outputs
                for (int i = 0; i < numberOfSlaves_; i++) {
                    slaves_[i + 1].inputs = reinterpret_cast<uint8*>(&gableInputs_[i]);
                    slaves_[i + 1].outputs = reinterpret_cast<uint8*>(&gableOutputs_[i]);
                }

                if (stateForced_) return ioMapSize_;
                // Move slaves to SAFE-OPERATIONAL
                for (int i = 0; i < numberOfSlaves_ + 1; i++) {
                    slaves_[i].state = EC_STATE_SAFE_OP;
                }
                return ioMapSize_;
            }));

        ON_CALL(*this, send_overlap_processdata()).WillByDefault(Invoke([this](){
            for (int i = 0; i < numberOfSlaves_; i++) {
                gableInputs_[i].gx = static_cast<float32>(1.8);
                gableInputs_[i].gy = static_cast<float32>(0.12);
                gableInputs_[i].gz = static_cast<float32>(2.5);
                gableInputs_[i].ax = static_cast<float32>(0.23);
                gableInputs_[i].ay = static_cast<float32>(1.15);
                gableInputs_[i].az = static_cast<float32>(0.68);
                gableInputs_[i].axHR = static_cast<float32>(0.23);
                gableInputs_[i].ayHR = static_cast<float32>(1.15);
                gableInputs_[i].azHR = static_cast<float32>(0.68);
                gableInputs_[i].gxHR = static_cast<float32>(1.8);
                gableInputs_[i].gyHR = static_cast<float32>(0.12);
                gableInputs_[i].gzHR = static_cast<float32>(2.5);
                gableInputs_[i].magX = static_cast<float32>(0.53);
                gableInputs_[i].magY = static_cast<float32>(0.27);
                gableInputs_[i].magZ = static_cast<float32>(1.3);
                gableInputs_[i].CRC_Errors = static_cast<uint32_t>(1);
                gableInputs_[i].WakeUp = static_cast<uint8_t>(1);
                gableInputs_[i].CommandResponse = static_cast<uint8_t>(1);
                gableInputs_[i].CommunicationErrors = static_cast<uint32_t>(1);
                gableInputs_[i].PacketCounter = static_cast<uint16_t>(1);
                gableInputs_[i].Baudrate = static_cast<uint8_t>(3);
                gableInputs_[i].Firmware = static_cast<int8_t>(65);
                gableInputs_[i].Hardware = static_cast<int8_t>(94);
                gableInputs_[i].DeviceID = static_cast<uint32_t>(648);
            }
            return true;
        }));
    }

 protected:
    std::vector<GableRxPDO_SE1> gableInputs_;
    std::vector<GableTxPDO> gableOutputs_;
    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::sensors::imu
