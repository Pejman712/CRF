/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "LinearStage/ILinearActuator.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::linearactuator {

class EtherCATLinearActuator : public ILinearActuator {
 public:
    explicit EtherCATLinearActuator(
        std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> motor);
    ~EtherCATLinearActuator() override;

    bool initialize() override;
    bool deinitialize() override;

    crf::expected<bool> setPosition(const double& position) override;
    crf::expected<bool> setVelocity(const double& velocity) override;

    crf::expected<double> getPosition() const override;
    crf::expected<double> getVelocity() const override;

 private:
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> motor_;

    bool initialized_;

    crf::utility::logger::EventLogger logger_;

    // Encoder 4096 cnt/rev, gearbox ratio 1:33
    const double velocity_ = 563200;  // (563200 cnt/s * 360 deg/rev) / (4096 cnt/rev * 33) = 1500 deg/s  NOLINT
    const double acceleration_ = 563200;  // 1500 deg/s^2
    const double deceleration_ = 563200;  // 1500 deg/s^2
};

}  // namespace crf::actuators::linearactuator
