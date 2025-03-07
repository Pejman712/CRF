/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <boost/optional.hpp>
#include <memory>

#include "CANOpenDevices/CANOpenMotors/ICANOpenMotor.hpp"
#include "LinearStage/ILinearStage.hpp"
#include "LinearStage/LinearStageConfiguration.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::linearstage {

/**
 * @ingroup group_can_open_linear_stage
 * @brief TODO
 */
class CANOpenLinearStage : public ILinearStage {
 public:
    CANOpenLinearStage() = delete;
    CANOpenLinearStage(std::shared_ptr<devices::canopendevices::ICANOpenMotor> motor,
        std::shared_ptr<LinearStageConfiguration> configuration);
    CANOpenLinearStage(const CANOpenLinearStage&) = delete;
    CANOpenLinearStage(CANOpenLinearStage&&) = delete;
    ~CANOpenLinearStage() override;

    bool initialize() override;
    bool deinitialize() override;

    bool setTargetPosition(float position) override;
    bool setTargetVelocity(float velocity) override;

    boost::optional<float> getActualPosition() override;
    boost::optional<float> getActualVelocity() override;

    std::shared_ptr<LinearStageConfiguration> getConfiguration() override;

 private:
    utility::logger::EventLogger logger_;

    std::shared_ptr<devices::canopendevices::ICANOpenMotor> motor_;
    std::shared_ptr<LinearStageConfiguration> configuration_;

    bool initialized_;
};

}  // namespace crf::actuators::linearstage
