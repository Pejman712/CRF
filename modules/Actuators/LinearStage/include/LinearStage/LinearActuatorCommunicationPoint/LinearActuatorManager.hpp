/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>

#include <nlohmann/json.hpp>
#include "CommunicationUtility/ExpectedJSONConverter.hpp"

#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "LinearStage/ILinearActuator.hpp"

namespace crf::actuators::linearactuator {

/**
 * @ingroup group_linear_actuator_communication_point
 * @brief
 */
class LinearActuatorManager: public crf::utility::devicemanager::DeviceManagerWithPriorityAccess {  // NOLINT
 public:
    LinearActuatorManager() = delete;
    LinearActuatorManager(std::shared_ptr<crf::actuators::linearactuator::ILinearActuator> actuator,  // NOLINT
        const std::chrono::milliseconds& inizializationTimeout  = std::chrono::seconds(60),
        const std::chrono::milliseconds& controlAccessTimeout  = std::chrono::seconds(10));
    LinearActuatorManager(const LinearActuatorManager& other) = delete;
    LinearActuatorManager(LinearActuatorManager&& other) = delete;
    ~LinearActuatorManager();

    crf::expected<bool> setPosition(const uint32_t& priority, const double& position);
    crf::expected<bool> setVelocity(const uint32_t& priority, const double& velocity);

    nlohmann::json getStatus();

 private:
    std::shared_ptr<crf::actuators::linearactuator::ILinearActuator> actuator_;
};

}  // namespace crf::actuators::linearactuator
