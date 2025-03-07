/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/
#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "RobotArmController/IRobotArmController.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Types/JsonConverters.hpp"
#include "MissionUtility/IDeployableDevice.hpp"

namespace crf::utility::missionutility {

/**
 * @brief Implementation of IDeployablefor the TIM Robot Arm to control the basic movements.
 *
 */
class DeployableRobotArm: public IDeployableDevice {
 public:
    DeployableRobotArm() = delete;
    DeployableRobotArm(
        std::shared_ptr<crf::control::robotarmcontroller::IRobotArmController> armController,
        const nlohmann::json& configFile);
    ~DeployableRobotArm();

    bool initialize() override;
    bool deinitialize() override;

    bool deploy() override;
    bool retract() override;
    bool stop() override;
    bool isRetracted() override;
    bool isDeployed() override;
    bool isMoving() override;

 private:
    std::shared_ptr<crf::control::robotarmcontroller::IRobotArmController> armController_;
    nlohmann::json trajectoryJson_;
    crf::utility::logger::EventLogger logger_;
    std::atomic<bool> movingArm_;
    float bigMargin_;
    std::mutex mtx_;

    bool moveArm(const types::JointPositions &position,
        const crf::utility::types::JointVelocities &velocity);
    bool equalMinusLastJoint(const crf::utility::types::JointPositions& pos1,
        const crf::utility::types::JointPositions& pos2, float delta);
};

}  // namespace crf::utility::missionutility
