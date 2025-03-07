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
#include <vector>

#include <nlohmann/json.hpp>

#include "MotionController/IMotionController.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Types/JsonConverters.hpp"
#include "MissionUtility/IDeployableDevice.hpp"

namespace crf::utility::missionutility {

/**
 * @brief Implementation of IDeployablefor the KinovaGen3 Robot Arm to control the basic movements.
 *
 */
class DeployableRobot: public IDeployableDevice {
 public:
    DeployableRobot() = delete;
    DeployableRobot(
        std::shared_ptr<crf::control::motioncontroller::IMotionController> armController,
        const nlohmann::json& configFile);
    ~DeployableRobot();

    bool initialize() override;
    bool deinitialize() override;

    bool deploy() override;
    bool retract() override;
    bool stop() override;
    bool isRetracted() override;
    bool isDeployed() override;
    bool isMoving() override;

 private:
    std::shared_ptr<crf::control::motioncontroller::IMotionController> armController_;
    std::vector<crf::utility::types::JointPositions> retract2deploy_;
    std::vector<crf::utility::types::JointPositions> deploy2retract_;
    crf::utility::types::JointPositions retractedPos_;
    crf::utility::types::JointPositions deployedPos_;
    crf::utility::logger::EventLogger logger_;
    std::atomic<bool> movingArm_;
    float bigMargin_;
    std::mutex mtx_;

    bool equalMinusLastJoint(const crf::utility::types::JointPositions& pos1,
        const crf::utility::types::JointPositions& pos2, float delta);
};

}  // namespace crf::utility::missionutility
