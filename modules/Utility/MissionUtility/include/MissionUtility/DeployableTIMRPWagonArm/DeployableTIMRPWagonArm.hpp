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

#include "MissionUtility/IDeployableDevice.hpp"
#include "TIMRPWagon/ITIMRPWagon.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::utility::missionutility {

/**
 * @brief Implementation of IDeployablefor the TIM RP Wagon Arm to control the basic movements.
 *
 */
class DeployableTIMRPWagonArm: public IDeployableDevice {
 public:
    DeployableTIMRPWagonArm() = delete;
    explicit DeployableTIMRPWagonArm(
        std::shared_ptr<crf::actuators::timrpwagon::ITIMRPWagon> timRPWagon);
    ~DeployableTIMRPWagonArm();

    bool initialize() override;
    bool deinitialize() override;

    bool deploy() override;
    bool retract() override;
    bool stop() override;
    bool isRetracted() override;
    bool isDeployed() override;
    bool isMoving() override;

 private:
    std::shared_ptr<crf::actuators::timrpwagon::ITIMRPWagon> timRPWagon_;
    crf::utility::logger::EventLogger logger_;
    std::atomic<bool> movingArm_;
    std::mutex mtx_;
};

}  // namespace crf::utility::missionutility
