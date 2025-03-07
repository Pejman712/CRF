/* © Copyright CERN 2023.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#pragma once

#include <iostream>
#include <cassert>
#include <string>
#include <memory>
#include <array>
#include <thread>
#include <pthread.h>
#include <utility>
#include <unistd.h>
#include <vector>
#include <queue>

#include <nlohmann/json.hpp>

#include "MissionManager/IMissionManager.hpp"
#include "MissionManager/ScienceGateway/Transitions/System.hpp"
#include "MissionManager/MissionStatus.hpp"

namespace crf::applications::missionmanager::sciencegateway {

class ScienceGateway: public IMissionManager {
 public:
    ScienceGateway() = delete;
    explicit ScienceGateway(
        std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable,
        const nlohmann::json& configFile);

    ~ScienceGateway() override;

    bool start() override;
    bool next() override;
    bool stop() override;
    bool pause() override;
    bool resume() override;
    bool goHome() override;
    bool recharge() override;
    nlohmann::json getStatus() override;
    bool setStatus(const nlohmann::json& status) override;
    bool emergency() override;
    bool rearm() override;

 private:
    std::shared_ptr<Actions> actionsPtr_;
    boost::sml::sm<System> sMachine_;

    MissionStatus missionStatus_;
    SetMission setMission_;

    crf::utility::logger::EventLogger logger_;

    void clearGuards();
};
}  // namespace crf::applications::missionmanager::sciencegateway
