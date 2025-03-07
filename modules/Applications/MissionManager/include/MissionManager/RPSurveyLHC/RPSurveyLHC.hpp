/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
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
#include "MissionManager/MissionStatus.hpp"
#include "MissionManager/RPSurveyLHC/Transitions/System.hpp"

namespace crf::applications::missionmanager::rpsurveylhc {

class RPSurveyLHC: public IMissionManager {
 public:
    RPSurveyLHC() = delete;
    explicit RPSurveyLHC(
        std::shared_ptr<crf::actuators::tim::ITIM> tim,
        std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable,
        std::shared_ptr<crf::sensors::rpsensor::IRPSensor> rpSensor,
        const nlohmann::json& configFile);

    ~RPSurveyLHC() override;

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
    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<crf::applications::missionmanager::rpsurveylhc::Actions> actionsPtr_;
    boost::sml::sm<crf::applications::missionmanager::rpsurveylhc::System> sMachine_;
    MissionStatus missionStatus_;
    std::thread pauseTimer_;
    std::atomic<bool> stopTimer_;
    SetStatus setStatus_;

    void clearGuards();
    void timer();
};

}  // namespace crf::applications::missionmanager::rpsurveylhc
