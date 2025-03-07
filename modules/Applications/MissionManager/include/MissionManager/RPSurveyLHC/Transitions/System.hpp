/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/
#pragma once

#include <memory>

#include "MissionManager/RPSurveyLHC/Guards/Guards.hpp"
#include "MissionManager/RPSurveyLHC/Actions/Actions.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::applications::missionmanager::rpsurveylhc {

class System {
 public:
    explicit System(std::shared_ptr<Actions> actions) noexcept:
        actions_(actions) {}
    ~System() {}

    auto operator()() const noexcept {
        using namespace boost::sml;  // NOLINT
        return make_transition_table(
            // RP Survey State Machine
            *"Stop"_s + event<Start>[([this] {return actions_->initialize();})] = "Initialized"_s,
            "Initialized"_s + event<Next>[([this] {return actions_->goToStart();})] = "InStart"_s,
            "InStart"_s + event<Next>[([this] {return actions_->goToEnd();})] = "InEnd"_s,
            "Initialized"_s + event<Resume>[([this] {return actions_->goToStart();})] = "InStart"_s,
            "InStart"_s + event<Resume>[([this] {return actions_->goToEnd();})] = "InEnd"_s,
            "Initialized"_s + event<Stop>[([this] {return actions_->stop();})] = "Stop"_s,
            "InStart"_s + event<Stop>[([this] {return actions_->stop();})] = "Stop"_s,
            "InEnd"_s + event<Stop>[([this] {return actions_->stop();})] = "Stop"_s,
            "Stop"_s + event<GoHome>/[this] {actions_->goHome();},
            "Initialized"_s + event<Pause>/[this] {actions_->pause();},
            "InStart"_s + event<Pause>/[this] {actions_->pause();},

            // User Input State Machine
            *"Idle"_s + event<SetStatus>[DeployGuard && [this] {return actions_->deploy();}],
            "Idle"_s + event<SetStatus>[RetractGuard && [this] {return actions_->retract();}],

            // Emergency State Machine
            *"Working"_s + event<Emergency>/[this] {actions_->emergencyStop(); });
    }

 private:
    std::shared_ptr<Actions> actions_;
};

}  // namespace crf::applications::missionmanager::rpsurveylhc
