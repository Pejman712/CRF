/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
*/
#pragma once

#include <memory>

#include "MissionManager/ScienceGateway/Guards/Guards.hpp"
#include "MissionManager/ScienceGateway/Actions/Actions.hpp"
#include "MissionManager/ScienceGateway/Transitions/EasyMission.hpp"
#include "MissionManager/ScienceGateway/Transitions/MediumMission.hpp"
#include "MissionManager/ScienceGateway/Transitions/HardMission.hpp"

#include "EventLogger/EventLogger.hpp"

namespace crf::applications::missionmanager::sciencegateway {

/**
 * @brief Full state machine description, it include the selction and transitions into
 * smaller state machines
 *
 */
class System {
 public:
    explicit System(std::shared_ptr<Actions> actions) noexcept:
        actions_(actions) {}
    ~System() {}

    auto operator()() const noexcept {
        using namespace boost::sml;  // NOLINT
        return make_transition_table(
            // Sciencegateway State Machine
            *"Stop"_s + event<Start>[([this] {return actions_->initialize();})] = "Initialized"_s,

            // Mission Selection, User Input State Machine
            "Initialized"_s + event<SetMission> [EasyMissionCheck] = state<EasyMission>,
            "Initialized"_s + event<SetMission> [MediumMissionCheck] = state<MediumMission>,
            "Initialized"_s + event<SetMission> [HardMissionCheck] = state<HardMission>,

            // Go back to main selection
            state<EasyMission> + event<GoHome> [(
                [this] {return actions_->retract();})] = "Initialized"_s,
            state<MediumMission> + event<GoHome> [(
                [this] {return actions_->retract();})] = "Initialized"_s,
            state<HardMission> + event<GoHome> [(
                [this] {return actions_->retract();})] = "Initialized"_s,

            // Stop commands
            "Initialized"_s + event<Stop> [([this] {return actions_->deinitialize();})] = "Stop"_s,
            state<EasyMission> + event<Stop> [(
                [this] {return actions_->retract() && actions_->deinitialize();})] = "Stop"_s,
            state<MediumMission> + event<Stop> [(
                [this] {return actions_->retract() && actions_->deinitialize();})] = "Stop"_s,
            state<HardMission> + event<Stop> [(
                [this] {return actions_->retract() && actions_->deinitialize();})] = "Stop"_s);
    }

 private:
    std::shared_ptr<Actions> actions_;
};

}  // namespace crf::applications::missionmanager::sciencegateway
