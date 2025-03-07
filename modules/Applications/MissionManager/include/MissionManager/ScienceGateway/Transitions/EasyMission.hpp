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
#include "EventLogger/EventLogger.hpp"

namespace crf::applications::missionmanager::sciencegateway {

/**
 * @brief Easy mission state machine description
 *
 */
class EasyMission {
 public:
    explicit EasyMission(std::shared_ptr<Actions> actions) noexcept:
        actions_(actions) {}
    ~EasyMission() {}

    auto operator()() const noexcept {
        using namespace boost::sml;  // NOLINT
        return make_transition_table(
            // First mission: Viewing the surroundings
            *"Easymission"_s + event<Next> [([this] {return actions_->deploy();})] = "Deployed"_s);
    }

 private:
    std::shared_ptr<Actions> actions_;
};

}  // namespace crf::applications::missionmanager::sciencegateway
