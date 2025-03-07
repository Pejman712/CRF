/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/
#pragma once

#include <memory>

#include <boost/sml.hpp>

#include "MissionManager/RPSurveyLHC/Events/Events.hpp"

namespace crf::applications::missionmanager::rpsurveylhc {

const auto DeployGuard = [](const auto& event) {
    return event.Deploy;
};
const auto RetractGuard = [](const auto& event) {
    return event.Retract;
};

}  // namespace crf::applications::missionmanager::rpsurveylhc
