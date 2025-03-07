/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
*/
#pragma once

#include <memory>

#include <boost/sml.hpp>

#include "MissionManager/ScienceGateway/Events/Events.hpp"

namespace crf::applications::missionmanager::sciencegateway {

const auto EasyMissionCheck = [](const auto& event) {
    return event.mission == MissionSelect::Easy;
};

const auto MediumMissionCheck = [](const auto& event) {
    return event.mission == MissionSelect::Medium;
};

const auto HardMissionCheck = [](const auto& event) {
    return event.mission == MissionSelect::Hard;
};

}  // namespace crf::applications::missionmanager::sciencegateway
