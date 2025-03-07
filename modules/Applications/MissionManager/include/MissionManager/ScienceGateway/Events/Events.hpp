/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
*/

#pragma once

namespace crf::applications::missionmanager::sciencegateway {

/**
 * @brief Enumeration of all the types of missions available
 *
 */
enum class MissionSelect {
    /**
     * @brief No mission has been selected
     *
     */
    NotSet = 0,

    /**
     * @brief Easy difficulty mission selected
     *
     */
    Easy = 1,

    /**
     * @brief Medium difficulty mission selected
     *
     */
    Medium = 2,

    /**
     * @brief Hard difficulty mission selected
     *
     */
    Hard = 3
};

struct Start{};
struct SetMission{
    MissionSelect mission = MissionSelect::NotSet;
};
struct Stop{};
struct Next{};
struct GoHome{};
struct SetStatus{
    bool Retract = false;
    bool Deploy = false;
};

}  // namespace crf::applications::missionmanager::sciencegateway
