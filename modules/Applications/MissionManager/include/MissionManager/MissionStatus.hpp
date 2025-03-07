/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

namespace crf::applications::missionmanager {

enum class MissionStatus {
    NotDefined = 0,
    /**
     * @brief Mission is stopped.
     */
    Stopped = 1,
    /**
     * @brief Mission is started.
     */
    Started = 2,
    /**
     * @brief Mission is paused.
     */
    Paused = 3
};

}  // namespace crf::applications::missionmanager
