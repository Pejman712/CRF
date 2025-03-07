/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/

#pragma once

namespace crf::applications::missionmanager::rpsurveylhc {

struct Start{};
struct Pause{};
struct Resume{};
struct GoHome{};
struct Recovery{};
struct Restart{};
struct Rearm{};
struct Emergency{};
struct Recharge{};
struct Stop{};
struct Reset{};
struct Next{};
struct SetStatus{
    bool Retract = false;
    bool Deploy = false;
};

}  // namespace crf::applications::missionmanager::rpsurveylhc
