/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>

#include "Types/Types.hpp"

namespace crf::control::trajectorygeneratordeprecated {

struct JointsTrajectoryData {
    std::vector<float> time;
    std::vector<utility::types::JointPositions> position;
    std::vector<utility::types::JointVelocities> velocity;
    std::vector<utility::types::JointAccelerations> acceleration;
    std::vector<crf::utility::types::JointForceTorques> torque;
};

struct TaskTrajectoryData {
    std::vector<float> time;
    std::vector<utility::types::TaskPose> position;
    std::vector<utility::types::TaskVelocity> velocity;
};

}  // namespace crf::control::trajectorygeneratordeprecated
