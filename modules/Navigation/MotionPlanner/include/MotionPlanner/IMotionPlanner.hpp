/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */
#pragma once

#include "Types/TaskTypes/TaskPose.hpp"

#include "CommonInterfaces/IInitializable.hpp"

namespace crf {
namespace applications {
namespace motionplanner {

class IMotionPlanner : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IMotionPlanner() = default;
    // (to be checked) - defined from user / gui / whatever kind of input
    virtual bool plan(const utility::types::TaskPose &goal) = 0;
};

}  // namespace motionplanner
}  // namespace applications
}  // namespace crf
