#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <boost/optional.hpp>
#include <vector>

#include "Types/TaskTypes/TaskVelocity.hpp"

namespace crf::actuators::robotbase {

class IRobotBaseKinematics {
 public:
    virtual ~IRobotBaseKinematics() = default;
    virtual boost::optional<crf::utility::types::TaskVelocity>
        getTaskVelocity(const std::vector<float>& wheelsVelocity) = 0;
    virtual boost::optional<std::vector<float>>
        getWheelsVelocity(const crf::utility::types::TaskVelocity&) = 0;
};

}  // namespace crf::actuators::robotbase
