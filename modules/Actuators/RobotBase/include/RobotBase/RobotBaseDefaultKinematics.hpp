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

#include "RobotBase/IRobotBaseKinematics.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"

namespace crf::actuators::robotbase {

class RobotBaseDefaultKinematics : public IRobotBaseKinematics {
 public:
    RobotBaseDefaultKinematics() = delete;
    explicit RobotBaseDefaultKinematics(const RobotBaseConfiguration& configuration);
    ~RobotBaseDefaultKinematics() override = default;

    boost::optional<crf::utility::types::TaskVelocity>
        getTaskVelocity(const std::vector<float>& wheelsVelocity) override;
    boost::optional<std::vector<float>>
        getWheelsVelocity(const crf::utility::types::TaskVelocity&) override;

 private:
    float lx_;
    float ly_;
    float wheelsRadius_;
};

}  // namespace crf::actuators::robotbase
