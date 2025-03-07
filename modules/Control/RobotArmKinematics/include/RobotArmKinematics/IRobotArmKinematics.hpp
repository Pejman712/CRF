/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <boost/optional.hpp>
#include <memory>
#include <vector>

#include "Types/Types.hpp"

namespace crf::control::robotarmkinematics {

class IRobotArmKinematics {
 public:
    virtual ~IRobotArmKinematics() = default;

    virtual boost::optional<crf::utility::types::TaskPose> getPositionForwardKinematic(
        const crf::utility::types::JointPositions&) = 0;
    virtual boost::optional<crf::utility::types::TaskVelocity> getVelocityForwardKinematic(
        const crf::utility::types::JointPositions&,
        const crf::utility::types::JointVelocities&) = 0;
    virtual std::vector<crf::utility::types::JointPositions> getPositionInverseKinematic(
        const crf::utility::types::TaskPose&,
        const crf::utility::types::JointPositions&) = 0;
    virtual boost::optional<crf::utility::types::JointVelocities> getVelocityInverseKinematic(
        const crf::utility::types::TaskVelocity&,
        const crf::utility::types::JointPositions&) = 0;
    virtual boost::optional<float> getManipulability(
        const crf::utility::types::JointPositions&) = 0;
};

}  // namespace crf::control::robotarmkinematics
