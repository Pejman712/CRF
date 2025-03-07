/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include "Types/JointTypes/VectorXd.hpp"

namespace crf::utility::types {

/**
 * @ingroup group_joint_types
 * @brief Class for representing and strongly typing joint accelerations.
 */
class JointAccelerations : public VectorXd {
    using VectorXd::VectorXd;

 public:
    using VectorXd::operator=;
};

}  // namespace crf::utility::types
