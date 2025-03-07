/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Ante Marić CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

namespace crf::math::kinematicchain {

/**
 * @ingroup group_kinematic_chain_description_types
 * @brief
 *
 */
enum class DescriptionType {
    NotDefined = 0,
    /**
     * @brief The Tool description type describes the geometric properties  of the end-effector or
     * sensor system using fixed joints
     */
    Tool = 1,
    /**
     * @brief The Platform description type describes the geometric properties  of a mobile platform
     * with 4 omniwheels
     */
    Platform = 2,
    /**
     * @brief The Arm description type describes the geometric properties of a robotic arm or
     * manipulator chain
     */
    Arm = 3,
    /**
     * @brief The Combined description type describes the geometric properties of a robotic system
     * consisting of a mobile platform and one or multiple robotic arms (manipulators)
     */
    Combined = 4
};

}  // namespace crf::math::kinematicchain
