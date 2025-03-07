/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN EN/SMM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

namespace crf::control::motioncontroller {

/**
 * @ingroup group_motion_controller
 * @brief Enumeration to define the possible types of trajectory execution
 *
 */
enum class TrajectoryExecutionMethod {
    /**
     * @brief Enumeration placeholder for a value that is not defined
     *
     */
    NotDefined = 0,

    /**
     * @brief Enumeration to signify that the provided path should be followed in
     * Joint space (aka: non linear movement)
     */
    JointSpace = 1,

    /**
     * @brief Enumeration to signify that the provided path should be followed in
     * Task space (aka: linear trajectory)
     */
    TaskSpace = 2,
};

}  // namespace crf::control::motioncontroller
