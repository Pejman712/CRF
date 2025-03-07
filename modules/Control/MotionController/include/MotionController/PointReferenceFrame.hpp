/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

namespace crf::control::motioncontroller {

/**
 * @ingroup group_motion_controller
 * @brief Enumeration to define the posible reference frames the robot can move through
 *
 */
enum class PointReferenceFrame {
    /**
     * @brief Enumeration placeholder for a value that is not defined
     *
     */
    NotDefined = 0,

    /**
     * @brief Enumeration to signify the reference frame used for the
     * provided points. Global expresses that the frame used is at the base
     * of the robot
     */
    Global = 1,

    /**
     * @brief Enumeration to signify the reference frame used for the
     * provided points. TCP expresses that the frame used is at tip of the
     * robot. (Every time the robot moves this reference frame changes too)
     */
    TCP = 2,
};

}  // namespace crf::control::motioncontroller
