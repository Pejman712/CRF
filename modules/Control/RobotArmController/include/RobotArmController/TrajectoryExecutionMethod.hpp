/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

namespace crf::control::robotarmcontroller {

enum class TrajectoryExecutionMethod {
    NotDefined = 0,
    /*
     * @brief 
     */
    JointSpace = 1,
    /*
     * @brief 
     */
    TaskSpaceJointLoop = 2,
    /*
     * @brief 
     */
    TaskSpaceTaskLoop = 3,
};

}  // namespace crf::control::robotarmcontroller
