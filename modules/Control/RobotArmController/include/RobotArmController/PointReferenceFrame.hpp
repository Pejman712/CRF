/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

namespace crf::control::robotarmcontroller {

enum class PointReferenceFrame {
    NotDefined = 0,
    /*
     * @brief 
     */
    Global = 1,
    /*
     * @brief 
     */
    TCP = 2,
};

}  // namespace crf::control::robotarmcontroller
