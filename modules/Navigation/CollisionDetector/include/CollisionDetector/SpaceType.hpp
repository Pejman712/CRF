/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

namespace crf::navigation::collisiondetector {

/* 
 * SpaceTypes: 
 *  - UNKNOWN_STATE_SPACE - all not supported types,
 *  - REALVECTOR_STATE_SPACE - a space to represent R^(dim) within custom bounds,
 *  - SO2_STATE_SPACE - a space to represent continuous (planar) rotation
 */
enum class SpaceType {
    UNKNOWN_STATE_SPACE = 0,
    REALVECTOR_STATE_SPACE = 1,
    SO2_STATE_SPACE = 2
};

}  // namespace crf::navigation::collisiondetector
