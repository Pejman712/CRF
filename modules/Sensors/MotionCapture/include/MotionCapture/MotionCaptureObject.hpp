/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 * ====================================================================
*/

#pragma once

#include <string>
#include <vector>
#include "Types/TaskTypes/TaskPose.hpp"
#include <MotionCapture/MotionCaptureMarker.hpp>

namespace crf::sensors::motioncapture {

/**
 * @ingroup group_motion_capture
 * @brief This struct contains the information streamed by a generic Motion Capture system
 *        for a generic rigid object. The information are: the name of the rigid object as a string,
 *        its pose with respect to a world frame as TaskPosition object (the orientation is in quaternion),
 *        and finally a vector containing the infos on the markers defining the object.
 */
struct MotionCaptureObject {
    std::string objectName;
    crf::utility::types::TaskPose objectPose;
    std::vector<MotionCaptureMarker> objectMarkers;
};

}  // namespace crf::sensors::motioncapture
