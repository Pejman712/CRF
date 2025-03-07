/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 * ====================================================================
*/

#pragma once

#include <string>
#include <array>

namespace crf::sensors::motioncapture {

/**
 * @ingroup group_motion_capture
 * @brief This struct contains the information streamed by a generic Motion Capture system
 *        for a generic marker. The information are the name of the marker as a string,
 *        its position with respect to a world frame as an array of three double values,
 *        and finally a boolean flag assessing weather the marker is occluded or not.
 */
struct MotionCaptureMarker {
    std::string markerName = "";
    std::array<double, 3> markerTranslation = {};
    bool markerOccluded = false;
};

}  // namespace crf::sensors::motioncapture
