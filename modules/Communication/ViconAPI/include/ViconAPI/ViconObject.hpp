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
#include <vector>
#include <ViconAPI/ViconMarker.hpp>

namespace crf::communication::viconapi {

/**
 * @ingroup group_vicon_api
 * @brief This struct contains the information streamed by vicon APIs for a generic rigid object.
 *        The information are: the name of the rigid object as a string, its translation with
 *        respect to a world frame as an array of three double values, its orientation
 *        (in quaternion) as an array of four double values, a vector containing the infos
 *        on the markers defining the object and finally a boolean flag assessing weather
 *        the object is occluded or not.
 */
struct ViconObject {
    std::string objectName;
    std::array<double, 3> objectTranslation;
    std::array<double, 4> objectQuaternion;
    std::vector<ViconMarker> objectMarkers;
    bool objectOccluded;
};

}  // namespace crf::communication::viconapi
