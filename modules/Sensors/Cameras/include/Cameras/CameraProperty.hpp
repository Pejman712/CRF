/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alvaro Garcia Gonzalez BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

namespace crf::sensors::cameras {

/**
 * @ingroup group_cameras
 * @brief Enum to classify alll physical properties a camera can have
 *
 * @{
 */

enum class Property {
    BRIGHTNESS,
    CONTRAST,
    SATURATION,
    HUE,
    GAIN,
    EXPOSURE,
    FOCUS,
    FOCUSMODE,
    SHUTTER,
    ISO,
    ZOOM,
    PAN,
    TILT,
    ROLL
};

/**@}*/

}  // namespace crf::sensors::cameras
