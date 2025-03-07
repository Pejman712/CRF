/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alvaro Garcia Gonzalez BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

namespace crf::sensors::cameras {

/**
 * @ingroup group_cameras
 * @brief Struct to represent a profile. It is composed of
 * a resolution and a framerate. Several framerates can be
 * added if the resolution allows it
 *
 * @{
 */

struct Profile {
    cv::Size resolution;
    std::vector<uint64_t> framerates;
    uint64_t framerate;

    Profile() = default;
    /**
     * @brief Construct a new Profile object
     *
     * @param _resolution Correspondent resolution
     * @param _framerates Possible framerates for this resolution
     */
    Profile(const cv::Size& _resolution, const std::vector<uint64_t>& _framerates):
        resolution(_resolution),
        framerates(_framerates),
        framerate(0) {
            if (framerates.size() < 1) {
                throw std::logic_error("A profile needs to have at least one framerate");
            }
            framerate = framerates[0];
    }

    /**
     * @brief Construct a new Profile object
     *
     * @param _resolution Correspondent resolution
     * @param _framerate Framerate for this resolution
     */
    Profile(const cv::Size& _resolution, const uint64_t& _framerate):
        resolution(_resolution),
        framerates({_framerate}),
        framerate(_framerate) {}
};

/**@}*/

}  // namespace crf::sensors::cameras
