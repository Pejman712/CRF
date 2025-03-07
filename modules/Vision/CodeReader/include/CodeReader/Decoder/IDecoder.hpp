/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include "crf/expected.hpp"

namespace crf::vision::codereader {

class IDecoder {
 public:
    virtual ~IDecoder() = default;

    /**
     * @brief  Function to decode a code on a given image knowing the corners of it
     * @param  Frame where the QR code is present
     * @param  Array of 4 points giving the X and Y position in pixels of the corners
     *         of the code (8 floats in total), they should be in order to form a square
     *         on a straight line (two points consecutive on the array should be consecutive
     *         on the image too)
     * @return Returns a string with the information encrypted on the code
     * @return If the decoding fails, it returns an error code
     */
    virtual crf::expected<std::string> decode(
        const cv::Mat &frame, std::array<float, 8> positions) = 0;
    /**
     * @brief  Function to detect and decode a QR on a given image
     * @param  Frame where the QR code is present
     * @return Returns a string with the information encrypted on the QR
     * @return If the decoding fails, it returns an error code
     */
    virtual crf::expected<std::string> decode(const cv::Mat &frame) = 0;
};

}  // namespace crf::vision::codereader
