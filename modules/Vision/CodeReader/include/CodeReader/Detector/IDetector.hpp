/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <array>
#include <vector>

#include <opencv2/opencv.hpp>

#include "Types/Types.hpp"

namespace crf::vision::codereader {

class IDetector {
 public:
    virtual ~IDetector() = default;
    /**
     * @brief  Function to detect a code on an image
     * @param  Image in which we want to detect the QR code
     * @return True if the detection was succesful
     * @return False if the detection failed
     */
    virtual bool detect(const cv::Mat &frame) = 0;

    /**
     * @brief  Once the code has been detected, this function provides the position
     *         of the code on space
     * @return Returns a vector of TaskPoses of the centers of how many codes are in the image
     *         or of how many the algorithm can detect
     */
    virtual std::vector<crf::utility::types::TaskPose> getCodeTaskPose() = 0;

    /**
     * @brief  Once the code has been detected, this function provides the position
     *         of the code on the image
     * @return Returns an array of 4 points giving the X and Y position in pixels of the corners
     *         of the code (8 floats in total), they should be in order to form a square
     *         on a straight line (two points consecutive on the array should be consecutive
     *         on the image too)
     */
    virtual std::array<float, 8> getPositionInImage() = 0;
};

}  // namespace crf::vision::codereader
