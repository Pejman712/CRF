/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "VideoCodecs/IVideoEncoder.hpp"

namespace crf::vision::videocodecs {

class cvMatVideoEncoder : public IVideoEncoder {
 public:
    cvMatVideoEncoder();
    ~cvMatVideoEncoder() override = default;
    bool addFrame(const cv::Mat&) override;
    std::string getBytes(bool clearBuffer = true) override;
    bool flush() override;

    CompressionQuality getCompressionQuality() override;
    cv::Size getResolution() override;

 private:
    utility::logger::EventLogger logger_;
    std::string bytes_;

    bool flushed_;
    std::string nalDivider_;
};

}  // namespace crf::vision::videocodecs
