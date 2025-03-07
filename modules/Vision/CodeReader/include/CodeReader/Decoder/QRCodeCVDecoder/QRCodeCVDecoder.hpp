/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include <opencv2/opencv.hpp>

#include "CodeReader/Decoder/IDecoder.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::vision::codereader {

class QRCodeCVDecoder: public IDecoder {
 public:
    QRCodeCVDecoder();
    ~QRCodeCVDecoder() override;

    crf::expected<std::string> decode(
        const cv::Mat &frame, std::array<float, 8> positions) override;
    crf::expected<std::string> decode(const cv::Mat &frame) override;

 private:
    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::vision::codereader
