/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
*/

#pragma once

#include <string>
#include <optional>

#include <opencv2/opencv.hpp>
#include <zbar.h>

#include "CodeReader/Decoder/IDecoder.hpp"
#include "EventLogger/EventLogger.hpp"

/**
 * @brief ZBar can read codes such as : EAN-13/UPC-A, UPC-E, EAN-8, Code 128, Code 39,
 * Interleaved 2 of 5 and QR Code.
 */

namespace crf::vision::codereader {

class ZBarDecoder: public IDecoder {
 public:
    ZBarDecoder();
    ~ZBarDecoder() override;

    crf::expected<std::string> decode(
        const cv::Mat &frame, std::array<float, 8> positions) override;
    crf::expected<std::string> decode(const cv::Mat &frame) override;

 private:
    zbar::ImageScanner scanner_;
    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::vision::codereader
