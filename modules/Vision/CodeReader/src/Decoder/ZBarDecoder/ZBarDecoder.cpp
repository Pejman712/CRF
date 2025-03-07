/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
*/

#include <string>

#include "CodeReader/Decoder/ZBarDecoder/ZBarDecoder.hpp"

namespace crf::vision::codereader {

ZBarDecoder::ZBarDecoder():
    logger_("ZBarDecoder") {
    logger_->debug("CTor");
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
}

ZBarDecoder::~ZBarDecoder() {
    logger_->debug("DTor");
}

crf::expected<std::string> ZBarDecoder::decode(
    const cv::Mat &frame, std::array<float, 8> positions) {
    logger_->debug("decode(frame, positions)");
    logger_->warn("ZBar does it's own detection and decoding, no need for positions");
    return decode(frame);
}

crf::expected<std::string> ZBarDecoder::decode(const cv::Mat &frame) {
    logger_->debug("decode(frame)");
    if (!frame.data || frame.empty()) {
        logger_->warn("No data in frame");
        return crf::Code::BadRequest;
    }
    std::string data;
    cv::Mat grayFrame;
    cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
    zbar::Image image(grayFrame.cols, grayFrame.rows, "Y800",
        reinterpret_cast<uchar*>(grayFrame.data), grayFrame.cols*grayFrame.rows);
    scanner_.scan(image);
    for (auto symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        data = symbol->get_data();
    }
    image.set_data(NULL, 0);
    if (data == "") {
        logger_->warn("Could not decode the QR code or QR code empty");
        return crf::Code::ThirdPartyQueryFailed;
    }
    return data;
}

}  // namespace crf::vision::codereader
