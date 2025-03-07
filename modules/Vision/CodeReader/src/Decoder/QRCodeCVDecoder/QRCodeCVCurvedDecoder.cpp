/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>

#include "CodeReader/Decoder/QRCodeCVDecoder/QRCodeCVCurvedDecoder.hpp"

namespace crf::vision::codereader {

QRCodeCVCurvedDecoder::QRCodeCVCurvedDecoder():
    logger_("QRCodeCVCurvedDecoder") {
    logger_->debug("CTor");
}

QRCodeCVCurvedDecoder::~QRCodeCVCurvedDecoder() {
    logger_->debug("DTor");
}

crf::expected<std::string> QRCodeCVCurvedDecoder::decode(
    const cv::Mat &frame, std::array<float, 8> positions) {
    logger_->debug("decode(frame, pos)");
    if (!frame.data || frame.empty()) {
        logger_->warn("No data in frame");
        return crf::Code::BadRequest;
    }
    if (positions.empty()) {
        logger_->warn("No data in the positons");
        return crf::Code::BadRequest;
    }
    cv::Mat_<cv::Point> pos(1, 4, cv::Point(0, 0));
    for (uint16_t i = 0; i < 4; i++) {
        pos(i) = cv::Point(positions[2*i], positions[2*i+1]);
    }
    // Need to create a new object each time, bug
    cv::QRCodeDetector qrDecode = cv::QRCodeDetector();
    std::string result;
    try {
        result = qrDecode.decodeCurved(frame, pos);
    } catch (cv::Exception &e) {
        logger_->warn("Exception during decoding qr: {}", e.what());
        return crf::Code::ThirdPartyQueryFailed;
    }
    if (result == "") {
        logger_->warn("Could not decode the QR code or QR code empty");
        return crf::Code::ThirdPartyQueryFailed;
    }
    return result;
}

crf::expected<std::string> QRCodeCVCurvedDecoder::decode(const cv::Mat &frame) {
    logger_->debug("decode(frame)");
    if (!frame.data || frame.empty()) {
        logger_->warn("No data in frame");
        return crf::Code::BadRequest;
    }
    // Need to create a new object each time, bug
    cv::QRCodeDetector qrDecode = cv::QRCodeDetector();
    std::string result;
    try {
        result = qrDecode.detectAndDecodeCurved(frame);
    } catch (cv::Exception &e) {
        logger_->warn("Exception during detetcting and decoding qr: {}", e.what());
        return crf::Code::ThirdPartyQueryFailed;
    }
    if (result == "") {
        logger_->warn("Could not decode the QR code or QR code empty");
        return crf::Code::ThirdPartyQueryFailed;
    }
    return result;
}

}  // namespace crf::vision::codereader
