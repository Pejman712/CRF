/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <atomic>
#include <vector>
#include <array>

#include <opencv2/opencv.hpp>
#include <visp3/vision/vpKeyPoint.h>

#include "CodeReader/Detector/IDetector.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::vision::codereader {

class QRCodeCVDetector: public IDetector {
 public:
    QRCodeCVDetector(
        const cv::Size& cameraResolution = cv::Size(),
        const double& widthQR = 0.0, const double& heightQR = 0.0);
    ~QRCodeCVDetector() override;

    bool detect(const cv::Mat &frame) override;
    std::vector<crf::utility::types::TaskPose> getCodeTaskPose() override;
    std::array<float, 8> getPositionInImage() override;

 private:
    cv::Size cameraResolution_;
    double widthQR_;
    double heightQR_;
    std::atomic<bool> detected_;
    bool parametersProvided_;
    cv::Mat frame_;
    cv::QRCodeDetector qrDetect_;
    cv::Mat positions_;

    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::vision::codereader
