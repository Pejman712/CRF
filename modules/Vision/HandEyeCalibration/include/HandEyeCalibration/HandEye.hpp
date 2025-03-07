#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include <vector>
#include <memory>
#include <string>
#include <visp3/core/vpPoint.h>

#include "EventLogger/EventLogger.hpp"
#include "HandEyeCalibration/IHandEye.hpp"

namespace crf {
namespace applications {
namespace handeyecalibration {

class HandEye: public IHandEye {
 public:
    HandEye() = delete;
    HandEye(const HandEye& other) = delete;
    HandEye(HandEye&& other) = delete;
    explicit HandEye(const std::string& configFileName);
    ~HandEye() override;
    bool estimateChessboardPose(int chessboardWidth,
        int chessboardHeight, double chessboardSquareSize) override;
    bool estimateQRCodePose(double codeWidthInMeters, double codeHeightInMeters) override;
    bool estimateRobotCameraTransform() override;

 private:
    vpHomogeneousMatrix computeQRPose(const std::vector<vpPoint>& points,
        const std::vector<vpImagePoint> &ip, const vpCameraParameters &cam, bool init);
    bool checkChessboardParameters(int chessboardWidth,
         int chessboardHeight, double chessboardSquareSize);
    bool checkQRCodeParameters(double codeWidthInMeters, double codeHeightInMeters);
    std::vector<vpPoint> calcChessboardCorners(int width, int height, double squareSize);
    crf::utility::logger::EventLogger _log;
    std::string _inputImageFilename;
    std::string _inputPoseFilename;
    std::string _outputFilename;
    std::string _intrinsicFile;
    std::string _cameraName;
    unsigned int _numberOfFrames;
};

}  // namespace handeyecalibration
}  // namespace applications
}  // namespace crf
