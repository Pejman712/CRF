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
#include <visp3/core/vpQuaternionVector.h>

#include "EventLogger/EventLogger.hpp"
#include "HandEyeCalibration/IHandEye.hpp"

namespace crf {
namespace applications {
namespace handeyecalibration {

class FixedHandeye: public IHandEye {
 public:
    FixedHandeye() = delete;
    explicit FixedHandeye(const std::string& configFileName);
    ~FixedHandeye() override;

    bool estimateQRCodePose(double codeWidthInMeters, double codeHeightInMeters) override;
    bool estimateChessboardPose(int chessboardWidth,
         int chessboardHeight, double chessboardSquareSize) override;
    bool estimateRobotCameraTransform() override;

 private:
    vpHomogeneousMatrix computeQRPose(const std::vector<vpPoint>& points,
        const std::vector<vpImagePoint> &ip, const vpCameraParameters &cam, bool init);
    bool estimateChessboardCameraTransform(int chessboardWidth,
         int chessboardHeight, double chessboardSquareSize);
    bool estimateEndEffectorPatternTransform();
    bool checkChessboardParameters(int chessboardWidth,
         int chessboardHeight, double chessboardSquareSize);
    bool checkQRCodeParameters(double codeWidthInMeters, double codeHeightInMeters);
    std::vector<vpPoint> calcChessboardCorners(int width, int height, double squareSize);
    vpQuaternionVector AverageQuaternion(std::vector<float> &cumulative, // NOLINT
        vpQuaternionVector newRotation, vpQuaternionVector firstRotation, int addAmount);
    float innerProduct(const vpQuaternionVector& q1, const vpQuaternionVector& q2);
    vpQuaternionVector NormalizeQuaternion(float x, float y, float z, float w);

    crf::utility::logger::EventLogger _log;
    std::string _inputImageFilename;
    std::string _inputPoseFilename;
    std::string _endEffectorToObjectoutputFilename;
    std::string _worldToCameraoutputFilename;
    std::string _intrinsicFile;
    std::string _cameraName;
    unsigned int _numberOfFrames;
};
}  // namespace handeyecalibration
}  // namespace applications
}  // namespace crf
