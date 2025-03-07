#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

namespace crf {
namespace applications {
namespace handeyecalibration {
class IHandEye {
 public:
    virtual ~IHandEye() = default;
    // Compute camera to Chessboard (object) transformation matrix
    // - Camera to object(cMo) : when camera attached to robot end effector
    // - Object to Camera(oMc) : when camera is fixed and not moving with robot
    virtual bool estimateChessboardPose(int chessboardWidth, int chessboardHeight,
        double chessboardSquareSize) = 0;
    // Compute camera to QRCode (object) transformation matrix
    // - Camera to object(cMo) : when camera attached to robot end effector
    // - Object to Camera(oMc) : when camera is fixed and not moving with robot
    virtual bool estimateQRCodePose(double codeWidthInMeters, double codeHeightInMeters) = 0;
    // Estimate end-effector to camera transformation matrix:
    // - End effecto to Camera(eMc): when camera attached to robot end effector
    //.- Robot base or world to Camera(wMc): when camera fixed and not moving with robot
    virtual bool estimateRobotCameraTransform() = 0;
};
}  // namespace handeyecalibration
}  // namespace applications
}  // namespace crf
