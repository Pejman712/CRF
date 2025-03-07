/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author:Sergio CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "HandEyeCalibration/IHandEye.hpp"
#include "HandEyeCalibration/FixedHandeye.hpp"

using crf::applications::handeyecalibration::FixedHandeye;

using testing::_;
using testing::Return;

class FixedHandeyeShould: public ::testing::Test {
 protected:
    FixedHandeyeShould() {
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("FixedHandeyeTests.cpp"));
        testDirName_ += "config/";
    }
    std::unique_ptr<crf::applications::handeyecalibration::IHandEye> sut_;
    std::string testDirName_;
};

TEST_F(FixedHandeyeShould, throwExceptionWhenIncorrectConfigFileProvided) {
    ASSERT_THROW(sut_.reset(new FixedHandeye(testDirName_ + "configFile_badSyntax.json")),
        nlohmann::detail::parse_error);
    ASSERT_THROW(sut_.reset(new FixedHandeye(testDirName_ + "fixedconfigFile_badKey.json")),
        nlohmann::detail::out_of_range);
    ASSERT_THROW(sut_.reset(new FixedHandeye("fixedconfigFile_badFileName.json")),
        nlohmann::detail::parse_error);
}

TEST_F(FixedHandeyeShould, notThrowExceptionWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ + "QRfixedconfigFile_good.json")));
}

TEST_F(FixedHandeyeShould, notestimateChessboardPoseIfBadChessboardParametersProvided) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ + "fixedconfigFile_good.json")));
    ASSERT_FALSE(sut_->estimateChessboardPose(-1, -1, -1));
    ASSERT_FALSE(sut_->estimateChessboardPose(0, 6, 0.0254));  // invalid chessboard width
    ASSERT_FALSE(sut_->estimateChessboardPose(9, 0, 0.0254));  // invalid chessboard height
    ASSERT_FALSE(sut_->estimateChessboardPose(9, 6, 0));  // invalid chessboard square size
    ASSERT_FALSE(sut_->estimateChessboardPose(9, 7, 0.0254));  // even chessboard width and height
    ASSERT_FALSE(sut_->estimateChessboardPose(8, 6, 0.0254));  // odd chessboard width and height
}

TEST_F(FixedHandeyeShould, notestimateChessboardPoseIfOneChessboardCornerNotFound) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ +
        "fixedconfigFile_ImagesWithoutChessboard.json")));
    ASSERT_FALSE(sut_->estimateChessboardPose(9, 6, 0.0254));
}

TEST_F(FixedHandeyeShould, notestimateChessboardPoseWhenIncorrectIntrinsicFilepathProvided) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ +
        "fixedconfigFile_badIntrinsics.json")));
    ASSERT_FALSE(sut_->estimateChessboardPose(9, 6, 0.0254));
}

TEST_F(FixedHandeyeShould, notestimateChessboardPoseWhenIncorrectCameraNameProvided) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ +
        "fixedconfigFile_badCameraName.json")));
    ASSERT_FALSE(sut_->estimateChessboardPose(9, 6, 0.0254));
}

TEST_F(FixedHandeyeShould, estimateChessboardPoseWhenCorrectFilesProvided) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ + "fixedconfigFile_good.json")));
    ASSERT_TRUE(sut_->estimateChessboardPose(9, 6, 0.0254));
}

TEST_F(FixedHandeyeShould, notEstimateRobotCameraTransformWhenLessThan8ImagesProvided) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ +
        "fixedconfigFile_lessThan8Good_images.json")));
    ASSERT_TRUE(sut_->estimateChessboardPose(9, 6, 0.0254));
    ASSERT_FALSE(sut_->estimateRobotCameraTransform());
}

TEST_F(FixedHandeyeShould, notEstimateRobotCameraTransformWhenBadRobotPosesProvided) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ +
        "fixedconfigFile_badRobotPoses.json")));
    ASSERT_TRUE(sut_->estimateQRCodePose(0.12, 0.12));
    ASSERT_FALSE(sut_->estimateRobotCameraTransform());
}

TEST_F(FixedHandeyeShould, notEstimateRobotCameraTransformIfQRorChessboardePoseEstimationNotCalledBefore) {  // NOLINT
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ + "fixedconfigFile_good.json")));
    ASSERT_FALSE(sut_->estimateRobotCameraTransform());
}

TEST_F(FixedHandeyeShould, EstimateRobotCameraTransformWhenChessboardAndRobotPoseProvided) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ + "fixedconfigFile_good.json")));
    ASSERT_TRUE(sut_->estimateChessboardPose(9, 6, 0.0254));
    ASSERT_TRUE(sut_->estimateRobotCameraTransform());
}

TEST_F(FixedHandeyeShould, notestimateQRPoseIfBadQRParametersProvided) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ + "QRfixedconfigFile_good.json")));
    ASSERT_FALSE(sut_->estimateQRCodePose(-1, 0.12));  // invalid QR width
    ASSERT_FALSE(sut_->estimateQRCodePose(0.12, -1));  // invalid QR height
}

TEST_F(FixedHandeyeShould, notestimateQRPoseIfMoreThanOneQRCodeDetectedInAnyImage) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ +
        "QRfixedconfigFile_multipleQR.json")));
    ASSERT_FALSE(sut_->estimateQRCodePose(0.12, 0.12));
}

TEST_F(FixedHandeyeShould, notestimateQRPoseIfNoQRCodeDetectedInAnyImage) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ +
        "QRfixedconfigFile_noQR.json")));
    ASSERT_FALSE(sut_->estimateQRCodePose(0.12, 0.12));
}

TEST_F(FixedHandeyeShould, notestimateQRPoseWhenIncorrectIntrinsicFilepathProvided) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ +
        "fixedconfigFile_badIntrinsics.json")));
    ASSERT_FALSE(sut_->estimateQRCodePose(0.12, 0.12));
}

TEST_F(FixedHandeyeShould, notestimateQRPoseWhenIncorrectCameraNameProvided) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ +
        "fixedconfigFile_badCameraName.json")));
    ASSERT_FALSE(sut_->estimateQRCodePose(0.12, 0.12));
}

TEST_F(FixedHandeyeShould, estimateQRdPoseWhenCorrectFilesProvided) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ + "QRfixedconfigFile_good.json")));
    ASSERT_TRUE(sut_->estimateQRCodePose(0.12, 0.12));
}

TEST_F(FixedHandeyeShould, EstimateRobotCameraTransformWhenQRCodeAndRobotPoseProvided) {
    ASSERT_NO_THROW(sut_.reset(new FixedHandeye(testDirName_ + "QRfixedconfigFile_good.json")));
    ASSERT_TRUE(sut_->estimateQRCodePose(0.12, 0.12));
    ASSERT_TRUE(sut_->estimateRobotCameraTransform());
}

