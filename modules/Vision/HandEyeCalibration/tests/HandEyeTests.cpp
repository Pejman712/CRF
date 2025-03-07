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
#include "HandEyeCalibration/HandEye.hpp"
#include "EventLogger/EventLogger.hpp"

using crf::applications::handeyecalibration::HandEye;

using testing::_;
using testing::Return;

class HandEyeShould: public ::testing::Test {
 protected:
    HandEyeShould(): logger_("HandEyeShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("HandEyeTests.cpp"));
        testDirName_ += "config/";
    }
    ~HandEyeShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::applications::handeyecalibration::IHandEye> sut_;
    std::string testDirName_;
};

TEST_F(HandEyeShould, throwExceptionWhenIncorrectConfigFileProvided) {
    ASSERT_THROW(sut_.reset(new HandEye(testDirName_ + "configFile_badSyntax.json")),
        nlohmann::detail::parse_error);
    ASSERT_THROW(sut_.reset(new HandEye(testDirName_ + "configFile_badKey.json")),
        nlohmann::detail::out_of_range);
    ASSERT_THROW(sut_.reset(new HandEye("configFile_badFileName.json")),
        nlohmann::detail::parse_error);
}

TEST_F(HandEyeShould, notThrowExceptionWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ + "configFile_good.json")));
}

TEST_F(HandEyeShould, notestimateChessboardPoseIfBadChessboardParametersProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ + "configFile_good.json")));
    ASSERT_FALSE(sut_->estimateChessboardPose(-1, -1, -1));
    ASSERT_FALSE(sut_->estimateChessboardPose(0, 6, 0.0254));  // invalid chessboard width
    ASSERT_FALSE(sut_->estimateChessboardPose(9, 0, 0.0254));  // invalid chessboard height
    ASSERT_FALSE(sut_->estimateChessboardPose(9, 6, 0));  // invalid chessboard square size
    ASSERT_FALSE(sut_->estimateChessboardPose(9, 7, 0.0254));  // even chessboard width and height
    ASSERT_FALSE(sut_->estimateChessboardPose(8, 6, 0.0254));  // odd chessboard width and height
}

TEST_F(HandEyeShould, notestimateChessboardPoseIfOneChessboardCornerNotFound) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ +
        "configFile_ImagesWithoutChessboard.json")));
    ASSERT_FALSE(sut_->estimateChessboardPose(9, 6, 0.0254));
}

TEST_F(HandEyeShould, notestimateChessboardPoseWhenIncorrectIntrinsicFilepathProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ +
        "configFile_badIntrinsics.json")));
    ASSERT_FALSE(sut_->estimateChessboardPose(9, 6, 0.0254));
}

TEST_F(HandEyeShould, notestimateChessboardPoseWhenIncorrectCameraNameProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ +
        "configFile_badCameraName.json")));
    ASSERT_FALSE(sut_->estimateChessboardPose(9, 6, 0.0254));
}

TEST_F(HandEyeShould, estimateChessboardPoseWhenCorrectFilesProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ + "configFile_good.json")));
    ASSERT_TRUE(sut_->estimateChessboardPose(9, 6, 0.0254));
}

TEST_F(HandEyeShould, notEstimateRobotCameraTransformWhenLessThan8ImagesProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ +
        "configFile_lessThan8Good_images.json")));
    ASSERT_TRUE(sut_->estimateChessboardPose(9, 6, 0.0254));
    ASSERT_FALSE(sut_->estimateRobotCameraTransform());
}

TEST_F(HandEyeShould, notEstimateRobotCameraTransformWhenBadRobotPosesProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ +
        "configFile_badRobotPoses.json")));
    ASSERT_TRUE(sut_->estimateChessboardPose(9, 6, 0.0254));
    ASSERT_FALSE(sut_->estimateRobotCameraTransform());
}

TEST_F(HandEyeShould, notEstimateRobotCameraTransformIfQRorChessboardePoseEstimationNotCalledBefore) {  // NOLINT
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ + "configFile_good.json")));
    ASSERT_FALSE(sut_->estimateRobotCameraTransform());
}

TEST_F(HandEyeShould, EstimateRobotCameraTransformWhenChessboardAndRobotPoseProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ + "configFile_good.json")));
    ASSERT_TRUE(sut_->estimateChessboardPose(9, 6, 0.0254));
    ASSERT_TRUE(sut_->estimateRobotCameraTransform());
}

TEST_F(HandEyeShould, notestimateQRPoseIfBadQRParametersProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ + "QRconfigFile_good.json")));
    ASSERT_FALSE(sut_->estimateQRCodePose(-1, 0.12));  // invalid QR width
    ASSERT_FALSE(sut_->estimateQRCodePose(0.12, -1));  // invalid QR height
}

TEST_F(HandEyeShould, notestimateQRPoseIfMoreThanOneQRCodeDetectedInAnyImage) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ +
        "configFile_multipleQR.json")));
    ASSERT_FALSE(sut_->estimateQRCodePose(0.12, 0.12));
}

TEST_F(HandEyeShould, notestimateQRPoseIfNoQRCodeDetectedInAnyImage) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ +
        "configFile_noQR.json")));
    ASSERT_FALSE(sut_->estimateQRCodePose(0.12, 0.12));
}

TEST_F(HandEyeShould, notestimateQRPoseWhenIncorrectIntrinsicFilepathProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ +
        "configFile_badIntrinsics.json")));
    ASSERT_FALSE(sut_->estimateQRCodePose(0.12, 0.12));
}

TEST_F(HandEyeShould, notestimateQRPoseWhenIncorrectCameraNameProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ +
        "configFile_badCameraName.json")));
    ASSERT_FALSE(sut_->estimateQRCodePose(0.12, 0.12));
}

TEST_F(HandEyeShould, estimateQRPoseWhenCorrectFilesProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ + "QRconfigFile_good.json")));
    ASSERT_TRUE(sut_->estimateQRCodePose(0.12, 0.12));
}

TEST_F(HandEyeShould, EstimateRobotCameraTransformWhenQRCodeAndRobotPoseProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandEye(testDirName_ + "QRconfigFile_good.json")));
    ASSERT_TRUE(sut_->estimateQRCodePose(0.12, 0.12));
    ASSERT_TRUE(sut_->estimateRobotCameraTransform());
}

