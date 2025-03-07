/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Victor Drame CERN BE/CEM/MRO 2022
 *
 * ====================================================================
*/

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "../DownloadImageFromURL.hpp"

#include "CodeReader/Detector/ArUcoCVDetector/ArUcoCVDetector.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

class ArUcoCVDetectorShould : public ::testing::Test {
 protected:
    ArUcoCVDetectorShould() :
        logger_("ArUcoCVDetectorShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~ArUcoCVDetectorShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    cv::Mat intrinsicmtx_ = (cv::Mat_<double>(3, 3) <<
        500,              0.0,                 300,
        0.0,              500,                 240,
        0.0,              0.0,                 1.0);;
    cv::Mat distortionmtx_ = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
    int dictionaryId_ = 5;
    double markerLength_ = 0.1;

    void SetUp() {
        sut_.reset(new crf::vision::codereader::ArUcoCVDetector(
            dictionaryId_, intrinsicmtx_, distortionmtx_, markerLength_));
    }

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::vision::codereader::ArUcoCVDetector> sut_;
};

TEST_F(ArUcoCVDetectorShould, notDetectInAnEmptyFrame) {
    ASSERT_FALSE(sut_->detect(cv::Mat()));
}

TEST_F(ArUcoCVDetectorShould, NotDetectAnArUcoMarker) {
    cv::Mat image = DownloadImageFromURL("https://cernbox.cern.ch/remote.php/dav/public-files/pnm0c0GGUy5EdUL/NotDetectAnArUcoMarker.jpg?access_token=null");  // NOLINT
    ASSERT_FALSE(sut_->detect(image));
}

TEST_F(ArUcoCVDetectorShould, NotDetectArUcoMarkerFromAnAngle) {
    cv::Mat image = DownloadImageFromURL("https://cernbox.cern.ch/remote.php/dav/public-files/WZubWQsAdYIVhfQ/1C00104F-3891-4FEC-8682-2B8ACC073F1E.jpg?access_token=null");  // NOLINT
    ASSERT_FALSE(sut_->detect(image));
}

TEST_F(ArUcoCVDetectorShould, DetectCorrectlyAnArucoCode) {
    cv::Mat image = DownloadImageFromURL(
        "https://cernbox.cern.ch/remote.php/dav/public-files/dAqGHthINxpTqBV/white_aruco.png?access_token=null");  // NOLINT
    ASSERT_TRUE(sut_->detect(image));
}

TEST_F(ArUcoCVDetectorShould, DetectCorrectlyAnotherArucoCode) {
    cv::Mat image = DownloadImageFromURL(
        "https://cernbox.cern.ch/remote.php/dav/public-files/jyxIwzDmpqgnloT/auroco_markers.png?access_token=null");  // NOLINT
    ASSERT_TRUE(sut_->detect(image));
    ASSERT_NO_THROW(sut_->getCodeTaskPose());
    ASSERT_NO_THROW(sut_->getMarkersIds());
}

TEST_F(ArUcoCVDetectorShould,
    throwExceptionIfGetArucoPositionIsCalledBeforeDetect) {
    ASSERT_THROW(sut_->getCodeTaskPose(), std::runtime_error);
    ASSERT_THROW(sut_->getMarkersIds(), std::runtime_error);
}

TEST_F(ArUcoCVDetectorShould, ReturnPositionOfArUcoMarker) {
    cv::Mat image = DownloadImageFromURL(
        "https://cernbox.cern.ch/remote.php/dav/public-files/dAqGHthINxpTqBV/white_aruco.png?access_token=null");  // NOLINT
    std::array<float, 8> positionInImage = {347.0, 161.0, 553.0, 151.0, 564.0, 360.0, 349.0, 368.0};
    sut_->detect(image);
    auto expectedPositionInImage = sut_->getPositionInImage();
    ASSERT_NEAR(positionInImage[0], expectedPositionInImage[0], 2);
    ASSERT_NEAR(positionInImage[1], expectedPositionInImage[1], 2);
    ASSERT_NEAR(positionInImage[2], expectedPositionInImage[2], 2);
    ASSERT_NEAR(positionInImage[3], expectedPositionInImage[3], 2);
    ASSERT_NEAR(positionInImage[4], expectedPositionInImage[4], 2);
    ASSERT_NEAR(positionInImage[5], expectedPositionInImage[5], 2);
    ASSERT_NEAR(positionInImage[6], expectedPositionInImage[6], 2);
    ASSERT_NEAR(positionInImage[7], expectedPositionInImage[7], 2);
}
