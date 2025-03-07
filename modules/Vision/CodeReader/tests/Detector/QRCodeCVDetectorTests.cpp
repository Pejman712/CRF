/* © Copyright crf 2022.  All rights reserved. This software is released under a crf proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to crf through mail-KT@crf.ch
 *
 * Author: Jorge Playan Garai crf BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

#include "../DownloadImageFromURL.hpp"

#include "CodeReader/Detector/QRCodeCVDetector/QRCodeCVDetector.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

class QRCodeCVDetectorShould : public ::testing::Test {
 protected:
    QRCodeCVDetectorShould() :
        logger_("QRCodeCVDetectorShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~QRCodeCVDetectorShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() {
        sut_.reset(new crf::vision::codereader::QRCodeCVDetector(cv::Size(300, 420), 0.05, 0.05));
    }

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::vision::codereader::QRCodeCVDetector> sut_;
};

TEST_F(QRCodeCVDetectorShould, notDetectInAnEmptyFrame) {
    ASSERT_FALSE(sut_->detect(cv::Mat()));
}

TEST_F(QRCodeCVDetectorShould, DetectCorrectlyAQRCode) {
    cv::Mat image = DownloadImageFromURL(
        "https://cernbox.cern.ch/remote.php/dav/public-files/UCvXprYnifVIaFS/TextQR2.jpg?access_token=null");  // NOLINT
    ASSERT_TRUE(sut_->detect(image));
}

TEST_F(QRCodeCVDetectorShould, DetectCorrectlyAnotherQRCode) {
    cv::Mat image = DownloadImageFromURL(
        "https://cernbox.cern.ch/remote.php/dav/public-files/rBXIBeYtmuoSLzJ/TextQR.png?access_token=null");  // NOLINT
    ASSERT_TRUE(sut_->detect(image));
    ASSERT_NO_THROW(sut_->getCodeTaskPose());
}

TEST_F(QRCodeCVDetectorShould, throwExceptionIfGetQRCodePositionIsCalledBeforeDetect) {
    ASSERT_THROW(sut_->getCodeTaskPose(), std::runtime_error);
}
