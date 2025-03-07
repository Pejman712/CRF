/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

#include "DownloadImageFromURL.hpp"
#include "CodeReader/Decoder/QRCodeCVDecoder/QRCodeCVCurvedDecoder.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

class QRCodeCVCurvedDecoderShould : public ::testing::Test {
 protected:
    QRCodeCVCurvedDecoderShould() :
        logger_("QRCodeCVCurvedDecoderShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~QRCodeCVCurvedDecoderShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() {
        sut_.reset(new crf::vision::codereader::QRCodeCVCurvedDecoder());
    }

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::vision::codereader::QRCodeCVCurvedDecoder> sut_;
};

TEST_F(QRCodeCVCurvedDecoderShould, notDecodeAnEmptyFrame) {
    ASSERT_FALSE(sut_->decode(cv::Mat()));
}

TEST_F(QRCodeCVCurvedDecoderShould, notDecodeAnEmptyFrameAndEmptyPositions) {
    ASSERT_FALSE(sut_->decode(cv::Mat(), std::array<float, 8>()));
}

TEST_F(QRCodeCVCurvedDecoderShould, DISABLED_DecodeCorrectlyAQRCode) {
    cv::Mat image = DownloadImageFromURL(
        "https://cernbox.cern.ch/remote.php/dav/public-files/rBXIBeYtmuoSLzJ/TextQR.png?access_token=null"); // NOLINT
    ASSERT_TRUE(sut_->decode(image));
    ASSERT_EQ(sut_->decode(image).value(), "Ah, haber estudiao");
}

TEST_F(QRCodeCVCurvedDecoderShould, DISABLED_DecodeCorrectlyAQRCodeWithPos) {
    cv::Mat image = DownloadImageFromURL(
        "https://cernbox.cern.ch/remote.php/dav/public-files/UCvXprYnifVIaFS/TextQR2.jpg?access_token=null"); // NOLINT
    std::array<float, 8> points = {
        1919.0298, 1400.4309, 2436, 1415, 2450.3726, 1965.0662, 1913, 1972};
    ASSERT_TRUE(sut_->decode(image, points));
    ASSERT_EQ(sut_->decode(image, points).value(), "Ah, haber estudiao");
}
