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

#include "DownloadImageFromURL.hpp"
#include "CodeReader/Decoder/QRCodeCVDecoder/QRCodeCVDecoder.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

class QRCodeCVDecoderShould : public ::testing::Test {
 protected:
    QRCodeCVDecoderShould() :
        logger_("QRCodeCVDecoderShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~QRCodeCVDecoderShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() {
        sut_.reset(new crf::vision::codereader::QRCodeCVDecoder());
    }

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::vision::codereader::QRCodeCVDecoder> sut_;
};

TEST_F(QRCodeCVDecoderShould, notDecodeAnEmptyFrame) {
    ASSERT_FALSE(sut_->decode(cv::Mat()));
}

TEST_F(QRCodeCVDecoderShould, notDecodeAnEmptyFrameAndEmptyPositions) {
    ASSERT_FALSE(sut_->decode(cv::Mat(), std::array<float, 8>()));
}

TEST_F(QRCodeCVDecoderShould, DecodeCorrectlyAQRCode) {
    cv::Mat image = DownloadImageFromURL(
        "https://cernbox.cern.ch/remote.php/dav/public-files/rBXIBeYtmuoSLzJ/TextQR.png?access_token=null");  // NOLINT
    ASSERT_TRUE(sut_->decode(image));
    ASSERT_EQ(sut_->decode(image).value(), "Ah, haber estudiao");
}
TEST_F(QRCodeCVDecoderShould, DISABLED_DecodeCorrectlyAQRCodeFromItsPosition) {
    cv::Mat image = DownloadImageFromURL(
        "https://cernbox.cern.ch/remote.php/dav/public-files/UCvXprYnifVIaFS/TextQR2.jpg?access_token=null");  // NOLINT
    std::array<float, 8> points = {
    1919.0298, 1400.4309, 2436, 1415, 2450.3726, 1965.0662, 1913, 1972};
    ASSERT_TRUE(sut_->decode(image, points));
    ASSERT_EQ(sut_->decode(image, points).value(), "Ah, haber estudiao");
}
