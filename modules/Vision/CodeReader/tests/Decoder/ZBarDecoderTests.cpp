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
#include "CodeReader/Decoder/ZBarDecoder/ZBarDecoder.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

class ZBarDecoderShould : public ::testing::Test {
 protected:
    ZBarDecoderShould() :
        logger_("ZBarDecoderShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~ZBarDecoderShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() {
        sut_.reset(new crf::vision::codereader::ZBarDecoder());
    }

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::vision::codereader::ZBarDecoder> sut_;
};

TEST_F(ZBarDecoderShould, notDecodeAnEmptyFrame) {
    ASSERT_FALSE(sut_->decode(cv::Mat()));
}

TEST_F(ZBarDecoderShould, notDecodeAnEmptyFrameAndEmptyPositions) {
    ASSERT_FALSE(sut_->decode(cv::Mat(), std::array<float, 8>()));
}

TEST_F(ZBarDecoderShould, DecodeCorrectlyAQRCode) {
    cv::Mat image = DownloadImageFromURL(
        "https://cernbox.cern.ch/remote.php/dav/public-files/rBXIBeYtmuoSLzJ/TextQR.png?access_token=null");  // NOLINT
    ASSERT_TRUE(sut_->decode(image));
    ASSERT_EQ(sut_->decode(image).value(), "Ah, haber estudiao");
}
