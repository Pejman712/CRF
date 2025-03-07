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
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

#include "../DownloadImageFromURL.hpp"

#include "Cameras/CameraMockConfiguration.hpp"

#include "CodeReader/Detector/QRCodeCVDetector/QRCodeCVDetector.hpp"
#include "CodeReader/Decoder/QRCodeCVDecoder/QRCodeCVDecoder.hpp"
#include "CodeReader/Decoder/ZBarDecoder/ZBarDecoder.hpp"
#include "CodeReader/Reader/CodeBackgroundReader/CodeBackgroundReader.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

using crf::sensors::cameras::CameraMock;
using crf::sensors::cameras::CameraMockConfiguration;
using crf::vision::codereader::QRCodeCVDetector;
using crf::vision::codereader::QRCodeCVDecoder;
using crf::vision::codereader::ZBarDecoder;
using crf::vision::codereader::CodeBackgroundReader;

class CodeBackgroundReaderShould : public ::testing::Test {
 protected:
    CodeBackgroundReaderShould() :
        zbar_(false),
        cameraMock_(std::make_shared<NiceMock<CameraMockConfiguration>>()),
        logger_("CodeBackgroundReaderShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~CodeBackgroundReaderShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() {
        qrDecoder_ = std::make_shared<QRCodeCVDecoder>();
        qrDetector_ = std::make_shared<QRCodeCVDetector>();
        zbardecoder_ = std::make_shared<ZBarDecoder>();
        cameraMock_->setFrame(DownloadImageFromURL(
            "https://cernbox.cern.ch/remote.php/dav/public-files/rBXIBeYtmuoSLzJ/TextQR.png?access_token=null"));  // NOLINT
        if (zbar_) {
            sut_.reset(new CodeBackgroundReader(
                cameraMock_,
                zbardecoder_));
        } else {
            sut_.reset(new CodeBackgroundReader(
                cameraMock_,
                qrDecoder_));
        }
    }

    std::atomic<bool> zbar_;
    std::shared_ptr<QRCodeCVDecoder> qrDecoder_;
    std::shared_ptr<QRCodeCVDetector> qrDetector_;
    std::shared_ptr<ZBarDecoder> zbardecoder_;
    std::shared_ptr<NiceMock<CameraMockConfiguration>> cameraMock_;
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<CodeBackgroundReader> sut_;
};

TEST_F(CodeBackgroundReaderShould, CorrectlyReadTheQRCode) {
    while (!sut_->isDetected()) {
        sut_->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_EQ(sut_->getCode(), "Ah, haber estudiao");
    sut_->stop();
}

TEST_F(CodeBackgroundReaderShould, WorkWithDetectorAndDecoder) {
    while (!sut_->isDetected()) {
        sut_->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_EQ(sut_->getCode(), "Ah, haber estudiao");
    sut_->stop();
}

TEST_F(CodeBackgroundReaderShould, WorkWithZbarDecoder) {
    zbar_ = true;
    while (!sut_->isDetected()) {
        sut_->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_EQ(sut_->getCode(), "Ah, haber estudiao");
    sut_->stop();
}

TEST_F(CodeBackgroundReaderShould, NotDetectAnythingIfThereIsNoImageOrImageEmpty) {
    cameraMock_->setFrame(cv::Mat());
    sut_->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_FALSE(sut_->isDetected());
    sut_->stop();
}
