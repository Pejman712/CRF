/* Copyright 2017 CERN */

#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include "Mocks/Vision/Output3DWrapperMock.hpp"
#include "Mocks/Applications/VideoCaptureMock.hpp"

#include "DirectSparseOdometry/IOWrapper/OutputWrapper/PangolinDSOViewer.hpp"

#include "EventLogger/EventLogger.hpp"
#include "DsoDevel/DsoDevel.hpp"

#define CONFIG_FILE_GOOD "dsoConfigFile_Good.json"

class DsoShould: public ::testing::Test {
 protected:
    DsoShould(): testDebugOutput_("DsoShould") {
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("DsoTests.cpp"));
        testDirName_ += "configFiles/";
        testDebugOutput_->debug("testDirName_: {}", testDirName_);
        owMock_.reset(new dso::IOWrap::Output3DWrapperMock);
        ON_CALL(*owMock_, needPushDepthImage()).WillByDefault(::testing::Return(true));
        cameraMock_.reset(new cv::VideoCaptureMock);
        ON_CALL(*cameraMock_, isOpened()).WillByDefault(::testing::Return(true));
    }
    void createSystemNormally() {
        sut_.reset(new crf::applications::dsodevel::DsoDevel(
            testDirName_ + CONFIG_FILE_GOOD, cameraMock_));
    }

    crf::utility::logger::EventLogger testDebugOutput_;
    std::string testDirName_;
    std::shared_ptr<dso::IOWrap::Output3DWrapperMock> owMock_;
    std::shared_ptr<cv::VideoCaptureMock> cameraMock_;
    std::unique_ptr<crf::applications::dsodevel::DsoDevel> sut_;
};

TEST_F(DsoShould, throwWhenCameraIsNotWorking) {
    EXPECT_CALL(*cameraMock_, isOpened()).WillOnce(::testing::Return(false));
    testDebugOutput_->debug("Bad camera test");
    EXPECT_ANY_THROW(
        sut_.reset(new crf::applications::dsodevel::DsoDevel(
            testDirName_ + CONFIG_FILE_GOOD, cameraMock_)));
}

TEST_F(DsoShould, throwWhenConfigurationIsWrong) {
    /*
     * Should throw when config file doesn't exist
     */
    testDebugOutput_->debug("No cfg file test");
    EXPECT_ANY_THROW(
        sut_.reset(new crf::applications::dsodevel::DsoDevel(
            testDirName_ + "blebleble", cameraMock_)));
    /*
     * Should throw when config file exists but has a bad json syntax
     */
    testDebugOutput_->debug("Bad json test");
    EXPECT_ANY_THROW(
        sut_.reset(new crf::applications::dsodevel::DsoDevel(
            testDirName_ + "dsoConfigFile_BadJson.json", cameraMock_)));
    /*
     * Should throw when config file exists but doesn't have all required keys
     */
    testDebugOutput_->debug("Missing key test");
    EXPECT_ANY_THROW(
        sut_.reset(new crf::applications::dsodevel::DsoDevel(
            testDirName_ + "dsoConfigFile_MissingKey.json", cameraMock_)));
    /*
     * Should throw when some of the files mentioned in cfg doesn't exist
     */
    testDebugOutput_->debug("No camera cfg test");
    EXPECT_ANY_THROW(
        sut_.reset(new crf::applications::dsodevel::DsoDevel(
            testDirName_ + "dsoConfigFile_NoCameraCfg.json", cameraMock_)));
    /*
     * Should throw when photometric calib fails
     */
    testDebugOutput_->debug("Bad photometry test");
    EXPECT_ANY_THROW(
        sut_.reset(new crf::applications::dsodevel::DsoDevel(
            testDirName_ + "dsoConfigFile_BadVignette.json", cameraMock_)));
}

TEST_F(DsoShould, returnTrueWhenAsyncStartedAndStoppedNormally) {
    createSystemNormally();
    testDebugOutput_->debug("start");
    ASSERT_TRUE(sut_->asyncRunDso(owMock_));
    ASSERT_TRUE(sut_->stopDso());
    ASSERT_TRUE(sut_->asyncRunDso(owMock_));
    ASSERT_TRUE(sut_->stopDso());
    testDebugOutput_->debug("finished");
}

TEST_F(DsoShould, returnFalseWhenStartedWhileAlreadyRunningOrStoppedWhenNotRunning) {
    createSystemNormally();
    ASSERT_FALSE(sut_->stopDso());
    ASSERT_TRUE(sut_->asyncRunDso(owMock_));
    ASSERT_FALSE(sut_->asyncRunDso(owMock_));
    ASSERT_TRUE(sut_->stopDso());
    ASSERT_FALSE(sut_->stopDso());
}


/*
 * POSIX hax to catch SIGINT
 * Just press Ctrl+C to shutDown the performance test
 */

#include <csignal>

namespace {
volatile std::sig_atomic_t exitFlag = 0;
int sleepIntervalMili = 100;
void signal_handler(int signal) {
    exitFlag = 1;
}
}  // unnamed namespace

TEST_F(DsoShould, DISABLED_performanceTestOnline) {
    std::signal(SIGINT, signal_handler);
    ASSERT_NO_THROW(sut_.reset(new crf::applications::dsodevel::DsoDevel(
        testDirName_ + CONFIG_FILE_GOOD, std::make_shared<cv::VideoCapture>(0))));
    ASSERT_TRUE(sut_->asyncRunDso(std::make_shared<dso::IOWrap::PangolinDSOViewer>(640, 480)));
    while (exitFlag == 0) {
        std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(sleepIntervalMili));
    }
    exitFlag = 0;
    ASSERT_TRUE(sut_->stopDso());
}

TEST_F(DsoShould, DISABLED_performanceTestOffline) {
    std::signal(SIGINT, signal_handler);
    // in dsoConfigFile_Offline.json change filepaths to the correct paths of the dataset files
    std::string configFileName(testDirName_ + "dsoConfigFile_Offline.json");
    std::string imgSequence;
    std::ifstream configFile(configFileName);
    ASSERT_TRUE(configFile.is_open());
    nlohmann::json jsonConfig;
    ASSERT_NO_THROW(configFile >> jsonConfig);
    configFile.close();
    ASSERT_NO_THROW(imgSequence = jsonConfig.at("offlineData"));
    ASSERT_NO_THROW(sut_.reset(new crf::applications::dsodevel::DsoDevel(configFileName,
        std::make_shared<cv::VideoCapture>(imgSequence))));
    ASSERT_TRUE(sut_->asyncRunDso(std::make_shared<dso::IOWrap::PangolinDSOViewer>(640, 480)));
    while (exitFlag == 0) {
        std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(sleepIntervalMili));
    }
    exitFlag = 0;
    ASSERT_TRUE(sut_->stopDso());
}
