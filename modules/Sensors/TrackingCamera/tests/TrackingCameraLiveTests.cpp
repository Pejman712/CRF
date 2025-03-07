/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gtest/gtest.h>

#include <memory>
#include <vector>
#include <string>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "nlohmann/json.hpp"

#include "EventLogger/EventLogger.hpp"

#include "TrackingCamera/ITrackingCamera.hpp"
#include "TrackingCamera/IntelT265Grabber.hpp"

/*
 * Mocking the realsense API is not feasible for now
 * For this reason we provide only tests that are to be executed on the
 * connected device and are therefore disabled by default
*/

using crf::sensors::trackingcamera::TrackingData;
using crf::sensors::trackingcamera::IntelT265Grabber;
using crf::sensors::trackingcamera::ITrackingCamera;

class IntelT265GrabberLiveShould: public ::testing::Test {
 protected:
    IntelT265GrabberLiveShould(): logger_("IntelT265GrabberLiveShould") {
        std::string testAddress_ = __FILE__;
        testAddress_ = testAddress_.substr(0, testAddress_.find(
            "tests/TrackingCameraTests/TrackingCameraLiveTests.cpp"));
        testAddress_ += "modules/Sensors/TrackingCamera/config/realsenseT265.json";
        std::string configFile(testAddress_);
        std::ifstream config(configFile);
        config >> jConfig;
        sut_.reset(new IntelT265Grabber(jConfig.at("RealSense")));
    }
    crf::utility::logger::EventLogger logger_;
    nlohmann::json jConfig;
    std::unique_ptr<ITrackingCamera> sut_;
};



TEST_F(IntelT265GrabberLiveShould, DISABLED_returnTrueIfInitializeAndDeinitializeCorrectly) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    usleep(50000);
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(IntelT265GrabberLiveShould, DISABLED_streamUpdatedTrackingDatafromOnlineDevice) {
    ASSERT_TRUE(sut_->initialize());
    TrackingData data;
    for (int i = 0; i < 50; i++) {
        data = sut_->getTrackingData();
        logger_->info("pose: {}, {}, {}", data.pose.translation.x,
                      data.pose.translation.y,
 data.pose.translation.z);
        usleep(30000);
    }
}

TEST_F(IntelT265GrabberLiveShould, DISABLED_resetTrackingDataifTrackingCamWasResetCorrectly) {
    ASSERT_TRUE(sut_->initialize());
    // Move camera a bit
    usleep(50000);
    ASSERT_TRUE(sut_->resetPose());
    TrackingData data = sut_->getTrackingData();
    ASSERT_NEAR(data.pose.translation.x, 0.000, 0.001);
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->resetPose());
}


TEST_F(IntelT265GrabberLiveShould, DISABLED_returnCVFramesWhenFishEyeStreamsEnabled) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_NO_THROW(sut_->getVideoFrames());
    std::vector<cv::Mat> currentFrames = sut_->getVideoFrames();
    ASSERT_FALSE(currentFrames.empty());
    ASSERT_EQ(currentFrames.size(), 2);
    cv::Mat leftFishEyeFrame = currentFrames[0];
    cv::imshow("leftFishEyeFrame", leftFishEyeFrame);
    cv::waitKey();
    cv::Mat rightFishEyeFrame = currentFrames[1];
    cv::imshow("rightFishEyeFrame", rightFishEyeFrame);
    cv::waitKey();
    ASSERT_TRUE(sut_->deinitialize());
}


TEST_F(IntelT265GrabberLiveShould, DISABLED_notThrowExceptionIfBaseVelocitySetAfterCorrectInit) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_NO_THROW(sut_->feedBaseVelocity({4.0994, 0.03, 0.-001}));
    ASSERT_TRUE(sut_->deinitialize());
}


