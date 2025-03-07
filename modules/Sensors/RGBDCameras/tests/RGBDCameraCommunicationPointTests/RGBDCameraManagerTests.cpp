/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */


#include <future>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraManager.hpp"
#include "VisionUtility/Image/ImageJSONConverter.hpp"
#include "RGBDCameras/RGBDCameraMockConfiguration.hpp"
#include "EventLogger/EventLogger.hpp"
#include "RGBDCameras/RGBDCameraMock.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::sensors::rgbdcameras::RGBDCameraMock;
using crf::sensors::rgbdcameras::RGBDCameraMockConfiguration;
using crf::sensors::rgbdcameras::RGBDCameraManager;
using crf::sensors::rgbdcameras::RGBDPointCloud;

class RGBDCameraManagerShould : public ::testing::Test {
 protected:
    RGBDCameraManagerShould() :
        logger_("RGBDCameraManagerShould") {
        logger_->info("{0} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    void SetUp() {
        mock_ = std::make_shared<NiceMock<RGBDCameraMockConfiguration>>();
        sut_ = std::make_unique<RGBDCameraManager>(mock_, std::chrono::seconds(3));
    }

    ~RGBDCameraManagerShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<NiceMock<RGBDCameraMockConfiguration>> mock_;
    std::unique_ptr<RGBDCameraManager> sut_;
};

TEST_F(RGBDCameraManagerShould, tryAllOperations) {
    sut_->requestFrameStream(
        0, Profile(cv::Size(1280, 720), 30), Profile(cv::Size(640, 480), 30), false);
    auto sts = sut_->getStatus();
    auto frame = sut_->getFrame(0, std::chrono::milliseconds(1000));
}

TEST_F(RGBDCameraManagerShould, getFrameFailsIfNotRequestedBefore) {
    ASSERT_TRUE(sut_->getFrame(0).empty());
}

TEST_F(RGBDCameraManagerShould, getFrameFailsIfRequestedSocketIsNotActive) {
    sut_->requestFrameStream(
        0, Profile(cv::Size(1280, 720), 30), Profile(cv::Size(640, 480), 30), false);
    ASSERT_THROW(sut_->getFrame(1), std::out_of_range);
}


TEST_F(RGBDCameraManagerShould, cantRequestResolutionWithOneDimensionEqualTo0) {
    ASSERT_FALSE(sut_->requestFrameStream(
        0, Profile(cv::Size(0, 720), 30), Profile(cv::Size(640, 480), 30), false));
    ASSERT_FALSE(sut_->requestFrameStream(
        0, Profile(cv::Size(1280, 0), 30), Profile(cv::Size(640, 480), 30), false));
    ASSERT_FALSE(sut_->requestFrameStream(
        0, Profile(cv::Size(1280, 720), 30), Profile(cv::Size(0, 480), 30), false));
    ASSERT_FALSE(sut_->requestFrameStream(
        0, Profile(cv::Size(1280, 720), 30), Profile(cv::Size(640, 0), 30), false));
}


TEST_F(RGBDCameraManagerShould, testGetPointCloud) {
    sut_->requestFrameStream(
        0, Profile(cv::Size(1280, 720), 30), Profile(cv::Size(640, 480), 30), true);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl = sut_->getPointCloud(0, std::chrono::milliseconds(10000));
    // EXPECT_NE(pcl->size(), 0);
}

TEST_F(RGBDCameraManagerShould, testGetRGBDFrameAndPointCloud) {
    sut_->requestFrameStream(
        0, Profile(cv::Size(1280, 720), 30), Profile(cv::Size(640, 480), 30), true);
    RGBDPointCloud rgbdp;
    while (rgbdp.frame.image.empty())
        rgbdp = sut_->getRGBDFrameAndPointCloud(0, std::chrono::milliseconds(10000));

    auto rgbdframe = rgbdp.frame;
    ASSERT_EQ(rgbdframe.image.rows, 720);
    ASSERT_EQ(rgbdframe.image.cols, 1280);
    ASSERT_EQ(rgbdframe.depth.rows, 480);
    ASSERT_EQ(rgbdframe.depth.cols, 640);
}

TEST_F(RGBDCameraManagerShould, correctlyGetFrameAndUpdateResolution) {
    ASSERT_TRUE(sut_->requestFrameStream(
        0, Profile(cv::Size(1280, 720), 30), Profile(cv::Size(1280, 720), 30), false));
    cv::Mat mat1;
    while (mat1.empty()) mat1 = sut_->getFrame(0);
    cv::rgbd::RgbdFrame rgbd1;
    while (rgbd1.image.empty()) rgbd1 = sut_->getRGBDFrame(0);

    ASSERT_TRUE(sut_->requestFrameStream(
        0, Profile(cv::Size(640, 480), 30), Profile(cv::Size(640, 480), 30), false));
    cv::Mat mat2;
    while (mat2.empty()) mat2 = sut_->getFrame(0);
    cv::rgbd::RgbdFrame rgbd2;
    while (rgbd2.image.empty()) rgbd2 = sut_->getRGBDFrame(0);

    ASSERT_EQ(mat1.rows, 720);
    ASSERT_EQ(mat1.cols, 1280);

    ASSERT_EQ(rgbd1.image.rows, 720);
    ASSERT_EQ(rgbd1.image.cols, 1280);
    ASSERT_EQ(rgbd1.depth.rows, 720);
    ASSERT_EQ(rgbd1.depth.cols, 1280);

    ASSERT_EQ(mat2.rows, 480);
    ASSERT_EQ(mat2.cols, 640);

    ASSERT_EQ(rgbd2.image.rows, 480);
    ASSERT_EQ(rgbd2.image.cols, 640);
    ASSERT_EQ(rgbd2.depth.rows, 480);
    ASSERT_EQ(rgbd2.depth.cols, 640);
}

TEST_F(RGBDCameraManagerShould, requestTwoFramesAtTheSameTime) {
    ASSERT_TRUE(sut_->requestFrameStream(
        0, Profile(cv::Size(1280, 720), 30), Profile(cv::Size(640, 480), 30), false));
    cv::Mat firstFrame;
    while (firstFrame.empty()) firstFrame = sut_->getFrame(0);

    ASSERT_TRUE(sut_->requestFrameStream(
        1, Profile(cv::Size(1920, 1080), 30), Profile(cv::Size(1920, 1080), 30), false));

    cv::rgbd::RgbdFrame secondFrame;
    while (secondFrame.image.empty()) secondFrame = sut_->getRGBDFrame(1);

    auto mat1 = firstFrame;
    auto mat2 = secondFrame.depth;

    ASSERT_EQ(mat1.rows, 720);
    ASSERT_EQ(mat1.cols, 1280);
    ASSERT_EQ(mat2.rows, 1080);
    ASSERT_EQ(mat2.cols, 1920);
}

TEST_F(RGBDCameraManagerShould, correctlyGetSetParameters) {
    ASSERT_TRUE(sut_->requestFrameStream(
        0, Profile(cv::Size(1280, 720), 30), Profile(cv::Size(640, 480), 30), false));
    auto parameters = sut_->getStatus();
}
