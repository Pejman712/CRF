/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <condition_variable>
#include <string>
#include <vector>
#include <memory>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"

#include "RobotPoseEstimators/RobotPoseEstimatorTrackingCam.hpp"
#include "RobotPoseEstimators/IRobotPoseEstimator.hpp"

#include "Mocks/Sensors/TrackingCameraMock.hpp"
#include "Mocks/Robots/RobotBaseMock.hpp"

using testing::_;
using testing::Return;
using testing::FloatEq;
using testing::NiceMock;
using testing::Invoke;
using testing::DoDefault;

using crf::utility::types::TaskPose;
using crf::sensors::trackingcamera::TrackingData;
using crf::sensors::trackingcamera::TrackingCameraMock;
using crf::robots::robotbase::RobotBaseMock;
using crf::applications::robotposeestimator::RobotPoseEstimatorTrackingCam;
using crf::applications::robotposeestimator::IRobotPoseEstimator;
using crf::robots::robotbase::RobotBaseConfiguration;


class RobotPoseEstimatorTrackingCamShould: public ::testing::Test {
 public:
    RobotPoseEstimatorTrackingCamShould():
    logger_("RobotPoseEstimatorTrackingCamShould"),
    data_{},
    m_(),
    timeOut_(1),
    robotBaseConfiguration_(new RobotBaseConfiguration()) {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        testFileDirName_ = __FILE__;
        testFileDirName_ = testFileDirName_.substr(0,
            testFileDirName_.find("tests/"));
        testFileDirName_ += "tests/Configurations/Robots/RobotBase/";
        testFileDirName_.append("goodCernBot2Config.json");
        std::ifstream config(testFileDirName_);
        robotBaseConfiguration_->parse(nlohmann::json::parse(config));
        robotBaseMock_.reset(new NiceMock<RobotBaseMock>());
        configureRobotBaseDefaultBehavior();
        trackingCamMock_.reset(new NiceMock<TrackingCameraMock>());
        configureTrackingCamDefaultBehavior();
        sut_.reset(new RobotPoseEstimatorTrackingCam(
            std::make_shared<Kalman::UnscentedKalmanFilter
            <crf::applications::robotposeestimator::SystemState>>(1),
            robotBaseMock_, trackingCamMock_));
    }
    ~RobotPoseEstimatorTrackingCamShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    void configureTrackingCamDefaultBehavior() {
        configureDefaultTrackingData();
        ON_CALL(*trackingCamMock_, initialize()).WillByDefault(Return(true));
        ON_CALL(*trackingCamMock_, deinitialize()).WillByDefault(Return(true));
        ON_CALL(*trackingCamMock_, resetPose()).WillByDefault(Return(true));
        ON_CALL(*trackingCamMock_, feedBaseVelocity(_)).WillByDefault(Invoke([this]
            (const std::array<float, 3>& velocity) {
            std::lock_guard<std::mutex> l(m_);
            writerCv_.notify_all();
            return true;}));
        ON_CALL(*trackingCamMock_, getTrackingData()).WillByDefault(Invoke([this]
            () {return data_;}));
    }
    void configureRobotBaseDefaultBehavior() {
        ON_CALL(*robotBaseMock_, getConfiguration())
            .WillByDefault(Return(robotBaseConfiguration_));
        ON_CALL(*robotBaseMock_, getMotorsVelocities())
            .WillByDefault(Invoke([this]() {
            return boost::optional<std::vector<float>>({0.0, 0.0, 0.0, 0.0});
            }));
    }
    void configureDefaultTrackingData() {
        data_.pose.translation.x = 0.1;
        data_.pose.translation.y = 0.0;
        data_.pose.translation.z = 0.3;
        data_.pose.velocity.x = 0.1;
        data_.pose.velocity.y = 0.7;
        data_.pose.velocity.z = 0.8;
        data_.pose.acceleration.x = 0.01;
        data_.pose.acceleration.y = 0.01;
        data_.pose.acceleration.z = 0.01;
        data_.pose.tracker_confidence = 0x1;
        data_.pose.mapper_confidence = 0x1;
    }
    crf::utility::logger::EventLogger logger_;
    TrackingData data_;
    mutable std::mutex m_;
    std::chrono::milliseconds timeOut_;
    std::condition_variable writerCv_;
    std::string testFileDirName_;
    std::shared_ptr<NiceMock<TrackingCameraMock>> trackingCamMock_;
    std::shared_ptr<RobotBaseConfiguration> robotBaseConfiguration_;
    std::shared_ptr<NiceMock<RobotBaseMock>> robotBaseMock_;
    std::unique_ptr<IRobotPoseEstimator> sut_;
};

TEST_F(RobotPoseEstimatorTrackingCamShould, initializeAndDeinitializeCorrectly) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotPoseEstimatorTrackingCamShould, returnFailureOnFaultyTrackerConfidence) {
    EXPECT_CALL(*trackingCamMock_, getTrackingData()).WillOnce(Invoke([this]() {
        TrackingData data;
        data.pose.tracker_confidence = 0x0;
        data.pose.mapper_confidence = 0x0;
        return data;
    })).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getPosition());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotPoseEstimatorTrackingCamShould, failIfNoTrackingCamera) {
    ASSERT_THROW(sut_.reset(new RobotPoseEstimatorTrackingCam(
            std::make_shared<Kalman::UnscentedKalmanFilter
            <crf::applications::robotposeestimator::SystemState>>(1),
            robotBaseMock_, nullptr)), std::runtime_error);
}

TEST_F(RobotPoseEstimatorTrackingCamShould, returnFalseIfTrackingCamisNotInitialized) {
    EXPECT_CALL(*trackingCamMock_, initialize()).WillOnce(Return(false)).
        WillRepeatedly(DoDefault());
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(RobotPoseEstimatorTrackingCamShould, returnFalseIfTrackingCamPoseIsNotReset) {
    EXPECT_CALL(*trackingCamMock_, resetPose()).WillOnce(Return(false)).
        WillRepeatedly(DoDefault());
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(RobotPoseEstimatorTrackingCamShould, returnFalseIfNotPossibleToDeinitializeCam) {
    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*trackingCamMock_, deinitialize()).WillOnce(Return(false)).
    WillRepeatedly(DoDefault());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(RobotPoseEstimatorTrackingCamShould, returnReceivedVelocityFromEstimator) {
    ASSERT_TRUE(sut_->initialize());
    auto robotVelocityValid = sut_->getVelocity();
    ASSERT_TRUE(robotVelocityValid);
    auto robotVelocity = robotVelocityValid.get();
    {
        std::unique_lock<std::mutex> lk(m_);
        writerCv_.wait_for(lk, timeOut_);
        std::this_thread::sleep_for(timeOut_);
    }
    ASSERT_NEAR(-data_.pose.velocity.z, robotVelocity(0),  1e-4);
}

TEST_F(RobotPoseEstimatorTrackingCamShould, stopIfCamError) {
    EXPECT_CALL(*trackingCamMock_, feedBaseVelocity(_)).WillRepeatedly(Invoke([this]
        (const std::array<float, 3>& velocity) {
        return false;}));
    ASSERT_TRUE(sut_->initialize());
    int threadsToJoinInterval = 50;
    std::this_thread::sleep_for(std::chrono::milliseconds(threadsToJoinInterval));
    ASSERT_FALSE(sut_->getPosition());
    ASSERT_FALSE(sut_->getVelocity());
    ASSERT_FALSE(sut_->getAcceleration());
}
