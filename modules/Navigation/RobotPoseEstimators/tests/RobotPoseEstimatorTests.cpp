/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <mutex>
#include <condition_variable>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"
#include "RobotBase/RobotBaseDefaultKinematics.hpp"
#include "RobotPoseEstimators/RobotPoseEstimator.hpp"
#include "RobotPoseEstimators/IRobotPoseEstimator.hpp"

#include "Mocks/Sensors/IMUMock.hpp"
#include "Mocks/Robots/RobotBaseMock.hpp"

using testing::_;
using testing::Return;
using testing::FloatEq;
using testing::NiceMock;
using testing::Invoke;
using testing::DoDefault;

using crf::sensors::imu::IMUMock;
using crf::sensors::imu::IMUData;
using crf::utility::types::TaskPose;
using crf::robots::robotbase::RobotBaseMock;
using crf::applications::robotposeestimator::IRobotPoseEstimator;
using crf::applications::robotposeestimator::RobotPoseEstimator;
using crf::robots::robotbase::RobotBaseConfiguration;
using crf::robots::robotbase::RobotBaseDefaultKinematics;

class RobotPoseEstimatorShould: public ::testing::Test {
 public:
    RobotPoseEstimatorShould():
    logger_("RobotPoseEstimatorShould"),
    testFileDirName_(),
    measuredMotorVelocity_{.0, .0, .0, .0},
    velocitiesValid_(true),
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
        kinematics_.reset(new RobotBaseDefaultKinematics(*robotBaseConfiguration_));
        robotBaseMock_.reset(new NiceMock<RobotBaseMock>());
        configureRobotBaseDefaultBehavior();
        imuMock_.reset(new NiceMock<IMUMock>());
        configureIMUBehavior();
        ukf = std::make_shared<
            Kalman::UnscentedKalmanFilter<crf::applications::robotposeestimator::SystemState>>(1);
        sut_.reset(new RobotPoseEstimator(
            ukf,
            robotBaseMock_, nullptr));
    }
    ~RobotPoseEstimatorShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    void configureIMUBehavior() {
        ON_CALL(*imuMock_, getIMUData()).WillByDefault(Invoke([this]() {
            std::lock_guard<std::mutex> l(m_);
            imuCv_.notify_all();
            return getRobotsAccelerations();}));
        ON_CALL(*imuMock_, initialize()).WillByDefault(Return(true));
        ON_CALL(*imuMock_, deinitialize()).WillByDefault(Return(true));
    }
    void configureRobotBaseDefaultBehavior() {
        ON_CALL(*robotBaseMock_, getConfiguration())
            .WillByDefault(Return(robotBaseConfiguration_));
        ON_CALL(*robotBaseMock_, getMotorsVelocities())
            .WillByDefault(Invoke([this]() {
            std::lock_guard<std::mutex> l(m_);
            writerCv_.notify_all();
            if (!velocitiesValid_) {
                return boost::optional<std::vector<float>>(boost::none);
            }
            return boost::optional<std::vector<float>>(measuredMotorVelocity_);
            }));
    }
    IMUData getRobotsAccelerations() {
        IMUData data;
        data.acceleration[0] = 3.532;
        data.acceleration[1] = 0.031;
        data.acceleration[2] = 0.98;
        return data;
    }
    crf::utility::logger::EventLogger logger_;
    std::string testFileDirName_;
    std::vector<float> measuredMotorVelocity_;
    bool velocitiesValid_;
    mutable std::mutex m_;
    std::chrono::milliseconds timeOut_;
    std::condition_variable writerCv_;
    std::condition_variable imuCv_;
    std::shared_ptr<NiceMock<IMUMock>> imuMock_;
    std::shared_ptr<RobotBaseDefaultKinematics> kinematics_;
    std::shared_ptr<RobotBaseConfiguration> robotBaseConfiguration_;
    std::shared_ptr<NiceMock<RobotBaseMock>> robotBaseMock_;
    std::shared_ptr<Kalman::UnscentedKalmanFilter<
        crf::applications::robotposeestimator::SystemState>> ukf;
    std::unique_ptr<IRobotPoseEstimator> sut_;
};

TEST_F(RobotPoseEstimatorShould, initializedeinitializeCorrectly) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotPoseEstimatorShould, returnNoResultIfNotInitialized) {
    ASSERT_FALSE(sut_->getPosition());
    ASSERT_FALSE(sut_->getVelocity());
    ASSERT_FALSE(sut_->getAcceleration());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(RobotPoseEstimatorShould, computeZeroVelocitiesOnInvalidMotorsVelocitiesSize) {
    measuredMotorVelocity_.clear();
    measuredMotorVelocity_ = {.0 , .0};
    ASSERT_TRUE(sut_->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        writerCv_.wait_for(lk, timeOut_);
    }
    std::this_thread::sleep_for(
            std::chrono::microseconds(robotBaseConfiguration_->getRTLoopTime()));
    auto posValidity = sut_->getPosition();
    ASSERT_FALSE(posValidity);
}

TEST_F(RobotPoseEstimatorShould, computeZeroVelocitiesOnInvalidMotorsVelocities) {
    velocitiesValid_ = false;
    ASSERT_TRUE(sut_->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        writerCv_.wait_for(lk, timeOut_);
    }
    std::this_thread::sleep_for(
            std::chrono::microseconds(robotBaseConfiguration_->getRTLoopTime()));
    auto velValidity = sut_->getVelocity();
    ASSERT_FALSE(velValidity);
}

// Test disabled due to a constant file. Needs to be checked
TEST_F(RobotPoseEstimatorShould, DISABLED_returnValidEstimate) {
    measuredMotorVelocity_.clear();
    auto measuredMotorVelocityValid = kinematics_->getWheelsVelocity({0.0, -0.2, .0, .0, .0, .0});
    ASSERT_TRUE(measuredMotorVelocityValid);
    measuredMotorVelocity_ = measuredMotorVelocityValid.get();
    auto kinematicsValid = kinematics_->getTaskVelocity(measuredMotorVelocity_);
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(kinematicsValid);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // put here a delay if you want to test the algorithm and validate position with NOLINT
    {
        std::unique_lock<std::mutex> lk(m_);
        writerCv_.wait_for(lk, timeOut_);
    }
    auto posValidity = sut_->getPosition();
    auto velValidity = sut_->getVelocity();
    ASSERT_TRUE(sut_->getAcceleration());
    ASSERT_TRUE(posValidity);
    ASSERT_TRUE(velValidity);
    float delay = 0.1;
    uint8_t dimension = 1;
    ASSERT_NEAR(velValidity.get()(dimension), kinematicsValid.get()(dimension), 1e-3);
    ASSERT_NEAR(posValidity.get()(dimension), kinematicsValid.get()(dimension)*delay, 1e-2);
}

TEST_F(RobotPoseEstimatorShould, behaveSameIfIMU) {
    sut_.reset(new RobotPoseEstimator(ukf, robotBaseMock_, imuMock_));
    ASSERT_TRUE(sut_->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        writerCv_.wait_for(lk, timeOut_);
    }
    {
        std::unique_lock<std::mutex> lk(m_);
        imuCv_.wait_for(lk, timeOut_);
    }
    auto velValidity = sut_->getVelocity();
    ASSERT_TRUE(velValidity);
    ASSERT_NEAR(velValidity.get()(0), 0.0, 1e-2);
    ASSERT_NEAR(velValidity.get()(1), 0.0, 1e-2);
    std::this_thread::sleep_for(timeOut_);
    ASSERT_TRUE(sut_->getAcceleration());
    auto posValidity = sut_->getPosition();
    ASSERT_NEAR(posValidity.get()(0), .0, 1e-1);
}

TEST_F(RobotPoseEstimatorShould, failOnIMUnotInitializable) {
    sut_.reset(new RobotPoseEstimator(ukf, robotBaseMock_, imuMock_));
    EXPECT_CALL(*imuMock_, initialize()).WillOnce(Return(false)).WillRepeatedly(DoDefault());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->getPosition());
    ASSERT_FALSE(sut_->getVelocity());
    ASSERT_FALSE(sut_->getAcceleration());
}
