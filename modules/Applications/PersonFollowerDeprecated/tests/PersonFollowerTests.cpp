/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <chrono>
#include <condition_variable>

#include "EventLogger/EventLogger.hpp"
#include "Mocks/Applications/RobotBaseControllerMock.hpp"
#include "Mocks/Applications/PersonTrackerMock.hpp"
#include "Mocks/Applications/WallDetectorMock.hpp"
#include "Mocks/Communication/IpcMock.hpp"
#include "LaserCommunicationPoint/LaserPacket.hpp"
#include "PersonFollower/PersonFollower.hpp"
#include "Types/Types.hpp"

using crf::applications::robotbasecontroller::RobotBaseControllerMock;

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::Throw;
using testing::SetArgReferee;

using namespace std::chrono_literals;  // NOLINT

class PersonFollowerShould: public ::testing::Test {
 protected:
    PersonFollowerShould():
        logger_("PersonFollowerShould"),
        wallVectorMock_() {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        robotBaseControllerMock_.reset(new NiceMock<RobotBaseControllerMock>);
        personTrackerMock_.reset(
            new NiceMock<crf::applications::personfollower::PersonTrackerMock>);
        wallDetectorMock_.reset(new NiceMock<crf::applications::walldetector::WallDetectorMock>);
        IpcMock_.reset(new NiceMock<IpcMock>);
        samplePointCloudPtr_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cameraPose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        writerOpened_ = false;
        ON_CALL(*personTrackerMock_, initialize()).WillByDefault(
          ::testing::Return(true));
        ON_CALL(*personTrackerMock_, getCameraDisplacement()).WillByDefault(
          ::testing::Return(15));
        ON_CALL(*robotBaseControllerMock_, initialize()).WillByDefault(
          ::testing::Return(true));
        ON_CALL(*robotBaseControllerMock_, setVelocity(_)).WillByDefault(
          ::testing::Return(true));
        returnVelocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        ON_CALL(*robotBaseControllerMock_, getVelocity()).WillByDefault(
          ::testing::Return(returnVelocity_));
        ON_CALL(*IpcMock_, open()).WillByDefault(Invoke(
            [this]() {
                if (writerOpened_) return false;
                writerOpened_ = true;
                return true;
            }));
        ON_CALL(*IpcMock_, close()).WillByDefault(Invoke(
            [this]() {
                if (!writerOpened_) return false;
                writerOpened_ = false;
                return true;
            }));
        ON_CALL(*IpcMock_, read(_, _)).WillByDefault(Invoke(
            [this](std::string& buf, Packets::PacketHeader& header) {
              Packets::LaserPacket laserPacketMock;
              for (size_t i = 0; i < 100; i++) {
                  pcl::PointXYZRGBA temporaryPoint;
                  temporaryPoint.x = 0.01*i;
                  temporaryPoint.y = 0.01*i*0.5;
                  temporaryPoint.z = (rand() + 1) / (static_cast<float>(RAND_MAX)*100); // NOLINT
                  samplePointCloudPtr_->points.push_back(temporaryPoint);
              }
              laserPacketMock.pointCloud = *samplePointCloudPtr_;
              buf = laserPacketMock.serialize();
              header = laserPacketMock.getHeader();
              return true;
        }));
        timeOut_ = 10ms;
    }

    ~PersonFollowerShould() {
        logger_->info("{} END with {}",
          testing::UnitTest::GetInstance()->current_test_info()->name(),
          testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::vector<crf::applications::walldetector::WallParameter> wallVectorMock_;
    std::shared_ptr<RobotBaseControllerMock> robotBaseControllerMock_;
    std::shared_ptr<crf::applications::personfollower::PersonTrackerMock> personTrackerMock_;
    std::shared_ptr<IpcMock> IpcMock_;
    std::shared_ptr<crf::applications::walldetector::WallDetectorMock> wallDetectorMock_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr samplePointCloudPtr_;
    crf::utility::types::TaskPose cameraPose_;
    bool writerOpened_;
    crf::utility::types::TaskVelocity returnVelocity_;
    std::chrono::milliseconds timeOut_;
    std::unique_ptr<crf::applications::personfollower::PersonFollower> sut_;
    crf::utility::types::TaskVelocity receivedVelocity_;
    std::condition_variable cv_;
    std::mutex m_;
    crf::applications::personfollower::PersonCentroid personCentroidMock_;
};

TEST_F(PersonFollowerShould, NotInitializeIfPersonTrackerCannotBeStarted) {
    EXPECT_CALL(*personTrackerMock_, initialize()).WillOnce(::testing::Return(false));
    sut_.reset(new crf::applications::personfollower::PersonFollower(
      personTrackerMock_, robotBaseControllerMock_, cameraPose_, IpcMock_, wallDetectorMock_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(PersonFollowerShould, NotInitializeIfIpcCannotBeOpened) {
    EXPECT_CALL(*IpcMock_, open()).WillOnce(::testing::Return(false));
    sut_.reset(new crf::applications::personfollower::PersonFollower(
      personTrackerMock_, robotBaseControllerMock_, cameraPose_, IpcMock_, wallDetectorMock_));
    ASSERT_FALSE(sut_->initialize());
}


TEST_F(PersonFollowerShould, NotInitializeIfRobotBaseControllerCannotBeStarted) {
    EXPECT_CALL(*robotBaseControllerMock_, initialize()).WillOnce(::testing::Return(false));
    sut_.reset(new crf::applications::personfollower::PersonFollower(
      personTrackerMock_, robotBaseControllerMock_, cameraPose_, IpcMock_, wallDetectorMock_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(PersonFollowerShould, ReturnFalseIfAlreadyRunningAndShouldNotStartTwice) {
    sut_.reset(new crf::applications::personfollower::PersonFollower(
      personTrackerMock_, robotBaseControllerMock_, cameraPose_, IpcMock_, wallDetectorMock_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(PersonFollowerShould, SetVelocityZeroIfPersonNotDetectedDuringCalibration) {
    EXPECT_CALL(*personTrackerMock_, getCameraDisplacement()).WillRepeatedly(
      ::testing::Return(666));
    EXPECT_CALL(*robotBaseControllerMock_, setVelocity(_)).WillRepeatedly(Invoke(
        [this](const crf::utility::types::TaskVelocity& targetVelocity){
        receivedVelocity_ = targetVelocity;
        return true;
    }));
    EXPECT_CALL(*robotBaseControllerMock_, getVelocity()).WillRepeatedly(Invoke(
          [this](){
          cv_.notify_one();
          return receivedVelocity_;
    }));
    returnVelocity_ = {0.0, 0.0, .0, .0, .0, .0};
    sut_.reset(new crf::applications::personfollower::PersonFollower(
      personTrackerMock_, robotBaseControllerMock_, cameraPose_, IpcMock_, wallDetectorMock_));
    ASSERT_TRUE(sut_->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(crf::utility::types::areAlmostEqual(receivedVelocity_, returnVelocity_));
}

TEST_F(PersonFollowerShould, RotateBaseIfAdjustingFrame) {
    EXPECT_CALL(*personTrackerMock_, getCameraDisplacement()).WillRepeatedly(
      ::testing::Return(150));
    EXPECT_CALL(*robotBaseControllerMock_, setVelocity(_)).WillRepeatedly(Invoke(
        [this](const crf::utility::types::TaskVelocity& targetVelocity){
        receivedVelocity_ = targetVelocity;
        return true;
    }));
    returnVelocity_ = {0.0, 0.0, .0, .0, .0, .05};
    EXPECT_CALL(*robotBaseControllerMock_,
      getVelocity()).WillRepeatedly(Invoke(
          [this](){
          cv_.notify_one();
          return receivedVelocity_;
      }));
    sut_.reset(new crf::applications::personfollower::PersonFollower(
      personTrackerMock_, robotBaseControllerMock_, cameraPose_, IpcMock_, wallDetectorMock_));
    ASSERT_TRUE(sut_->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(crf::utility::types::areAlmostEqual(receivedVelocity_, returnVelocity_));
}

TEST_F(PersonFollowerShould, SetVelocityZeroIfFrameIsJustAdjusted) {
    EXPECT_CALL(*personTrackerMock_, getCameraDisplacement()).WillRepeatedly(::testing::Return(15));
    EXPECT_CALL(*robotBaseControllerMock_, setVelocity(_)).WillOnce(Invoke(
        [this](const crf::utility::types::TaskVelocity& targetVelocity){
        receivedVelocity_ = targetVelocity;
        return true;
    })).WillRepeatedly(::testing::Return(true));
    returnVelocity_ = {0.0, 0.0, .0, .0, .0, .0};
    EXPECT_CALL(*robotBaseControllerMock_,
      getVelocity()).WillOnce(Invoke(
          [this](){
          cv_.notify_one();
          return receivedVelocity_;
      })).WillRepeatedly(::testing::Return(returnVelocity_));
    sut_.reset(new crf::applications::personfollower::PersonFollower(
      personTrackerMock_, robotBaseControllerMock_, cameraPose_, IpcMock_, wallDetectorMock_));
    ASSERT_TRUE(sut_->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(crf::utility::types::areAlmostEqual(receivedVelocity_, returnVelocity_));
}

TEST_F(PersonFollowerShould, SetVelocityZeroIfPersonLostInPointCloud) {
    EXPECT_CALL(*robotBaseControllerMock_, setVelocity(_)).WillOnce(
      ::testing::Return(true)).WillOnce(Invoke(
        [this](const crf::utility::types::TaskVelocity& targetVelocity){
        receivedVelocity_ = targetVelocity;
        cv_.notify_one();
        return true;
    })).WillRepeatedly(::testing::Return(true));
    returnVelocity_ = {0.0, 0.0, .0, .0, .0, .0};
    EXPECT_CALL(*robotBaseControllerMock_,
      getVelocity()).WillRepeatedly(Invoke(
          [this](){
          return receivedVelocity_;
      }));
    EXPECT_CALL(
      *personTrackerMock_, trackPerson(_)).WillRepeatedly(::testing::Return(personCentroidMock_));
    personCentroidMock_.objectID = -1;
    sut_.reset(new crf::applications::personfollower::PersonFollower(
      personTrackerMock_, robotBaseControllerMock_, cameraPose_, IpcMock_, wallDetectorMock_));
    ASSERT_TRUE(sut_->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(crf::utility::types::areAlmostEqual(receivedVelocity_, returnVelocity_));
}

TEST_F(PersonFollowerShould, ComeCloserIfPersonIsTooFar) {
    personCentroidMock_.objectID = 1;
    personCentroidMock_.euclideanDistance = 6;
    crf::applications::walldetector::WallParameter wallDataMock;
    wallDataMock.theta = 1;
    wallVectorMock_.push_back(wallDataMock);
    EXPECT_CALL(*wallDetectorMock_, detectWall(_, _)).WillRepeatedly(
      ::testing::Return(wallVectorMock_));
    EXPECT_CALL(*robotBaseControllerMock_, setVelocity(_)).WillOnce(
      ::testing::Return(true)).WillOnce(Invoke(
        [this](const crf::utility::types::TaskVelocity& targetVelocity){
        receivedVelocity_ = targetVelocity;
        cv_.notify_one();
        return true;
    })).WillRepeatedly(::testing::Return(true));
    returnVelocity_ = {-0.2, 0.0, .0, .0, .0, -.05};
    EXPECT_CALL(*robotBaseControllerMock_,
      getVelocity()).WillRepeatedly(Invoke(
          [this](){
          return receivedVelocity_;
      }));
    EXPECT_CALL(
      *personTrackerMock_, trackPerson(_)).WillRepeatedly(::testing::Return(personCentroidMock_));
    sut_.reset(new crf::applications::personfollower::PersonFollower(
      personTrackerMock_, robotBaseControllerMock_, cameraPose_, IpcMock_, wallDetectorMock_));
    ASSERT_TRUE(sut_->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(crf::utility::types::areAlmostEqual(receivedVelocity_, returnVelocity_));
}

TEST_F(PersonFollowerShould, GoBackIfPersonIsTooClose) {
    personCentroidMock_.objectID = 1;
    personCentroidMock_.euclideanDistance = 4;
    EXPECT_CALL(*wallDetectorMock_, detectWall(_, _)).WillRepeatedly(
      ::testing::Return(wallVectorMock_));
    EXPECT_CALL(*robotBaseControllerMock_, setVelocity(_)).WillOnce(
      ::testing::Return(true)).WillOnce(Invoke(
        [this](const crf::utility::types::TaskVelocity& targetVelocity){
        receivedVelocity_ = targetVelocity;
        cv_.notify_one();
        return true;
    })).WillRepeatedly(::testing::Return(true));
    returnVelocity_ = {0.2, 0.0, .0, .0, .0, .0};
    EXPECT_CALL(*robotBaseControllerMock_,
      getVelocity()).WillRepeatedly(Invoke(
          [this](){
          return receivedVelocity_;
      }));
    EXPECT_CALL(
      *personTrackerMock_, trackPerson(_)).WillRepeatedly(::testing::Return(personCentroidMock_));
      sut_.reset(new crf::applications::personfollower::PersonFollower(
        personTrackerMock_, robotBaseControllerMock_, cameraPose_, IpcMock_, wallDetectorMock_));
    ASSERT_TRUE(sut_->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(crf::utility::types::areAlmostEqual(receivedVelocity_, returnVelocity_));
}
