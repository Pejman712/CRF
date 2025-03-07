/* Â© Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>

#include "PersonFollower/PersonTracker.hpp"
#include "LaserCommunicationPoint/LaserPacket.hpp"
#include "Types/TaskTypes/TaskPose.hpp"

#include "Mocks/Communication/IpcMock.hpp"
#include "Mocks/Applications/PersonDetectorMock.hpp"
#include "Mocks/Utility/DummyPacket.hpp"

#define IMAGE_CENTER_WIDTH 320

using crf::robots::robotbase::RobotBaseConfiguration;
using crf::applications::personfollower::PersonDetectorMock;
using crf::applications::personfollower::PersonTracker;
using crf::applications::personfollower::PersonCentroid;

using ::testing::_;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Throw;

class PersonTrackerShould: public ::testing::Test {
 protected:
    PersonTrackerShould(): logger_("PersonTrackerShould"),
        testAddress_(__FILE__),
        edgeDataReference_(),
        clusterBinsReference_(),
        robotBaseConfiguration_(new RobotBaseConfiguration),
        outputIpcMock_(new NiceMock<IpcMock>),
        PersonDetectorMock_(new NiceMock<PersonDetectorMock>),
        mockPointCloudData_(new pcl::PointCloud<pcl::PointXYZRGBA>),
        cameraPose_({0.1, 0.1, 0.2, 0, 0, 0}) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        testAddress_ = testAddress_.substr(0, testAddress_.find("PersonTrackerTests.cpp"));
        std::ifstream param(testAddress_ + "./config/HokuyoLaserParameters.json");
        param >> parameters_;
        ON_CALL(*PersonDetectorMock_, initialize()).WillByDefault(::testing::Return(true));
        ON_CALL(*PersonDetectorMock_,
          getImageCenterWidth()).WillByDefault(::testing::Return(IMAGE_CENTER_WIDTH));
        ON_CALL(*outputIpcMock_, open()).WillByDefault(::testing::Return(true));
        ON_CALL(*outputIpcMock_, close()).WillByDefault(::testing::Return(true));
        ON_CALL(*outputIpcMock_, read(_, _)).WillByDefault(Invoke(
            [this](std::string& bytes, Packets::PacketHeader& header) {
                Packets::LaserPacket packet;
                packet.pointCloud = *mockPointCloudData_;
                bytes = packet.serialize();
                header = packet.getHeader();
                return true;
            }));
    }
    bool assignDummyPacketToReader(std::string& bytes, Packets::PacketHeader& header) {  // NOLINT
        Packets::DummyPacket packet{};
        bytes = packet.serialize();
        header = packet.getHeader();
        return true;
    }

    ~PersonTrackerShould() {
        logger_->info("{} END with {}",
          testing::UnitTest::GetInstance()->current_test_info()->name(),
          testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        std::string configFileName = __FILE__;
        configFileName = configFileName.substr(0,
            configFileName.find("tests/"));
        configFileName += "tests/Configurations/Robots/PersonFollowerTests/";
        ASSERT_NO_THROW(robotBaseConfiguration_->parse(configFileName));
        logger_->info("test SetUp done");
    }

    void importMockData(std::string fileName) {
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (
          testAddress_+ "hokuyoDataExamples/" + fileName, *mockPointCloudData_) == -1) {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
            throw std::runtime_error("Failed to open config file.");
            return;
        }
        return;
    }

    crf::utility::logger::EventLogger logger_;
    std::string testAddress_;
    nlohmann::json parameters_;
    std::vector<int> edgeDataReference_;
    std::vector<std::vector<int> > clusterBinsReference_;
    std::unique_ptr<PersonTracker> sut_;
    std::shared_ptr<RobotBaseConfiguration> robotBaseConfiguration_;
    std::shared_ptr<NiceMock<IpcMock> > outputIpcMock_;
    std::shared_ptr<PersonDetectorMock> PersonDetectorMock_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mockPointCloudData_;
    crf::utility::types::TaskPose cameraPose_;
};

TEST_F(PersonTrackerShould, ReturnFalseIfAlreadyRunningAndShouldNotStartTwice) {
    sut_.reset(
      new PersonTracker(outputIpcMock_, PersonDetectorMock_, parameters_, robotBaseConfiguration_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(PersonTrackerShould, NotInitializeIfPersonDetectorCannotBeStarted) {
    EXPECT_CALL(*PersonDetectorMock_, initialize()).WillOnce(::testing::Return(false));
    sut_.reset(new PersonTracker(
      outputIpcMock_, PersonDetectorMock_, parameters_, robotBaseConfiguration_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(PersonTrackerShould, NotInitializeIfIpcCannotBeStarted) {
    EXPECT_CALL(*outputIpcMock_, open()).WillOnce(::testing::Return(false));
    sut_.reset(new PersonTracker(
      outputIpcMock_, PersonDetectorMock_, parameters_, robotBaseConfiguration_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(PersonTrackerShould, ReturnNumberOfTheBeastIfNoBoxesPresent) {
    cv::Rect2d personOnLeftSide;
    EXPECT_CALL(*PersonDetectorMock_,
      getPersonBoundingBox()).WillOnce(::testing::Return(personOnLeftSide));
    sut_.reset(new PersonTracker(
      outputIpcMock_, PersonDetectorMock_, parameters_, robotBaseConfiguration_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(sut_->getCameraDisplacement(), 666);
}

TEST_F(PersonTrackerShould, MoveCameraLeft) {
    cv::Rect2d personOnLeftSide(0, 0, 50, 200);
    EXPECT_CALL(*PersonDetectorMock_,
      getPersonBoundingBox()).WillOnce(::testing::Return(personOnLeftSide));
    sut_.reset(new PersonTracker(
      outputIpcMock_, PersonDetectorMock_, parameters_, robotBaseConfiguration_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(sut_->getCameraDisplacement(), -295);
}

TEST_F(PersonTrackerShould, MoveCameraRight) {
    cv::Rect2d personOnRightSide(400, 50, 50, 200);
    EXPECT_CALL(*PersonDetectorMock_,
      getPersonBoundingBox()).WillOnce(::testing::Return(personOnRightSide));
    sut_.reset(new PersonTracker(
      outputIpcMock_, PersonDetectorMock_, parameters_, robotBaseConfiguration_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(sut_->getCameraDisplacement(), 105);
}

TEST_F(PersonTrackerShould, detectOneCetroidAndInitializePersonPoint) {
    cv::Rect2d personCentered(300, 50, 40, 200);
    importMockData("1mopen.pcd");
    EXPECT_CALL(*PersonDetectorMock_,
      getPersonBoundingBox()).WillOnce(::testing::Return(personCentered));
    sut_.reset(new PersonTracker(
      outputIpcMock_, PersonDetectorMock_, parameters_, robotBaseConfiguration_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(sut_->getCameraDisplacement(), 0);
    PersonCentroid personPoint = sut_->trackPerson(cameraPose_);
    ASSERT_EQ(personPoint.objectID, 1);
}

TEST_F(PersonTrackerShould, notFindAnyCentroids) {
    cv::Rect2d personCentered(300, 50, 40, 200);
    importMockData("emptyPCD.pcd");
    EXPECT_CALL(*PersonDetectorMock_,
      getPersonBoundingBox()).WillOnce(::testing::Return(personCentered));
    sut_.reset(new PersonTracker(
      outputIpcMock_, PersonDetectorMock_, parameters_, robotBaseConfiguration_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(sut_->getCameraDisplacement(), 0);
    PersonCentroid personPoint = sut_->trackPerson(cameraPose_);
    ASSERT_EQ(personPoint.objectID, -1);
}

TEST_F(PersonTrackerShould, redefinePersonLocationWithMostClosestPoint) {
    cv::Rect2d personCentered(300, 50, 40, 200);
    EXPECT_CALL(*PersonDetectorMock_,
      getPersonBoundingBox()).WillOnce(::testing::Return(personCentered));
    importMockData("1mopen.pcd");
    sut_.reset(new PersonTracker(
      outputIpcMock_, PersonDetectorMock_, parameters_, robotBaseConfiguration_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(sut_->getCameraDisplacement(), 0);
    PersonCentroid personPoint = sut_->trackPerson(cameraPose_);
    ASSERT_EQ(personPoint.objectID, 1);
    importMockData("1mstep.pcd");
    personPoint = sut_->trackPerson(cameraPose_);
    ASSERT_EQ(personPoint.dissapeared, 0);
}

TEST_F(PersonTrackerShould, neglectDiscoveredCentroidsBecauseTheyAreTooFar) {
    cv::Rect2d personCentered(300, 50, 40, 200);
    EXPECT_CALL(*PersonDetectorMock_,
      getPersonBoundingBox()).WillOnce(::testing::Return(personCentered));
    importMockData("1mopen.pcd");
    sut_.reset(new PersonTracker(
      outputIpcMock_, PersonDetectorMock_, parameters_, robotBaseConfiguration_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(sut_->getCameraDisplacement(), 0);
    PersonCentroid personPoint = sut_->trackPerson(cameraPose_);
    ASSERT_EQ(personPoint.objectID, 1);
    importMockData("3mopen.pcd");
    personPoint = sut_->trackPerson(cameraPose_);
    ASSERT_EQ(personPoint.dissapeared, 1);
}

TEST_F(PersonTrackerShould, registerPersonPointAndDeregisterAfter20Iterations) {
    cv::Rect2d personCentered(300, 50, 40, 200);
    EXPECT_CALL(*PersonDetectorMock_,
      getPersonBoundingBox()).WillOnce(::testing::Return(personCentered));
    importMockData("1mopen.pcd");
    sut_.reset(new PersonTracker(
      outputIpcMock_, PersonDetectorMock_, parameters_, robotBaseConfiguration_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(sut_->getCameraDisplacement(), 0);
    PersonCentroid personPoint = sut_->trackPerson(cameraPose_);
    ASSERT_EQ(personPoint.objectID, 1);
    for (size_t i = 0; i < 21; i++) {
        importMockData("emptyPCD.pcd");
        personPoint = sut_->trackPerson(cameraPose_);
    }
    ASSERT_EQ(personPoint.objectID, -1);
}
