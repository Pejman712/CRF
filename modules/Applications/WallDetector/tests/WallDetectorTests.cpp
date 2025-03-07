/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "WallDetector/WallDetector.hpp"

using crf::applications::walldetector::WallDetector;
using crf::applications::walldetector::WallParameter;
using crf::robots::robotbase::RobotBaseConfiguration;

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::SetArgReferee;

class WallDetectorShould: public ::testing::Test {
 protected:
    WallDetectorShould(): logger_("WallDetectorShould"),
        testAddress_(__FILE__),
        robotBaseConfiguration_(new RobotBaseConfiguration()),
        cameraPose_({0, 0, 0, 0, 0, 0}) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        testAddress_ = testAddress_.substr(0, testAddress_.find("WallDetectorTests.cpp"));
        std::ifstream param(testAddress_ + "./config/WallDetectorParameters.json");
        param >> parameters_;
    }

    ~WallDetectorShould() {
        logger_->info("{} END with {}",
          testing::UnitTest::GetInstance()->current_test_info()->name(),
          testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        std::string configFileName = testAddress_;
        configFileName = configFileName.append(
            "../../Configurations/Robots/RobotBase/charmBotConfig.json");

        std::ifstream configFile(configFileName);
        auto json = nlohmann::json::parse(configFile);
        ASSERT_TRUE(robotBaseConfiguration_->parse(json));
        logger_->info("test SetUp done");
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr importMockData(const std::string& fileName) {
        std::string fileDirName = testAddress_;
        fileDirName = fileDirName.append(fileName);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
            if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (fileDirName, *cloud) == -1) {
                PCL_ERROR("Couldn't read file test_pcd.pcd \n");
                return nullptr;
            }
        return cloud;
    }

    crf::utility::logger::EventLogger logger_;
    std::string testAddress_;
    std::shared_ptr<RobotBaseConfiguration> robotBaseConfiguration_;
    crf::utility::types::TaskPose cameraPose_;
    nlohmann::json parameters_;
    std::unique_ptr<WallDetector> sut_;
};

TEST_F(WallDetectorShould, ReturnExceptionIfWallParameterConfigIsCorrupt) {
    std::ifstream param(testAddress_ + "./config/MissingWallDetectorParameters.json");
    param >> parameters_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mockPointCloudData =
    importMockData("hokuyoDataExamples/laserData.pcd");
    ASSERT_THROW(
      sut_.reset(new WallDetector(robotBaseConfiguration_, parameters_)), std::invalid_argument);
}

TEST_F(WallDetectorShould, ReturnEmptyVectorIfPointCloudDataIsEmpty) {
    sut_.reset(new WallDetector(robotBaseConfiguration_, parameters_));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mockPointCloudData(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
    auto walls = sut_->detectWall(mockPointCloudData, cameraPose_);
    ASSERT_EQ(walls.size(), 0);
}

TEST_F(WallDetectorShould, ReturnEmptyVectorIfFilteredCloudDataIsEmpty) {
    sut_.reset(new WallDetector(robotBaseConfiguration_, parameters_));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mockPointCloudData(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (size_t i = 0; i < 100; i++) {
        pcl::PointXYZRGBA temporaryPoint;
        temporaryPoint.x = i*0.001;
        temporaryPoint.y = i*0.001;
        temporaryPoint.z = (rand() + 1) / (static_cast<float>(RAND_MAX)*100);  // NOLINT
        mockPointCloudData->points.push_back(temporaryPoint);
    }
    auto walls = sut_->detectWall(mockPointCloudData, cameraPose_);
    ASSERT_EQ(walls.size(), 0);
}

TEST_F(WallDetectorShould, ReturnEmptyVectorIfPointCloudIsTooSparse) {
    sut_.reset(new WallDetector(robotBaseConfiguration_, parameters_));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mockPointCloudData(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (size_t i = 0; i < 100; i++) {
        pcl::PointXYZRGBA temporaryPoint;
        temporaryPoint.x = i+1;
        temporaryPoint.y = i+1;
        temporaryPoint.z = (rand() + 1) / (static_cast<float>(RAND_MAX)*100);  // NOLINT
        mockPointCloudData->points.push_back(temporaryPoint);
    }
    auto walls = sut_->detectWall(mockPointCloudData, cameraPose_);
    ASSERT_EQ(walls.size(), 0);
}

TEST_F(WallDetectorShould, ReturnEmptyVectorIfNoLinesWereDetected) {
    sut_.reset(new WallDetector(robotBaseConfiguration_, parameters_));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mockPointCloudData(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (size_t i = 0; i < 100; i++) {
        pcl::PointXYZRGBA temporaryPoint;
        temporaryPoint.x = 1 + rand() / (static_cast<float>(RAND_MAX)*10);  // NOLINT
        temporaryPoint.y = 1 + rand() / (static_cast<float>(RAND_MAX)*10);  // NOLINT
        temporaryPoint.z = 1 + rand() / (static_cast<float>(RAND_MAX)*100);  // NOLINT
        mockPointCloudData->points.push_back(temporaryPoint);
    }
    auto walls = sut_->detectWall(mockPointCloudData, cameraPose_);
    ASSERT_EQ(walls.size(), 0);
}

TEST_F(WallDetectorShould, ReturnPreciseSlopeOfWall) {
    sut_.reset(new WallDetector(robotBaseConfiguration_, parameters_));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mockPointCloudData(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (size_t i = 0; i < 100; i++) {
        pcl::PointXYZRGBA temporaryPoint;
        temporaryPoint.x = 0.01*i;
        temporaryPoint.y = 0.01*i*0.5;
        temporaryPoint.z = (rand() + 1) / (static_cast<float>(RAND_MAX)*100);  // NOLINT
        mockPointCloudData->points.push_back(temporaryPoint);
    }
    auto walls = sut_->detectWall(mockPointCloudData, cameraPose_);
    ASSERT_EQ(walls.size(), 1);
    WallParameter leftWall = walls[0];
    ASSERT_NEAR(leftWall.theta, 1.10715, 0.01);
}

TEST_F(WallDetectorShould, ReturnPreciseWallDistance) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mockPointCloudData =
    importMockData("hokuyoDataExamples/laserData.pcd");
    sut_.reset(new WallDetector(robotBaseConfiguration_, parameters_));
    auto walls = sut_->detectWall(mockPointCloudData, cameraPose_);
    ASSERT_EQ(walls.size(), 2);
    WallParameter leftWall = walls[0];
    ASSERT_NEAR(leftWall.distance, 0.9500, 0.01);
    ASSERT_NEAR(leftWall.length, 2.206, 0.01);
}

TEST_F(WallDetectorShould, ReturnResultWallDistanceFromPCDFile) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mockPointCloudData =
        importMockData("hokuyoDataExamples/test_pcd.pcd");
    ASSERT_NO_THROW(sut_.reset(new WallDetector(robotBaseConfiguration_, parameters_)));
    cameraPose_ = crf::utility::types::TaskPose({-0.333, .0, .0, .0, .0, .0});
    auto walls = sut_->detectWall(mockPointCloudData, cameraPose_);
    ASSERT_EQ(walls.size(), 3);
}
