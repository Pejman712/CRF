/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>
#include <vector>
#include <random>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/algorithm/string/split.hpp>

#include "Mocks/Sensors/LaserMock.hpp"
#include "Mocks/Algorithms/CollisionDetectorMock.hpp"
#include "Mocks/Algorithms/PathPlannerMock.hpp"
#include "Mocks/Algorithms/TaskTrajectoryGeneratorMock.hpp"
#include "Mocks/Robots/RobotBaseControllerMock.hpp"

#include "Laser/Hokuyo/HokuyoLaser.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Types/TaskTypes/TaskPose.hpp"
#include "TrajectoryGenerator/TrajectoryData.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"
#include "MotionPlanner/MotionPlannerLaser.hpp"
#include "MotionPlanner/IMotionPlanner.hpp"

using testing::_;
using testing::Return;
using testing::NiceMock;
using testing::DoAll;
using testing::DoDefault;
using testing::Invoke;
using testing::Matcher;

using crf::algorithms::pathplanner::PathPlannerMock;
using crf::algorithms::collisiondetector::CollisionDetectorMock;
using crf::sensors::laser::LaserMock;
using crf::robots::robotbase::RobotBaseConfiguration;
using crf::algorithms::trajectorygenerator::TaskTrajectoryGeneratorMock;
using crf::robots::robotbasecontroller::RobotBaseControllerMock;
using crf::applications::motionplanner::MotionPlannerLaser;
using crf::applications::motionplanner::IMotionPlanner;

class MotionPlannerLaserShould: public ::testing::Test {
 protected:
    MotionPlannerLaserShould():
        logger_("MotionPlannerLaserShould"),
        testDirName_(__FILE__),
        interruptTrajectory_(false),
        computedPath_(),
        visualizeMotion_(false),
        robotBaseConfiguration_(new RobotBaseConfiguration()),
        position_({0, 0, 0, 0, 0, 0}),
        velocity_({0, 0, 0, 0, 0, 0}),
        laserPose_() {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = testDirName_.substr(0, testDirName_.find("MotionPlannerLaserTests.cpp"));
        std::string localTestDir = testDirName_;
        cloud_ = importMockData(localTestDir + "data/sps.pcd");
        computedPath_ = importPathData(localTestDir + "data/path.csv");
        std::vector<crf::utility::types::TaskPose> computedPositions;
        for (auto& pathPoint : computedPath_) {
            computedPositions.push_back(
                MotionPlannerLaser::VectorPath2TaskPosePath(pathPoint));
        }
        computedTrajectory_.position = computedPositions;
        // Mocking RobotBaseController
        configureRobotBaseControllerDefaultBehavior();

        // Mocking Laser
        laserMock_.reset(new NiceMock<LaserMock>());
        ON_CALL(*laserMock_, initialize()).WillByDefault(Return(true));
        ON_CALL(*laserMock_, deinitialize()).WillByDefault(Return(true));
        ON_CALL(*laserMock_, getPointCloud()).WillByDefault(Return(cloud_));


        // Mocking Path Planning
        pathPlannerMock_.reset(new NiceMock<PathPlannerMock>());
        ON_CALL(*pathPlannerMock_, computePath(_, _)).WillByDefault(Return(true));
        ON_CALL(*pathPlannerMock_, getPathLength()).WillByDefault(Return(computedPath_.size()));
        ON_CALL(*pathPlannerMock_, getPath()).WillByDefault(Return(computedPath_));

        // Mocking Collision Detector
        collisionDetectorMock_.reset(new NiceMock<CollisionDetectorMock>());
        ON_CALL(*collisionDetectorMock_, updateMap(_)).WillByDefault(Return(true));

        trajectoryGeneratorMock_.reset(new NiceMock<TaskTrajectoryGeneratorMock>());
        ON_CALL(*trajectoryGeneratorMock_, computeTrajectory(_)).WillByDefault(Return(true));
        ON_CALL(*trajectoryGeneratorMock_, getTaskTrajectory()).
        WillByDefault(Return(boost::optional
            <crf::algorithms::trajectorygenerator::TaskTrajectoryData>
            (computedTrajectory_)));

        // Creating Robot Base Motion Planner
        sut_.reset( new MotionPlannerLaser(robotBaseControllerMock_, pathPlannerMock_,
            collisionDetectorMock_, trajectoryGeneratorMock_, robotBaseConfiguration_, laserMock_, laserPose_, 0.01));   // NOLINT
    }
    ~MotionPlannerLaserShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    void configureRobotBaseControllerDefaultBehavior() {
        std::string configurationsDir = testDirName_;
        configurationsDir = configurationsDir.substr(0,
            configurationsDir.find("tests/"));
        configurationsDir += "tests/Configurations/Robots/RobotBase/";
        configurationsDir.append("charmBotConfig.json");
        std::ifstream config(configurationsDir);

        ASSERT_TRUE(robotBaseConfiguration_->parse(nlohmann::json::parse(config)));

        // Mocking RobotBaseController
        robotBaseControllerMock_.reset(new NiceMock<RobotBaseControllerMock>());

        ON_CALL(*robotBaseControllerMock_, initialize()).WillByDefault(Return(true));
        ON_CALL(*robotBaseControllerMock_, deinitialize()).WillByDefault(Return(true));

        ON_CALL(*robotBaseControllerMock_, setPosition(
            Matcher<const crf::utility::types::TaskPose&>(_)))
            .WillByDefault(Invoke([this](const crf::utility::types::TaskPose&) {
                return std::async(std::launch::async, [this]() {
                    for (int i = 0; i < 50; i++) {
                        if (interruptTrajectory_) return false;
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                    return true;
                });
            }));

        ON_CALL(*robotBaseControllerMock_, setPosition(
            Matcher<const std::vector<crf::utility::types::TaskPose>&>(_)))
            .WillByDefault(Invoke([this](const std::vector<crf::utility::types::TaskPose>&){  // NOLINT
                return std::async(std::launch::async, [this](){
                    for (int i = 0; i < 50; i++) {
                        if (interruptTrajectory_) return false;
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                    return true;
                });
            }));

/*
        ON_CALL(*robotBaseControllerMock_, setVelocity(
            Matcher<const crf::utility::types::TaskVelocity&>(_))).WillByDefault(Return(true));
*/
        ON_CALL(*robotBaseControllerMock_, interruptTrajectory()).WillByDefault(Invoke([this]() {
            interruptTrajectory_ = true;
            return true;
        }));

        ON_CALL(*robotBaseControllerMock_, getPosition()).WillByDefault(Return(position_));
        ON_CALL(*robotBaseControllerMock_, getVelocity()).WillByDefault(Return(velocity_));
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr importMockData(const std::string& fileName) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
            if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (fileName, *cloud) == -1) {
                PCL_ERROR("Couldn't read file test_pcd.pcd \n");
                return nullptr;
            }
        return cloud;
    }
    std::vector<std::vector<float>> importPathData(const std::string& fileDirName) {
        std::ifstream readFile;
        readFile.open(fileDirName, std::ios::app);
        if (!readFile.is_open()) {
            logger_->error("Failed to open data file.");
            throw std::runtime_error("Failed to open data file.");
        }
        std::vector<std::vector<float>> data;
        while (!readFile.eof()) {
            std::vector<std::string> Vec;
            std::string row;
            std::getline(readFile, row);
            if (readFile.bad() || readFile.fail() || row.length() == 0) {
                logger_->error("Bad line.");
                break;
            }
            boost::algorithm::split(Vec, row, boost::is_any_of(",;"));
            std::vector<float> pathPoint;
            if (Vec[0].find("steps") != std::string::npos) {
                if (data.size() == 0) {
                    continue;
                }
            }
            if (Vec.size() != 4) {
                std::cout << "Error in reading" << row << std::endl;
                return data;
            }
            for (size_t i = 1; i < 4; i++) {
                pathPoint.push_back(std::atof(Vec[i].c_str()));
            }
            data.push_back(pathPoint);
        }
        return data;
    }
    void visualize() {
        pcl::visualization::PCLVisualizer::Ptr viewer
            (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor(47, 79, 79);
        viewer->initCameraParameters();
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mockPointCloudData =
            importMockData("cloud.pcd");
        auto pathResult =  importPathData("Path.csv");
        viewer->addPointCloud<pcl::PointXYZRGBA> (mockPointCloudData,
            "sample cloud "+ std::to_string(0));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            2, "sample cloud "+ std::to_string(0));
        auto  robotParameters = robotBaseConfiguration_->getRobotParameters();
        for (size_t i = 0; i < pathResult.size(); i++) {
            Eigen::Vector3f translation;
            auto trajPoint = pathResult[i];
            translation << trajPoint[0],  trajPoint[1], 0;
            Eigen::Matrix3f mat;
            mat.setIdentity();
            crf::utility::types::TaskPose cameraPose_({.0, .0, .0, .0, .0, trajPoint[2]});
            auto transform = cameraPose_.getPosRotMatrix();
            int counter = 3;
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    mat(j, k) = transform[counter];
                    counter++;
                }
            }
            Eigen::Quaternionf q(mat);
            viewer->addCube(translation, q, robotParameters.wheelsDistanceY,
                robotParameters.wheelsDistanceX, 0.15, "robot" + std::to_string(i), 0);
            // viewer->spinOnce (500);
            // usleep(50000);
        }
        viewer->addCoordinateSystem(0.1);
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            sleep(1);
        }
    }
    crf::utility::logger::EventLogger logger_;
    std::string testDirName_;
    bool interruptTrajectory_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
    std::vector<std::vector<float>> computedPath_;
    crf::algorithms::trajectorygenerator::TaskTrajectoryData computedTrajectory_;
    bool visualizeMotion_;
    std::shared_ptr<RobotBaseConfiguration> robotBaseConfiguration_;
    crf::utility::types::TaskPose position_;
    crf::utility::types::TaskVelocity velocity_;
    crf::utility::types::TaskPose laserPose_;
    std::shared_ptr<NiceMock<LaserMock> >laserMock_;
    std::shared_ptr<NiceMock<RobotBaseControllerMock> > robotBaseControllerMock_;
    std::shared_ptr<NiceMock<PathPlannerMock> > pathPlannerMock_;
    std::shared_ptr<NiceMock<CollisionDetectorMock> > collisionDetectorMock_;
    std::shared_ptr<NiceMock<TaskTrajectoryGeneratorMock> > trajectoryGeneratorMock_;
    std::unique_ptr<IMotionPlanner> sut_;
};

TEST_F(MotionPlannerLaserShould, returnFalseIfInitializedOrDeinitializedTwice) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(MotionPlannerLaserShould, stopIfCannotRetrievePositionFromRobotBaseController) {
    EXPECT_CALL(*robotBaseControllerMock_, getPosition())
        .WillRepeatedly(Return(crf::utility::types::TaskPose()));
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::TaskPose goal({1.0, 2.0, .0, .0, .0, .0});
    ASSERT_TRUE(sut_->plan(goal));
}

TEST_F(MotionPlannerLaserShould, stopIfCannotRetrievePCLFromLaser) {
    EXPECT_CALL(*laserMock_, getPointCloud())
        .WillOnce(Return(nullptr)).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::TaskPose goal({1.0, 2.0, .0, .0, .0, .0});
    ASSERT_FALSE(sut_->plan(goal));
}

TEST_F(MotionPlannerLaserShould, planWithPCLFromFile) {
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::TaskPose goal({1.0, 2.0, .0, .0, .0, .0});
    ASSERT_TRUE(sut_->plan(goal));
    ASSERT_TRUE(sut_->plan(goal));
    // visualize();
}

TEST_F(MotionPlannerLaserShould, noResultIfPathNotObtained) {
     EXPECT_CALL(*pathPlannerMock_, getPathLength())
        .WillOnce(Return(boost::none)).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::TaskPose goal({1.0, 2.0, .0, .0, .0, .0});
    ASSERT_FALSE(sut_->plan(goal));
}

/*
 * Trajectories are now performed inside the robot arm controller for now,
 * With the new implemenation we should be able to un-disable this test in
 * the future (jplayang)
 */

TEST_F(MotionPlannerLaserShould, DISABLED_noResultIfTrajectoryNotObtained) {
     EXPECT_CALL(*trajectoryGeneratorMock_, computeTrajectory(_))
        .WillOnce(Return(false)).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::TaskPose goal({1.0, 2.0, .0, .0, .0, .0});
    ASSERT_FALSE(sut_->plan(goal));
}
