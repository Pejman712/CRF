/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Prados CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <Eigen/Dense>

#include "EventLogger/EventLogger.hpp"
#include "GraphOptimization/GraphOptimization.hpp"
#include "GraphOptimization/GraphOptimizationConfiguration.hpp"
#include "Mapper3d/Mapper3d.hpp"

#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <mutex>
#include <condition_variable>
#include <pcl/common/transforms.h>

#include "Types/Types.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"
#include "RobotBase/RobotBaseDefaultKinematics.hpp"
#include "RobotPoseEstimators/RobotPoseEstimator.hpp"
#include "RobotPoseEstimators/IRobotPoseEstimator.hpp"

#include "Mocks/Sensors/IMUMock.hpp"
#include "Mocks/Robots/RobotBaseMock.hpp"

using crf::applications::graphoptimization::GraphOptimization;
using crf::applications::graphoptimization::GraphOptimizationConfiguration;
using crf::applications::graphoptimization::NeighboursSearch;
using crf::applications::graphoptimization::Solvers;
using crf::applications::graphoptimization::Simulations;
using crf::applications::graphoptimization::SlamParameters;

using testing::_;
using testing::Return;
using testing::NiceMock;
using testing::AtLeast;
using testing::Invoke;
using testing::FloatEq;
using testing::Matcher;

using crf::sensors::imu::IMUMock;
using crf::sensors::imu::IMUData;
using crf::utility::types::TaskPose;
using crf::robots::robotbase::RobotBaseMock;
using crf::applications::robotposeestimator::IRobotPoseEstimator;
using crf::applications::robotposeestimator::RobotPoseEstimator;
using crf::robots::robotbase::RobotBaseConfiguration;
using crf::robots::robotbase::RobotBaseDefaultKinematics;

class SlamShould: public ::testing::Test {
 public:
    SlamShould():
    logger_("SlamShould"),
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
        testFileDirName_.append("charmBotConfig.json");
        std::ifstream config(testFileDirName_);
        robotBaseConfiguration_->parse(nlohmann::json::parse(config));
        kinematics_.reset(new RobotBaseDefaultKinematics(*robotBaseConfiguration_));
        robotBaseMock_.reset(new NiceMock<RobotBaseMock>());
        configureRobotBaseDefaultBehavior();
        imuMock_.reset(new NiceMock<IMUMock>());
        configureIMUBehavior();
        ukf = std::make_shared<
            Kalman::UnscentedKalmanFilter<crf::applications::robotposeestimator::SystemState>>(1);
        poseEstimator_.reset(new RobotPoseEstimator(
            ukf,
            robotBaseMock_, nullptr));

        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("SlamTest.cpp"));
        testDirName_ += "config/";
    }
    ~SlamShould() {
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
    std::unique_ptr<IRobotPoseEstimator> poseEstimator_;

    std::string testDirName_;
    std::unique_ptr<GraphOptimization> graph_;
};

TEST_F(SlamShould, DISABLED_testSlamRGBdSystem) {
    measuredMotorVelocity_.clear();
    auto measuredMotorVelocityValid = kinematics_->getWheelsVelocity({0.0, -0.2, .0, .0, .0, .0});
    ASSERT_TRUE(measuredMotorVelocityValid);
    measuredMotorVelocity_ = measuredMotorVelocityValid.get();
    auto kinematicsValid = kinematics_->getTaskVelocity(measuredMotorVelocity_);
    ASSERT_TRUE(poseEstimator_->initialize());
    ASSERT_TRUE(kinematicsValid);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
        std::unique_lock<std::mutex> lk(m_);
        writerCv_.wait_for(lk, timeOut_);
    }

    ASSERT_NO_THROW(graph_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_TRUE(graph_->parse(testDirName_ + "configFile_good.json"));

    Eigen::Matrix4f realMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previousMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f SupposedMotion = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr previousPointCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());

    for (int i = 1; i < 4; i++) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud
            (new pcl::PointCloud<pcl::PointXYZRGBA>());
        std::string name = testDirName_+ "../data/testOrganizedCloud.pcd";
        pcl::io::loadPCDFile<pcl::PointXYZRGBA> (name, *inPointCloud);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud(
            new pcl::PointCloud<pcl::PointXYZRGBA>());
        Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();
        transformationMatrix(2, 3) = -(i - 1) * 0.04;
        pcl::transformPointCloud(*inPointCloud, *transformedCloud, transformationMatrix);

        auto posValidity = poseEstimator_->getPosition();
        auto velValidity = poseEstimator_->getVelocity();
        ASSERT_TRUE(poseEstimator_->getAcceleration());
        ASSERT_TRUE(posValidity);
        ASSERT_TRUE(velValidity);

        auto robotPosition = posValidity.get();
        Eigen::AngleAxisd rollAngle(robotPosition(3), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(robotPosition(4), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(robotPosition(5), Eigen::Vector3d::UnitZ());
        Eigen::Quaternion<double> rotVector = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix3d rotationMatrix = rotVector.matrix();

        SupposedMotion <<
            rotationMatrix(0, 0), rotationMatrix(0, 1), rotationMatrix(0, 2), robotPosition(0),
            rotationMatrix(1, 0), rotationMatrix(1, 1), rotationMatrix(1, 2), robotPosition(1),
            rotationMatrix(2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2), robotPosition(2),
            0, 0, 0, 1;
        std::cout << "Supposed motion: " << realMotion << std::endl;

        if (i > 1) {
            // Update map and take the matrix of motion
            boost::optional<Eigen::Matrix4f> boostRealMotion;
            previousMotion = realMotion;

            boostRealMotion = graph_->comparePointClouds(SupposedMotion, realMotion,
                transformedCloud, previousPointCloud);

            ASSERT_TRUE(boostRealMotion);
            realMotion = boostRealMotion.get();
            std::cout << "Real motion: " << realMotion << std::endl;
        }
        previousPointCloud = transformedCloud;

        ASSERT_TRUE(graph_->addVertex(realMotion, previousMotion, boost::none, boost::none,
            boost::none));
    }

    ASSERT_TRUE(poseEstimator_->deinitialize());
}

TEST_F(SlamShould, DISABLED_testSlamVelodyneSystem) {
    measuredMotorVelocity_.clear();
    auto measuredMotorVelocityValid = kinematics_->getWheelsVelocity({0.0, -0.2, .0, .0, .0, .0});
    ASSERT_TRUE(measuredMotorVelocityValid);
    measuredMotorVelocity_ = measuredMotorVelocityValid.get();
    auto kinematicsValid = kinematics_->getTaskVelocity(measuredMotorVelocity_);
    ASSERT_TRUE(poseEstimator_->initialize());
    ASSERT_TRUE(kinematicsValid);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
        std::unique_lock<std::mutex> lk(m_);
        writerCv_.wait_for(lk, timeOut_);
    }

    ASSERT_NO_THROW(graph_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_TRUE(graph_->parse(testDirName_ + "configFile_unorganized.json"));

    Eigen::Matrix4f realMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previousMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f SupposedMotion = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr previousPointCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());

    for (int i = 1; i < 4; i++) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud
            (new pcl::PointCloud<pcl::PointXYZRGBA>());
        std::string name = testDirName_+ "../data/testUnOrganizedCloud.pcd";
        pcl::io::loadPCDFile<pcl::PointXYZRGBA> (name, *inPointCloud);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud(
            new pcl::PointCloud<pcl::PointXYZRGBA>());
        Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();
        transformationMatrix(2, 3) = -(i - 1) * 0.04;
        pcl::transformPointCloud(*inPointCloud, *transformedCloud, transformationMatrix);

        auto posValidity = poseEstimator_->getPosition();
        auto velValidity = poseEstimator_->getVelocity();
        ASSERT_TRUE(poseEstimator_->getAcceleration());
        ASSERT_TRUE(posValidity);
        ASSERT_TRUE(velValidity);

        auto robotPosition = posValidity.get();
        Eigen::AngleAxisd rollAngle(robotPosition(3), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(robotPosition(4), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(robotPosition(5), Eigen::Vector3d::UnitZ());
        Eigen::Quaternion<double> rotVector = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix3d rotationMatrix = rotVector.matrix();

        SupposedMotion <<
            rotationMatrix(0, 0), rotationMatrix(0, 1), rotationMatrix(0, 2), robotPosition(0),
            rotationMatrix(1, 0), rotationMatrix(1, 1), rotationMatrix(1, 2), robotPosition(1),
            rotationMatrix(2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2), robotPosition(2),
            0, 0, 0, 1;
        std::cout << "Supposed motion: " << realMotion << std::endl;

        if (i > 1) {
            // Update map and take the matrix of motion
            boost::optional<Eigen::Matrix4f> boostRealMotion;
            previousMotion = realMotion;

            boostRealMotion = graph_->comparePointClouds(SupposedMotion, realMotion,
                transformedCloud, previousPointCloud);

            ASSERT_TRUE(boostRealMotion);
            realMotion = boostRealMotion.get();
            std::cout << "Real motion: " << realMotion << std::endl;
        }
        previousPointCloud = transformedCloud;

        ASSERT_TRUE(graph_->addVertex(realMotion, previousMotion, boost::none, boost::none,
            boost::none));
    }

    ASSERT_TRUE(poseEstimator_->deinitialize());
}
