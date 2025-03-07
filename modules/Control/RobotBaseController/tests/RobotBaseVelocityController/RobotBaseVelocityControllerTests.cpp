/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN BE/CEM/MRO
 *         Jorge Playán Garai CERN BE/CEM/MRO
 * 
 *  ==================================================================================================
 */

#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"

#include "RobotBase/RobotBaseConfiguration.hpp"
#include "RobotBaseController/RobotBaseVelocityController/RobotBaseVelocityController.hpp"
#include "RobotBaseController/IRobotBaseController.hpp"
#include "RobotBaseController/RobotBaseControllerCommunicationPoint/RobotBaseControllerCommunicationPointFactory.hpp"

#include "TrajectoryPointGenerator/TrajectoryPointGeneratorMock.hpp"
#include "RobotBase/RobotBaseMock.hpp"

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

using crf::control::robotbasecontroller::IRobotBaseController;
using crf::control::robotbasecontroller::RobotBaseVelocityController;
using crf::control::robotbasecontroller::RobotBaseControllerCommunicationPointFactory;
using crf::control::trajectorypointgenerator::TrajectoryPointGeneratorMock;

using crf::actuators::robotbase::RobotBaseMock;
using crf::actuators::robotbase::RobotBaseConfiguration;

using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskTrajectoryData;

#define CTRL_ALLOWED_OFFSET 0.1
#define TASK_SPACE_SIZE 6

class RobotBaseVelocityControllerShould: public ::testing::Test {
 protected:
    RobotBaseVelocityControllerShould():
        logger_("RobotBaseVelocityControllerShould"),
        trajPoint_{},
        basePosition_{},
        robotBaseConfiguration_(new RobotBaseConfiguration()) {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        testFileDirName_ = __FILE__;
    }
    ~RobotBaseVelocityControllerShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        robotBaseMock_.reset(new NiceMock<RobotBaseMock>);
        configureRobotBaseDefaultBehavior();
        sut_.reset(new RobotBaseVelocityController(robotBaseMock_));
    }

    void configureRobotBaseDefaultBehavior() {
        testFileDirName_ = testFileDirName_.substr(0, testFileDirName_.find("modules/"));
        testFileDirName_ += "modules/Actuators/RobotBase/tests/config/";
        testFileDirName_.append("goodCernBot2Config.json");

        std::ifstream config(testFileDirName_);
        ASSERT_TRUE(robotBaseConfiguration_->parse(nlohmann::json::parse(config)));
        ON_CALL(*robotBaseMock_, getConfiguration())
            .WillByDefault(Return(robotBaseConfiguration_));
        ON_CALL(*robotBaseMock_, setTaskVelocity(_))
            .WillByDefault(Invoke([this](const TaskVelocity& velocity) {
                baseVelocityOpt_ = velocity;
                return true;
            }));
        ON_CALL(*robotBaseMock_, getTaskPose())
            .WillByDefault(Return(basePosition_));
        ON_CALL(*robotBaseMock_, getTaskVelocity())
            .WillByDefault(Invoke([this]() {
                if (!baseVelocityOpt_) {
                    baseVelocityOpt_ = TaskVelocity({0, 0, 0, 0, 0, 0});
                }
                return baseVelocityOpt_;
            }));
        ON_CALL(*robotBaseMock_, stopBase())
            .WillByDefault(Invoke([this]() {
                baseVelocityOpt_ = boost::optional<TaskVelocity>(
                    crf::utility::types::TaskVelocity({0, 0, 0, 0, 0, 0}));
                return true;
            }));
    }
    crf::utility::logger::EventLogger logger_;
    std::string testFileDirName_;
    TaskTrajectoryData trajPoint_;
    TaskPose basePosition_;
    boost::optional<TaskVelocity> baseVelocityOpt_;
    std::condition_variable lastTrajPointCv_;
    std::mutex m_;
    std::shared_ptr<RobotBaseConfiguration> robotBaseConfiguration_;
    std::shared_ptr<NiceMock<RobotBaseMock> > robotBaseMock_;
    std::unique_ptr<IRobotBaseController> sut_;
};

TEST_F(RobotBaseVelocityControllerShould, returnTrueIfInitializedOrDeinitializedTwice) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->deinitialize());
    logger_->info("Do same thing once again");
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotBaseVelocityControllerShould, callZeroVelocityOnDeinitialize) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    for (int i = 0; i < TASK_SPACE_SIZE; i++) {
        ASSERT_FLOAT_EQ(baseVelocityOpt_.get()[i], .0);
    }
}

TEST_F(RobotBaseVelocityControllerShould, callZeroVelocityOnCallControllerStop) {
    baseVelocityOpt_ = TaskVelocity({0, 0, 0, 0, 0, 0});
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->interruptTrajectory());
    ASSERT_FLOAT_EQ(baseVelocityOpt_.get()[0], 0);
}

TEST_F(RobotBaseVelocityControllerShould, allOperationsFailIfControllerIsNotInitializedTest) {
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getPosition(), crf::utility::types::TaskPose()));
    ASSERT_TRUE(areAlmostEqual(sut_->getVelocity(), crf::utility::types::TaskVelocity()));
    ASSERT_FALSE(sut_->interruptTrajectory());
    TaskTrajectoryData trajPoint;
    ASSERT_FALSE(sut_->setVelocity(trajPoint.velocity));
    TaskPose taskpos1;
    std::vector<TaskPose> trajectory;
    trajectory.push_back(taskpos1);
    ASSERT_FALSE(sut_->setPosition(trajectory).valid());
    ASSERT_FALSE(sut_->interruptTrajectory());
}

TEST_F(RobotBaseVelocityControllerShould, haveNonZeroVelocityWhenMoving) {
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::TaskSpace taskSpace({true, true, false, false, false, true});
    ASSERT_TRUE(areAlmostEqual(
        sut_->getPosition(), crf::utility::types::TaskPose(), 1.0e-6, taskSpace));
    ASSERT_TRUE(areAlmostEqual(
        sut_->getVelocity(), crf::utility::types::TaskVelocity(), 1.0e-6, taskSpace));
    ASSERT_TRUE(sut_->interruptTrajectory());

    TaskPose pos(Eigen::Vector3d({0.1, 0.1, 0}),
    crf::math::rotation::CardanXYZ({0, 0, 0}));
    auto res = sut_->setPosition(pos);
    ASSERT_TRUE(res.valid());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_FALSE(areAlmostEqual(
        sut_->getVelocity(),
        crf::utility::types::TaskVelocity({0, 0, 0, 0, 0, 0}),
        1.0e-6,
        taskSpace));
    ASSERT_TRUE(sut_->interruptTrajectory());
}
