/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * Author: Jorge Playán Garai   CERN BE/CEM/MRO
 * 
 *  ==================================================================================================
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "RobotArmKinematics/IRobotArmKinematics.hpp"
#include "RobotArmKinematics/RobotArmKDLKinematics/RobotArmKDLKinematics.hpp"
#include "RobotArmController/RobotArmVelocityController/RobotArmVelocityController.hpp"

#include "ClosedLoopController/ClosedLoopControllerMock.hpp"
#include "RobotArm/RobotArmMock.hpp"

#define NUM_JOINTS 6

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

using crf::control::closedloopcontroller::ClosedLoopControllerMock;
using crf::control::robotarmcontroller::IRobotArmController;
using crf::control::robotarmcontroller::RobotArmVelocityController;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::control::robotarmkinematics::IRobotArmKinematics;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::actuators::robotarm::RobotArmConfiguration;
using crf::control::robotarmkinematics::RobotArmKDLKinematics;
using crf::actuators::robotarm::RobotArmMock;

class RobotArmVelocityControllerShould: public ::testing::Test {
 protected:
    RobotArmVelocityControllerShould():
        logger_("RobotArmVelocityControllerShould"),
        simulatedJointPositions_(NUM_JOINTS),
        simulatedJointVelocities_(NUM_JOINTS) {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0,
            testDirName_.find("RobotArmVelocityController/RobotArmVelocityControllerTests.cpp"));
        robotArmMock_.reset(new NiceMock<RobotArmMock>);

        simulatedJointPositions_[1] = -M_PI/2;
        simulatedJointPositions_[2] = M_PI/2;
        simulatedJointPositions_[4] = M_PI/2;

        lastSetVelocityTime_ = std::chrono::high_resolution_clock::now();
    }

    void SetUp() override {
        robotArmConfiguration_.reset(new RobotArmConfiguration);
        // let's test the controller with kinova configuration
        std::ifstream robotData(testDirName_ + "/config/KinovaConfig.json");
        nlohmann::json robotJSON = nlohmann::json::parse(robotData);
        ASSERT_TRUE(robotArmConfiguration_->parse(robotJSON));
        kinematics_.reset(new RobotArmKDLKinematics(robotArmConfiguration_));
        closedLoopControllerMock_.reset(new NiceMock<ClosedLoopControllerMock>);
        configureRobotArmDefaultBehavior();
        configureClosedLoopControllerDefaultBehavior();
        sut_.reset(new RobotArmVelocityController(robotArmMock_));
    }
    ~RobotArmVelocityControllerShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    void configureRobotArmDefaultBehavior() {
        ON_CALL(*robotArmMock_, getConfiguration()).WillByDefault(Return(robotArmConfiguration_));
        /*
         * Simulated behavior is to increase position by requested velocity,
         * because closed loop controller mock is returning the difference between
         * the goal and the feedback value; In that way we always arrive at the goal in one
         * iteration which makes the test slightly faster
         */
        ON_CALL(*robotArmMock_, setJointVelocities(_)).WillByDefault(Invoke(
            [this](const JointVelocities& jointVelocities) {
                if (jointVelocities.size() != NUM_JOINTS) return false;
                auto now = std::chrono::high_resolution_clock::now();
                float timestep = static_cast<float>(std::chrono::duration_cast
                    <std::chrono::microseconds>(now-lastSetVelocityTime_).count())/1e6;
                for (int i = 0; i < NUM_JOINTS; i++) {
                    simulatedJointPositions_[i] += jointVelocities[i]*timestep;
                }
                simulatedJointVelocities_ = jointVelocities;
                lastSetVelocityTime_ = now;
                return true;
            }));
        ON_CALL(*robotArmMock_, getJointPositions())
            .WillByDefault(Invoke([this](){ return simulatedJointPositions_;}));
        ON_CALL(*robotArmMock_, getJointVelocities())
            .WillByDefault(Invoke([this](){ return simulatedJointVelocities_;}));
        ON_CALL(*robotArmMock_, stopArm())
            .WillByDefault(Return(true));
    }

    void configureClosedLoopControllerDefaultBehavior() {
        ON_CALL(*closedLoopControllerMock_, calculate(_, _)).WillByDefault(Invoke(
            [this](const std::vector<double>& setpoint, const std::vector<double>& feedbackValue) {
                // PID does nothing because simulated robot arm is perfect
                return std::vector<double>(NUM_JOINTS);
            }));
    }

    std::chrono::time_point<std::chrono::high_resolution_clock> lastSetVelocityTime_;
    crf::utility::logger::EventLogger logger_;
    JointPositions simulatedJointPositions_;
    JointVelocities simulatedJointVelocities_;
    std::string testDirName_;
    std::shared_ptr<NiceMock<RobotArmMock> > robotArmMock_;
    std::shared_ptr<RobotArmConfiguration> robotArmConfiguration_;
    std::shared_ptr<IRobotArmKinematics> kinematics_;
    std::shared_ptr<ClosedLoopControllerMock> closedLoopControllerMock_;
    std::unique_ptr<IRobotArmController> sut_;
};

TEST_F(RobotArmVelocityControllerShould, returnTrueIfInitializedOrDeinitializedTwice) {
    sut_.reset(new RobotArmVelocityController(robotArmMock_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->deinitialize());
    // again ...
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmVelocityControllerShould, callStopArmOnDeinitialize) {
    sut_.reset(new RobotArmVelocityController(robotArmMock_));

    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*robotArmMock_, stopArm()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmVelocityControllerShould, allOperationsFailIfControllerIsNotInitializedTest) {
    sut_.reset(new RobotArmVelocityController(robotArmMock_));

    ASSERT_TRUE(sut_->deinitialize());

    JointPositions jointpos1(robotArmConfiguration_->getNumberOfJoints());
    std::vector<JointPositions> trajectory;
    trajectory.push_back(jointpos1);
    ASSERT_FALSE(sut_->setPosition(trajectory).valid());

    TaskPose taskpos1;
    std::vector<TaskPose> trajectoryTask;
    trajectoryTask.push_back(taskpos1);
    ASSERT_FALSE(sut_->setPosition(
        trajectoryTask,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::TaskSpaceTaskLoop, // NOLINT
        crf::control::robotarmcontroller::PointReferenceFrame::Global).valid());

    ASSERT_FALSE(sut_->interruptTrajectory());
    ASSERT_FALSE(sut_->setVelocity(JointVelocities(
        robotArmConfiguration_->getNumberOfJoints())));
    ASSERT_FALSE(sut_->setVelocity(
        TaskVelocity(),
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::TaskSpaceTaskLoop, // NOLINT
        crf::control::robotarmcontroller::PointReferenceFrame::Global));
    ASSERT_FALSE(sut_->setVelocity(
        TaskVelocity(),
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::JointSpace,
        crf::control::robotarmcontroller::PointReferenceFrame::Global));
}

TEST_F(RobotArmVelocityControllerShould, allOperationsFailOnWrongInputs) {
    sut_.reset(new RobotArmVelocityController(robotArmMock_));

    ASSERT_TRUE(sut_->initialize());

    JointPositions jointpos1(robotArmConfiguration_->getNumberOfJoints()-1);
    std::vector<JointPositions> trajectory;
    trajectory.push_back(jointpos1);
    ASSERT_FALSE(sut_->setPosition(trajectory).valid());

    trajectory.clear();
    ASSERT_FALSE(sut_->setPosition(trajectory).valid());

    jointpos1 = JointPositions(robotArmConfiguration_->getNumberOfJoints());
    jointpos1[0] = robotArmConfiguration_->getJointsConfiguration()[0].maximumPosition + 0.5;  // Out of range NOLINT
    trajectory.push_back(jointpos1);
    ASSERT_FALSE(sut_->setPosition(trajectory).valid());

    TaskPose taskpos1(
        {234.0, 0, 0},
        crf::math::rotation::CardanXYZ({0, 0, 0}));

    std::vector<TaskPose> trajectoryTask;
    trajectoryTask.push_back(taskpos1);
    ASSERT_FALSE(sut_->setPosition(
        trajectoryTask,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::JointSpace,
        crf::control::robotarmcontroller::PointReferenceFrame::Global).valid());

    trajectoryTask.clear();
    ASSERT_FALSE(sut_->setPosition(
        trajectoryTask,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::TaskSpaceTaskLoop, // NOLINT
        crf::control::robotarmcontroller::PointReferenceFrame::Global).valid());
    ASSERT_FALSE(sut_->setPosition(
        trajectoryTask,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::JointSpace,
        crf::control::robotarmcontroller::PointReferenceFrame::Global).valid());

    ASSERT_FALSE(sut_->setVelocity(JointVelocities(
        robotArmConfiguration_->getNumberOfJoints()-1)));
    ASSERT_FALSE(sut_->setJointsMaximumVelocity(JointVelocities(
        robotArmConfiguration_->getNumberOfJoints()-1)));
    ASSERT_FALSE(sut_->setJointsMaximumAcceleration(JointAccelerations(
        robotArmConfiguration_->getNumberOfJoints()-1)));

    TaskVelocity taskVel;
    taskVel[0] = robotArmConfiguration_->getTaskLimits().maximumVelocity[0]+1.0;
    ASSERT_FALSE(sut_->setTaskMaximumVelocity(taskVel));
    TaskAcceleration taskAcc;
    taskAcc[0] = robotArmConfiguration_->getTaskLimits().maximumAcceleration[0]+1.0;
    ASSERT_FALSE(sut_->setTaskMaximumAcceleration(taskAcc));
}

TEST_F(RobotArmVelocityControllerShould, jointTrajectoryIsInterruptedCorrectlyTest) {
    sut_.reset(new RobotArmVelocityController(robotArmMock_));

    ASSERT_TRUE(sut_->initialize());
    JointPositions jointpos1({
        M_PI/2, -M_PI/2,
        M_PI/2, -M_PI/2,
        M_PI/2, -M_PI/2
    });
    std::vector<JointPositions> trajectory;
    trajectory.push_back(jointpos1);

    auto result = sut_->setPosition(trajectory);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ASSERT_TRUE(sut_->interruptTrajectory());
    ASSERT_TRUE(result.valid());
    ASSERT_FALSE(result.get());
}

TEST_F(RobotArmVelocityControllerShould, DISABLED_taskTrajectoryIsInterruptedCorrectlyTest) {
    sut_.reset(new RobotArmVelocityController(robotArmMock_));

    ASSERT_TRUE(sut_->initialize());
    TaskPose taskPos(
        {0.1, -0.1, 0.2},
        crf::math::rotation::CardanXYZ({0, M_PI/2, 0}));

    std::vector<TaskPose> trajectory;
    trajectory.push_back(taskPos);

    auto result = sut_->setPosition(
        trajectory,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::JointSpace,
        crf::control::robotarmcontroller::PointReferenceFrame::Global);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ASSERT_TRUE(sut_->interruptTrajectory());
    ASSERT_TRUE(result.valid());
    ASSERT_FALSE(result.get());
}

TEST_F(RobotArmVelocityControllerShould, DISABLED_taskTrajectoryLinearIsInterruptedCorrectlyTest) {
    sut_.reset(new RobotArmVelocityController(robotArmMock_));

    ASSERT_TRUE(sut_->initialize());
    // Allow for controlLoop to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    TaskPose offset(
        {0.05, 0, 0},
        Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0}));

    std::vector<TaskPose> trajectory;
    trajectory.push_back(multiply(offset, sut_->getTaskPose()));
    auto result = sut_->setPosition(
        trajectory,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::TaskSpaceTaskLoop, // NOLINT
        crf::control::robotarmcontroller::PointReferenceFrame::Global);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ASSERT_TRUE(sut_->interruptTrajectory());
    ASSERT_TRUE(result.valid());
    ASSERT_FALSE(result.get());
}

TEST_F(RobotArmVelocityControllerShould, jointsTrajectoryArrivingToDestinationTest) {
    sut_.reset(new RobotArmVelocityController(robotArmMock_));

    ASSERT_TRUE(sut_->initialize());
    // Allow for controlLoop to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    JointPositions jointpos1({
        M_PI/2, -M_PI/2,
        M_PI/2, -M_PI/2,
        M_PI/2, -M_PI/2
    });
    std::vector<JointPositions> trajectory;
    trajectory.push_back(jointpos1);

    auto result = sut_->setPosition(trajectory);

    ASSERT_TRUE(result.valid());
    ASSERT_TRUE(result.get());

    for (unsigned int i=0; i < robotArmConfiguration_->getNumberOfJoints(); i++) {
        ASSERT_NEAR(jointpos1[i], sut_->getJointPositions()[i], 1e-2);
    }
}

TEST_F(RobotArmVelocityControllerShould, DISABLED_taskTrajectoryArrivingToDestinationTest) {
    sut_.reset(new RobotArmVelocityController(robotArmMock_));

    ASSERT_TRUE(sut_->initialize());
    // Allow for controlLoop to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    TaskPose offset(
        {0.05, 0, 0},
        Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0}));

    auto destination = multiply(offset, sut_->getTaskPose());

    std::vector<TaskPose> trajectory;
    trajectory.push_back(destination);
    auto result = sut_->setPosition(
        trajectory,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::JointSpace,
        crf::control::robotarmcontroller::PointReferenceFrame::Global);

    ASSERT_TRUE(result.valid());
    ASSERT_TRUE(result.get());

    for (int i=0; i < 3; i++) {
        ASSERT_NEAR(destination.getPosition()(i), sut_->getTaskPose().getPosition()(i), 1e-3);
    }
}

TEST_F(RobotArmVelocityControllerShould, DISABLED_taskLinearTrajectoryArrivingToDestinationJointLoopTest) { // NOLINT
    sut_.reset(new RobotArmVelocityController(robotArmMock_));

    ASSERT_TRUE(sut_->initialize());
    // Allow for controlLoop to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    TaskPose offset(
        {0.05, 0, 0},
        Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0}));

    auto destination = multiply(offset, sut_->getTaskPose());

    std::vector<TaskPose> trajectory;
    trajectory.push_back(destination);
    auto result = sut_->setPosition(
        trajectory,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::TaskSpaceJointLoop,
        crf::control::robotarmcontroller::PointReferenceFrame::Global);

    ASSERT_TRUE(result.valid());
    ASSERT_TRUE(result.get());

    for (int i=0; i < 3; i++) {
        ASSERT_NEAR(destination.getPosition()(i), sut_->getTaskPose().getPosition()(i), 1e-3);
    }
}

TEST_F(RobotArmVelocityControllerShould, DISABLED_taskLinearTrajectoryArrivingToDestinationTaskLoopTest) { // NOLINT
    sut_.reset(new RobotArmVelocityController(robotArmMock_));

    ASSERT_TRUE(sut_->initialize());
    // Allow for controlLoop to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    TaskPose offset(
        {0.05, 0, 0},
        Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0}));

    auto destination = multiply(offset, sut_->getTaskPose());

    std::vector<TaskPose> trajectory;
    trajectory.push_back(destination);
    auto result = sut_->setPosition(
        trajectory,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::TaskSpaceTaskLoop,// NOLINT
        crf::control::robotarmcontroller::PointReferenceFrame::Global);

    ASSERT_TRUE(result.valid());
    ASSERT_TRUE(result.get());

    for (int i=0; i < 3; i++) {
        ASSERT_NEAR(destination.getPosition()(i), sut_->getTaskPose().getPosition()(i), 1e-3);
    }
}

TEST_F(RobotArmVelocityControllerShould, setPositionJoints) {
    sut_.reset(new RobotArmVelocityController(robotArmMock_));
    ASSERT_TRUE(sut_->initialize());
    JointPositions target =
        crf::utility::types::JointPositions(sut_->getJointPositions().raw()*0.9);
    ASSERT_TRUE(sut_->setPosition(target).valid());
    ASSERT_TRUE(areAlmostEqual(target, sut_->getJointPositions(), 0.01));
}

TEST_F(RobotArmVelocityControllerShould, DISABLED_setPositionTask) {
    sut_.reset(new RobotArmVelocityController(robotArmMock_));
    ASSERT_TRUE(sut_->initialize());
    TaskPose target = multiply(sut_->getTaskPose(), TaskPose(
        {0.1, 0.1, 0.1},
        quaternionFromCardanXYZ(crf::math::rotation::CardanXYZ({0.1, 0.1, 0.1}))));
    ASSERT_TRUE(sut_->setPosition(
        target,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::JointSpace,
        crf::control::robotarmcontroller::PointReferenceFrame::Global).valid());
    ASSERT_TRUE(areAlmostEqual(target, sut_->getTaskPose()));
}

TEST_F(RobotArmVelocityControllerShould, failToSetPositionJointsForOutOfLimitsInput) {
    sut_.reset(new RobotArmVelocityController(robotArmMock_));
    ASSERT_TRUE(sut_->initialize());
    JointPositions target({100, 200, 300, 400, 500, 600});
    ASSERT_FALSE(sut_->setPosition(target).valid());
}

TEST_F(RobotArmVelocityControllerShould, failToSetPositionTaskForOutOfLimitsInput) {
    sut_.reset(new RobotArmVelocityController(robotArmMock_));
    ASSERT_TRUE(sut_->initialize());
    TaskPose target(
        {100, 200, 300},
        crf::math::rotation::CardanXYZ({400, 500, 600}));
    ASSERT_FALSE(sut_->setPosition(target,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::JointSpace,
        crf::control::robotarmcontroller::PointReferenceFrame::Global).valid());
}
