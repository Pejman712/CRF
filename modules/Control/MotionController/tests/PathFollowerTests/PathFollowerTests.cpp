/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <future>
#include <memory>
#include <string>
#include <chrono>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "MotionController/PathFollower/PathFollower.hpp"

#include "Robot/RobotMockConfiguration.hpp"
#include "InverseKinematics/OptCLIK/OptCLIK.hpp"
#include "Controller/PositionCtrlVelocityFF/PositionCtrlVelocityFF.hpp"
#include "TrajectoryGenerator/PointToPointJointsTrajectory/PointToPointJointsTrajectory.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::control::motioncontroller::PathFollower;

using crf::utility::types::Signals;
using crf::utility::types::JointSignals;
using crf::utility::types::TaskSignals;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;

using crf::actuators::robot::RobotConfiguration;
using crf::control::inversekinematics::ResultFlags;
using crf::control::controller::IController;
using crf::actuators::robot::RobotMockConfiguration;
using crf::control::trajectorygenerator::IJointTrajectoryGenerator;
using crf::control::trajectorygenerator::ITaskTrajectoryGenerator;
using crf::control::forwardkinematics::IForwardKinematics;
using crf::control::inversekinematics::IClosedLoopInverseKinematics;
using crf::math::geometricmethods::Sinusoid;
using crf::math::geometricmethods::ComputationMethod;

class PathFollowerShould: public ::testing::Test {
 protected:
    PathFollowerShould() :
        Kp_(6, 1.0),
        Ki_(6, 0.1),
        Kd_(6, 0.01),
        Ts_(2.0),
        logger_("PathFollowerShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~PathFollowerShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() {
        robot_ = std::make_shared<RobotMockConfiguration>(6);

        robot_->configureMock();
        robot_->setPosition(JointPositions({0, 0, 0, 0, 0, 0}));
        // robot_->setControlLoopTimeMs(std::chrono::milliseconds(2));
        EXPECT_CALL(*robot_, initialize()).Times(1);
        robot_->initialize();

        inverseKinematics_ =
            std::make_shared<crf::control::inversekinematics::OptCLIK>(
                JointPositions({0, 0, 0, 0, 0, 0}),
                robot_->getConfiguration()->getRobotControllerLoopTime(),
                robot_->getConfiguration(),
                std::vector<double>(robot_->getConfiguration()->getJointSpaceDoF(), 1));

        controller_ =
            std::make_shared<crf::control::controller::PositionCtrlVelocityFF>(
                Kp_, Ki_, Kd_, Ts_, inverseKinematics_);

        EXPECT_CALL(*robot_, getConfiguration()).Times(AtLeast(1));
        jointTrajGenerator_ =
            std::make_shared<crf::control::trajectorygenerator::PointToPointJointsTrajectory>(
                robot_->getConfiguration()->getProfileParameters().jointVelocities,
                robot_->getConfiguration()->getProfileParameters().jointAccelerations);

        sut_ = std::make_unique<PathFollower>(
            robot_,
            controller_,
            jointTrajGenerator_,
            taskTrajGenerator_);
    }

    std::vector<double> Kp_;
    std::vector<double> Ki_;
    std::vector<double> Kd_;
    double Ts_;

    std::unique_ptr<PathFollower> sut_;

    std::shared_ptr<RobotMockConfiguration> robot_;
    std::shared_ptr<IController> controller_;
    std::shared_ptr<IJointTrajectoryGenerator> jointTrajGenerator_;
    std::shared_ptr<ITaskTrajectoryGenerator> taskTrajGenerator_;
    std::shared_ptr<IForwardKinematics> forwardKinematics_;
    std::shared_ptr<IClosedLoopInverseKinematics> inverseKinematics_;

    crf::utility::logger::EventLogger logger_;
};

TEST_F(PathFollowerShould, appendJointsPathCorrectlyAndFollowTheTrajectory) {
    EXPECT_CALL(*robot_, getJointVelocities()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointPositions()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointAccelerations()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointForceTorques()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getTaskForceTorque()).Times(AtLeast(1));

    EXPECT_CALL(*robot_, setJointVelocities(_, _, _)).Times(AtLeast(1));

    ASSERT_TRUE(sut_->appendPath({JointPositions({2, 1, 1, -0.5, -1, 0.5})}));

    while (!crf::utility::types::areAlmostEqual(
            sut_->getSignals().joints.positions.value(),
            JointPositions({2, 1, 1, -0.5, -1, 0.5}))) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    EXPECT_CALL(*robot_, softStop()).Times(1);
}

TEST_F(PathFollowerShould, failToAppendIfInputIsWrong) {
    ASSERT_FALSE(sut_->appendPath({JointPositions({0})}));
}

TEST_F(PathFollowerShould, failOnNotAllowedMethods) {
    crf::expected<bool> res = sut_->setVelocity(JointVelocities({0}));
    ASSERT_FALSE(res);
    ASSERT_EQ(res.get_response(), crf::Code::MethodNotAllowed);

    res = sut_->setTorque(JointForceTorques({0}));
    ASSERT_FALSE(res);
    ASSERT_EQ(res.get_response(), crf::Code::MethodNotAllowed);
}

TEST_F(PathFollowerShould, profileVelocityAndAccelerationShouldNotBeExceededInATraj) {
    EXPECT_CALL(*robot_, getJointVelocities()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointPositions()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointAccelerations()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointForceTorques()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getTaskForceTorque()).Times(AtLeast(1));

    EXPECT_CALL(*robot_, setJointVelocities(_, _, _)).Times(AtLeast(1));

    ASSERT_TRUE(sut_->setProfileVelocity(JointVelocities({1, 1, 1, 1, 1, 1})));

    ASSERT_TRUE(sut_->setProfileAcceleration(JointAccelerations({2, 2, 2, 2, 2, 2})));

    ASSERT_TRUE(sut_->appendPath({JointPositions({2, 1, 1, -0.5, -1, 0.5})}));

    while (!crf::utility::types::areAlmostEqual(
            sut_->getSignals().joints.positions.value(),
            JointPositions({2, 1, 1, -0.5, -1, 0.5}))) {
        // We add a tolerance of 0.001 to the values since doubles have small deviations
        ASSERT_TRUE(isLesser(sut_->getSignals().joints.velocities.value(), JointVelocities({1.001, 1.001, 1.001, 1.001, 1.001, 1.001})));  // NOLINT
        ASSERT_TRUE(isLesser(sut_->getSignals().joints.accelerations.value(), JointAccelerations({2.001, 2.001, 2.001, 2.001, 2.001, 2.001})));  // NOLINT

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    EXPECT_CALL(*robot_, softStop()).Times(1);
}

TEST_F(PathFollowerShould, stopTrajectoryWhenSoftStopIsCalled) {
    EXPECT_CALL(*robot_, getJointVelocities()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointPositions()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointAccelerations()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointForceTorques()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getTaskForceTorque()).Times(AtLeast(1));

    EXPECT_CALL(*robot_, setJointVelocities(_, _, _)).Times(AtLeast(1));

    ASSERT_TRUE(sut_->setProfileVelocity(JointVelocities({1, 1, 1, 1, 1, 1})));

    ASSERT_TRUE(sut_->setProfileAcceleration(JointAccelerations({2, 2, 2, 2, 2, 2})));

    ASSERT_TRUE(sut_->appendPath({JointPositions({2, 1, 1, -0.5, -1, 0.5})}));

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    EXPECT_CALL(*robot_, softStop()).Times(1);

    sut_->softStop();
}

TEST_F(PathFollowerShould, stopTrajectoryWhenHardStopIsCalled) {
    EXPECT_CALL(*robot_, getJointVelocities()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointPositions()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointAccelerations()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointForceTorques()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getTaskForceTorque()).Times(AtLeast(1));

    EXPECT_CALL(*robot_, setJointVelocities(_, _, _)).Times(AtLeast(1));

    ASSERT_TRUE(sut_->setProfileVelocity(JointVelocities({1, 1, 1, 1, 1, 1})));

    ASSERT_TRUE(sut_->setProfileAcceleration(JointAccelerations({2, 2, 2, 2, 2, 2})));

    ASSERT_TRUE(sut_->appendPath({JointPositions({2, 1, 1, -0.5, -1, 0.5})}));

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    EXPECT_CALL(*robot_, hardStop()).Times(1);

    sut_->hardStop();
}

TEST_F(PathFollowerShould, beAbleToReadDataFromRobot) {
    EXPECT_CALL(*robot_, getJointVelocities()).Times(1);
    EXPECT_CALL(*robot_, getJointPositions()).Times(1);
    EXPECT_CALL(*robot_, getJointAccelerations()).Times(1);
    EXPECT_CALL(*robot_, getJointForceTorques()).Times(1);

    Signals sig = sut_->getSignals();

    ASSERT_TRUE(sig.joints.positions);
    ASSERT_TRUE(sig.joints.velocities);

    // ASSERT_TRUE(sig.task.position);
    // ASSERT_TRUE(sig.task.velocity);
}

TEST_F(PathFollowerShould, ReturnCorrectlyIsTrajectoryRunning) {
    EXPECT_CALL(*robot_, getJointVelocities()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointPositions()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointAccelerations()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointForceTorques()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getTaskForceTorque()).Times(AtLeast(1));

    EXPECT_CALL(*robot_, setJointVelocities(_, _, _)).Times(AtLeast(1));

    ASSERT_TRUE(sut_->setProfileVelocity(JointVelocities({1, 1, 1, 1, 1, 1})));

    ASSERT_TRUE(sut_->setProfileAcceleration(JointAccelerations({2, 2, 2, 2, 2, 2})));

    ASSERT_FALSE(sut_->isTrajectoryRunning().value());

    ASSERT_TRUE(sut_->appendPath({JointPositions({2, 1, 1, -0.5, -1, 0.5})}));

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ASSERT_TRUE(sut_->isTrajectoryRunning().value());

    EXPECT_CALL(*robot_, softStop()).Times(1);
}
