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

#include "MotionController/Teleoperation/Teleoperation.hpp"

#include "Robot/RobotMockConfiguration.hpp"
#include "Controller/DirectOpenLoopVelocity/DirectOpenLoopVelocity.hpp"
#include "TrajectoryGenerator/PointToPointJointsTrajectory/PointToPointJointsTrajectory.hpp"
#include "ForwardKinematics/MathExprForwardKinematics/MathExprForwardKinematics.hpp"
#include "InverseKinematics/OptOLIK/OptOLIK.hpp"
#include "InverseKinematics/JointLimits/JointLimits.hpp"
#include "InverseKinematics/DesiredJointPositions/DesiredJointPositions.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::control::motioncontroller::Teleoperation;

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
using crf::control::forwardkinematics::IForwardKinematics;
using crf::math::geometricmethods::Sinusoid;
using crf::math::geometricmethods::ComputationMethod;

class TeleoperationShould: public ::testing::Test {
 protected:
    TeleoperationShould() :
        logger_("TeleoperationShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~TeleoperationShould() {
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

        EXPECT_CALL(*robot_, getConfiguration()).Times(AtLeast(0));

        // Create Forward Kinematics
        forwardKinematics_ = robot_->getConfiguration()->getForwardKinematics();

        // Create Objective Functions
        // Joint limits
        vecObjFun_.push_back(std::make_shared<crf::control::inversekinematics::JointLimits>(
            10.0,  // 10s for the range of the sinusoid
            static_cast<double>(robot_->getConfiguration()->getRobotControllerLoopTime().count())/1000,  // NOLINT
            1.0,  // Unitary exponential function
            0.1,  // Low proportional gain
            robot_->getConfiguration()->getJointLimits().minPosition,
            robot_->getConfiguration()->getJointLimits().maxPosition));

        // Desired Joint Position
        vecObjFun_.push_back(std::make_shared<crf::control::inversekinematics::DesiredJointPositions>(  // NOLINT
            10.0,  // 10s for the range of the sinusoid
            static_cast<double>(robot_->getConfiguration()->getRobotControllerLoopTime().count())/1000,  // NOLINT
            1.0));  // Unitary exponential function

        // Inverse Kinematics
        inverseKinematics_ = std::make_shared<crf::control::inversekinematics::OptOLIK>(
            robot_->getConfiguration(),
            std::vector<double>(robot_->getConfiguration()->getJointSpaceDoF(), 1),
            vecObjFun_);

        controller_ = std::make_shared<crf::control::controller::DirectOpenLoopVelocity>(
            robot_->getConfiguration()->getJointSpaceDoF(), inverseKinematics_, vecObjFun_);

        sut_ = std::make_unique<Teleoperation>(robot_, controller_);
    }

    std::unique_ptr<Teleoperation> sut_;

    std::shared_ptr<RobotMockConfiguration> robot_;
    std::shared_ptr<IController> controller_;
    std::shared_ptr<IForwardKinematics> forwardKinematics_;
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>>
        vecObjFun_;
    std::shared_ptr<IOpenLoopInverseKinematics> inverseKinematics_;

    crf::utility::logger::EventLogger logger_;
};

TEST_F(TeleoperationShould, setJointVelocitiesCorrectly) {
    EXPECT_CALL(*robot_, getJointVelocities()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointPositions()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointAccelerations()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointForceTorques()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, setJointVelocities(_, _, _)).Times(AtLeast(1));

    ASSERT_TRUE(sut_->setVelocity({JointVelocities({0.1, 0.1, 0.1, 0.1, 0.1, 0.1})}));

    EXPECT_CALL(*robot_, softStop()).Times(1);
}

TEST_F(TeleoperationShould, setTaskVelocityCorrectly) {
    EXPECT_CALL(*robot_, getJointVelocities()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointPositions()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointAccelerations()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointForceTorques()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, setJointVelocities(_, _, _)).Times(AtLeast(1));

    ASSERT_TRUE(sut_->setVelocity(
        {TaskVelocity({0.1, 0.1, 0.1, 0.1, 0.1, 0.1})},
        crf::control::motioncontroller::PointReferenceFrame::Global));
    EXPECT_CALL(*robot_, softStop()).Times(1);
}

TEST_F(TeleoperationShould, callSoftStopCorrectly) {
    EXPECT_CALL(*robot_, getJointVelocities()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointPositions()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointAccelerations()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointForceTorques()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, setJointVelocities(_, _, _)).Times(AtLeast(1));

    ASSERT_TRUE(sut_->setVelocity({JointVelocities({0.1, 0.1, 0.1, 0.1, 0.1, 0.1})}));

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    EXPECT_CALL(*robot_, softStop()).Times(1);

    sut_->softStop();
}

TEST_F(TeleoperationShould, callHardStopCorrectly) {
    EXPECT_CALL(*robot_, getJointVelocities()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointPositions()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointAccelerations()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, getJointForceTorques()).Times(AtLeast(1));
    EXPECT_CALL(*robot_, setJointVelocities(_, _, _)).Times(AtLeast(1));

    ASSERT_TRUE(sut_->setVelocity({JointVelocities({0.1, 0.1, 0.1, 0.1, 0.1, 0.1})}));

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    EXPECT_CALL(*robot_, hardStop()).Times(1);

    sut_->hardStop();
}

TEST_F(TeleoperationShould, ReturnErrorCodeWithTrajectory) {
    ASSERT_EQ(sut_->isTrajectoryRunning().get_response(), crf::Code::MethodNotAllowed);
}
