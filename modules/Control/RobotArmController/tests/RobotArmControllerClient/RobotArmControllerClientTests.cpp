/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *         Alejandro Diaz Rosales CERN BE/CEM/MRO
 * 
 *  ==================================================================================================
*/

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <optional>
#include <fstream>

#include <nlohmann/json.hpp>

#include "../RobotArmBehaviourForTests/RobotArmBehaviourForTests.hpp"

#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerCommunicationPointFactory.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerCommunicationPoint.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerManager.hpp"
#include "RobotArmController/RobotArmControllerClient/RobotArmControllerClient.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "Sockets/TCP/TCPServer.hpp"
#include "Sockets/TCP/TCPSocket.hpp"

#include "EventLogger/EventLogger.hpp"

using crf::control::robotarmcontroller::IRobotArmController;

class RobotArmControllerClientShould : public RobotArmBehaviourForTests {
 protected:
    RobotArmControllerClientShould():
        RobotArmBehaviourForTests(),
        net_port_(10000),
        simulatedJointForceTorques_(NUM_JOINTS),
        logger_("RobotArmControllerClientShould"),
        statusJSON_() {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        robotArmMock_.reset(new NiceMock<RobotArmMock>);
    }

    ~RobotArmControllerClientShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        robotArmMock_.reset(new NiceMock<crf::actuators::robotarm::RobotArmMock>);
        statusJSON_["taskAcceleration"] = std::vector<int>({0, 0, 0, 0, 0, 0});
        statusJSON_["taskVelocity"] = std::vector<int>({0, 0, 0, 0, 0, 0});
        statusJSON_["taskPose"] = std::vector<double>({-5.472004886541981e-06,-0.00979967787861824,1.2460999488830566,-7.240058948809747e-06,-7.239953902171692e-06,1.4479934179689735e-05});  // NOLINT
        statusJSON_["jointAccelerations"] = std::vector<int>(NUM_JOINTS, 0);
        statusJSON_["jointPositions"] = std::vector<int>(NUM_JOINTS, 0);
        statusJSON_["jointForceTorques"] = std::vector<int>(NUM_JOINTS, 0);
        statusJSON_["jointVelocities"] = std::vector<int>(NUM_JOINTS, 0);
        statusJSON_["priorityUnderControl"] = 0;
        statusJSON_["status"] = "initialized";
        statusJSON_["mode"] = 2;  // Velocity
        configureRobotArmDefaultBehavior();

        std::shared_ptr<crf::control::robotarmcontroller::RobotArmControllerManager> manager(
            new crf::control::robotarmcontroller::RobotArmControllerManager(robotArmMock_));

        std::shared_ptr<crf::control::robotarmcontroller::RobotArmControllerCommunicationPointFactory> communicationPointFactory( // NOLINT
            new crf::control::robotarmcontroller::RobotArmControllerCommunicationPointFactory(
                manager));

        std::shared_ptr<crf::communication::sockets::ISocketServer> server =
            std::make_shared<crf::communication::sockets::TCPServer>(net_port_);

        networkServer_.reset(
            new crf::communication::communicationpointserver::CommunicationPointServer(server,
            communicationPointFactory));

        networkServer_->initialize();

        std::shared_ptr<crf::communication::sockets::TCPSocket> code =
            std::make_shared<crf::communication::sockets::TCPSocket>("localhost", net_port_);

        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket(
            new crf::communication::datapacketsocket::PacketSocket(code));

        std::chrono::milliseconds server_reply_timeout(300);
        int priority_client = 1;
        float streamer_frequency = 0;
        sut_.reset(new crf::control::robotarmcontroller::RobotArmControllerClient(
            socket,
            server_reply_timeout,
            streamer_frequency,
            priority_client));
    }
    int net_port_;
    JointForceTorques simulatedJointForceTorques_;
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::control::robotarmcontroller::RobotArmControllerClient> sut_;
    nlohmann::json statusJSON_;
    std::unique_ptr<crf::communication::communicationpointserver::CommunicationPointServer>
        networkServer_;
};

TEST_F(RobotArmControllerClientShould, returnTrueIfInitializedOrDeinitializedTwice) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    // again ...
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(RobotArmControllerClientShould, allOperationsFailIfControllerIsNotInitializedTest) {
    ASSERT_FALSE(sut_->deinitialize());

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

    ASSERT_FALSE(sut_->setAcceleration(
        TaskAcceleration(),
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::TaskSpaceTaskLoop, // NOLINT
        crf::control::robotarmcontroller::PointReferenceFrame::Global));
    ASSERT_FALSE(sut_->setAcceleration(
        JointAccelerations(robotArmConfiguration_->getNumberOfJoints())));
}

TEST_F(RobotArmControllerClientShould, allOperationsFailOnWrongInputs) {
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

    ASSERT_FALSE(sut_->setAcceleration(
        JointAccelerations(robotArmConfiguration_->getNumberOfJoints()-1)));

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

TEST_F(RobotArmControllerClientShould, DISABLED_jointTrajectoryIsInterruptedCorrectlyTest) {
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

TEST_F(RobotArmControllerClientShould, DISABLED_taskTrajectoryIsInterruptedCorrectlyTest) {
    ASSERT_TRUE(sut_->initialize());
    // Allow for controlLoop to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    JointPositions jointpos1({
        0, -0.7,
        0.7, 0,
        0.7, 0
    });

    auto resultJoi = sut_->setPosition(jointpos1);

    ASSERT_TRUE(resultJoi.valid());
    ASSERT_TRUE(resultJoi.get());

    for (unsigned int i=0; i < robotArmConfiguration_->getNumberOfJoints(); i++) {
        ASSERT_NEAR(jointpos1[i], sut_->getJointPositions()[i], 1e-2);
    }

    TaskPose offset(
        {0.05, 0, 0},
        Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0}));

    auto result = sut_->setPosition(
        offset,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::JointSpace,
        crf::control::robotarmcontroller::PointReferenceFrame::TCP);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ASSERT_TRUE(sut_->interruptTrajectory());
    ASSERT_TRUE(result.valid());
    ASSERT_FALSE(result.get());
}

TEST_F(RobotArmControllerClientShould, DISABLED_taskTrajectoryLinearIsInterruptedCorrectlyTest) {
    ASSERT_TRUE(sut_->initialize());
    // Allow for controlLoop to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    JointPositions jointpos1({
        0, -M_PI/2,
        M_PI/2, 0,
        0, 0
    });

    auto resultJoi = sut_->setPosition(jointpos1);

    ASSERT_TRUE(resultJoi.valid());
    ASSERT_TRUE(resultJoi.get());

    for (unsigned int i=0; i < robotArmConfiguration_->getNumberOfJoints(); i++) {
        ASSERT_NEAR(jointpos1[i], sut_->getJointPositions()[i], 1e-2);
    }

    TaskPose offset(
        {0.05, 0, 0},
        Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0}));

    auto destination = multiply(offset, sut_->getTaskPose());
    auto result = sut_->setPosition(
        destination,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::TaskSpaceTaskLoop, // NOLINT
        crf::control::robotarmcontroller::PointReferenceFrame::Global);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ASSERT_TRUE(sut_->interruptTrajectory());
    ASSERT_TRUE(result.valid());
    ASSERT_FALSE(result.get());
}

TEST_F(RobotArmControllerClientShould, DISABLED_jointsTrajectoryArrivingToDestinationTest) {
    ASSERT_TRUE(sut_->initialize());
    // Allow for controlLoop to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    JointPositions jointpos1({
        0, -M_PI/2,
        M_PI/2, 0,
        0, 0
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

TEST_F(RobotArmControllerClientShould, DISABLED_taskTrajectoryArrivingToDestinationTest) {
    ASSERT_TRUE(sut_->initialize());
    // Allow for controlLoop to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    JointPositions jointpos1({
        0, -M_PI/2,
        M_PI/2, 0,
        0, 0
    });

    auto resultJoi = sut_->setPosition(jointpos1);

    ASSERT_TRUE(resultJoi.valid());
    ASSERT_TRUE(resultJoi.get());

    for (unsigned int i=0; i < robotArmConfiguration_->getNumberOfJoints(); i++) {
        ASSERT_NEAR(jointpos1[i], sut_->getJointPositions()[i], 1e-2);
    }

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

TEST_F(RobotArmControllerClientShould, DISABLED_taskLinearTrajectoryArrivingToDestinationJointLoopTest) { // NOLINT
    ASSERT_TRUE(sut_->initialize());
    // Allow for controlLoop to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    JointPositions jointpos1({
        0, -M_PI/2,
        M_PI/2, 0,
        0, 0
    });

    auto resultJoi = sut_->setPosition(jointpos1);

    ASSERT_TRUE(resultJoi.valid());
    ASSERT_TRUE(resultJoi.get());

    for (unsigned int i=0; i < robotArmConfiguration_->getNumberOfJoints(); i++) {
        ASSERT_NEAR(jointpos1[i], sut_->getJointPositions()[i], 1e-2);
    }

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

TEST_F(RobotArmControllerClientShould, DISABLED_taskLinearTrajectoryArrivingToDestinationTaskLoopTest) { // NOLINT
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

TEST_F(RobotArmControllerClientShould, failToSetPositionJointsForOutOfLimitsInput) {
    ASSERT_TRUE(sut_->initialize());
    JointPositions target({100, 200, 300, 400, 500, 600});
    ASSERT_FALSE(sut_->setPosition(target).valid());
}

TEST_F(RobotArmControllerClientShould, failToSetPositionTaskForOutOfLimitsInput) {
    ASSERT_TRUE(sut_->initialize());
    TaskPose target(
        {100, 200, 300},
        crf::math::rotation::CardanXYZ({400, 500, 600}));
    ASSERT_FALSE(sut_->setPosition(target,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::JointSpace,
        crf::control::robotarmcontroller::PointReferenceFrame::Global).valid());
}

TEST_F(RobotArmControllerClientShould, sendCorrectlyTheAccelerationButFailSinceNotImplemented) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setAcceleration(
        TaskAcceleration({0, 0, 0, 0, 0, 0}),
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::TaskSpaceTaskLoop, // NOLINT
        crf::control::robotarmcontroller::PointReferenceFrame::Global));
    ASSERT_FALSE(sut_->setAcceleration(
        JointAccelerations({0, 0, 0, 0, 0, 0})));
}



