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

// CommPoint
#include "Sockets/TCP/TCPServer.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "MotionController/MotionControllerCommunicationPoint/MotionControllerCommunicationPoint.hpp"
#include "MotionController/MotionControllerCommunicationPoint/MotionControllerCommunicationPointFactory.hpp"
#include "MotionController/MotionControllerCommunicationPoint/MotionControllerManager.hpp"

// Client
#include "Sockets/TCP/TCPSocket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "MotionController/MotionControllerClient/MotionControllerClient.hpp"

// Mock
#include "MotionController/MotionControllerMockConfiguration.hpp"

// Logger
#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::control::motioncontroller::MotionControllerCommunicationPoint;
using crf::control::motioncontroller::MotionControllerManager;
using crf::control::motioncontroller::MotionControllerMockConfiguration;

class MotionControllerClientTests: public ::testing::Test {
 protected:
    MotionControllerClientTests() :
        logger_("MotionControllerClientTests") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~MotionControllerClientTests() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() {
        std::shared_ptr<MotionControllerMockConfiguration> controller =
            std::make_shared<MotionControllerMockConfiguration>(6);

        controller->configureMock();

        std::shared_ptr<crf::control::motioncontroller::MotionControllerManager> manager =
            std::make_shared<crf::control::motioncontroller::MotionControllerManager>(controller);

        std::shared_ptr<crf::control::motioncontroller::MotionControllerCommunicationPointFactory> factory =  // NOLINT
            std::make_shared<crf::control::motioncontroller::MotionControllerCommunicationPointFactory>(  // NOLINT
                manager);

        std::shared_ptr<crf::communication::sockets::ISocketServer> socket =
            std::make_shared<crf::communication::sockets::TCPServer>(port_);

        server_ = std::make_shared<crf::communication::communicationpointserver::CommunicationPointServer>(  // NOLINT
            socket,
            factory);

        if (!server_->initialize()) {
            throw std::runtime_error(
                "MotionControllerClientTests - SetUp - Communication point server could not be initialized!");  // NOLINT
        }

        // Client
        std::shared_ptr<crf::communication::sockets::TCPSocket> clientSocket =
            std::make_shared<crf::communication::sockets::TCPSocket>(
                host_,
                port_);

        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> clientPacketSocket =
            std::make_shared<crf::communication::datapacketsocket::PacketSocket>(
                clientSocket);

        sut_ = std::make_unique<crf::control::motioncontroller::MotionControllerClient>(
            clientPacketSocket,
            std::chrono::milliseconds(timeoutMs_),
            frequency_,
            priority_);
    }

    const uint32_t port_ = 10000;
    const std::string host_ = "localhost";
    const uint32_t timeoutMs_ = 3000;
    const uint32_t priority_ = 60;
    const float frequency_ = 0;

    std::shared_ptr<crf::communication::communicationpointserver::CommunicationPointServer> server_;
    std::unique_ptr<crf::control::motioncontroller::MotionControllerClient> sut_;
    crf::utility::logger::EventLogger logger_;
};

TEST_F(MotionControllerClientTests, initializeAndDeinitialize) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(MotionControllerClientTests, tryAppendPathJoints) {
    ASSERT_TRUE(sut_->initialize());

    crf::expected<bool> result = sut_->appendPath(
        {crf::utility::types::JointPositions({0, 0, 0, 0, 0, 0})});

    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());

    crf::expected<bool> traj = sut_->isTrajectoryRunning();

    ASSERT_TRUE(traj);
    ASSERT_FALSE(traj.value());
}

TEST_F(MotionControllerClientTests, tryAppendPathTask) {
    ASSERT_TRUE(sut_->initialize());

    crf::expected<bool> result = sut_->appendPath(
        {crf::utility::types::TaskPose({0.1, 0, 0, 0, 0, 0},
        crf::math::rotation::OrientationRepresentation::CardanXYZ)},
        crf::control::motioncontroller::TrajectoryExecutionMethod::TaskSpace,
        crf::control::motioncontroller::PointReferenceFrame::TCP);

    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
}

TEST_F(MotionControllerClientTests, trySetVelocityJoints) {
    ASSERT_TRUE(sut_->initialize());

    crf::expected<bool> result = sut_->setVelocity(
        crf::utility::types::JointVelocities({0, 0, 0, 0, 0, 0}));

    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
}

TEST_F(MotionControllerClientTests, trySetVelocityTask) {
    ASSERT_TRUE(sut_->initialize());

    crf::expected<bool> result = sut_->setVelocity(
        crf::utility::types::TaskVelocity({0, 0, 0, 0, 0, 0}),
        crf::control::motioncontroller::PointReferenceFrame::TCP);

    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
}

TEST_F(MotionControllerClientTests, trySetTorqueJoints) {
    ASSERT_TRUE(sut_->initialize());

    crf::expected<bool> result = sut_->setTorque(
        crf::utility::types::JointForceTorques({0, 0, 0, 0, 0, 0}));

    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
}

TEST_F(MotionControllerClientTests, trySetTorqueTask) {
    ASSERT_TRUE(sut_->initialize());

    crf::expected<bool> result = sut_->setTorque(
        crf::utility::types::TaskForceTorque({0, 0, 0, 0, 0, 0}),
        crf::control::motioncontroller::PointReferenceFrame::TCP);

    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
}

TEST_F(MotionControllerClientTests, trySetProfileVelJoints) {
    ASSERT_TRUE(sut_->initialize());

    crf::expected<bool> result = sut_->setProfileVelocity(
        crf::utility::types::JointVelocities({0, 0, 0, 0, 0, 0}));

    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
}

TEST_F(MotionControllerClientTests, trySetProfileVelTask) {
    ASSERT_TRUE(sut_->initialize());

    crf::expected<bool> result = sut_->setProfileVelocity(
        crf::utility::types::TaskVelocity({0, 0, 0, 0, 0, 0}));

    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
}

TEST_F(MotionControllerClientTests, trySetProfileAccJoints) {
    ASSERT_TRUE(sut_->initialize());

    crf::expected<bool> result = sut_->setProfileAcceleration(
        crf::utility::types::JointAccelerations({0, 0, 0, 0, 0, 0}));

    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
}

TEST_F(MotionControllerClientTests, trySetProfileAccTask) {
    ASSERT_TRUE(sut_->initialize());

    crf::expected<bool> result = sut_->setProfileAcceleration(
        crf::utility::types::TaskAcceleration({0, 0, 0, 0, 0, 0}));

    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
}

TEST_F(MotionControllerClientTests, trySoftStop) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_NO_THROW(sut_->softStop());
}

TEST_F(MotionControllerClientTests, tryHardStop) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_NO_THROW(sut_->hardStop());
}

TEST_F(MotionControllerClientTests, tryGetSignals) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_NO_THROW(sut_->getSignals());
}

TEST_F(MotionControllerClientTests, tryGetControllerStatus) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_NO_THROW(sut_->getSignals());
}
