/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <shared_mutex>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "crf/expected.hpp"

#include "KinovaJacoAPI/KinovaJacoAPIInterfaceMock.hpp"

#include "Types/Types.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::An;

using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointForceTorques;
using crf::utility::types::areAlmostEqual;

namespace crf::communication::kinovajacoapi {

class KinovaJacoAPIInterfaceMockConfiguration : public KinovaJacoAPIInterfaceMock {
 public:
    KinovaJacoAPIInterfaceMockConfiguration():
        controlMode_(0),
        logger_("KinovaJacoAPIInterfaceMockConfiguration") {
            logger_->info("CTor");
    }

    ~KinovaJacoAPIInterfaceMockConfiguration() {
        logger_->info("DTor");
    }

    int testAddDevicesList(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result) {  // NOLINT
        result = 1;
        snprintf(devices[0].SerialNumber, SERIAL_LENGTH, "SomeBullshit");
        snprintf(devices[1].SerialNumber, SERIAL_LENGTH, "PJ00900006163400002");
        snprintf(devices[2].SerialNumber, SERIAL_LENGTH, "PJ00900006509031-0 ");
        return 3;
    }
    void testFakeJointPositions(AngularPosition& position) { // NOLINT
        position.Actuators.Actuator1 = -90;
        position.Actuators.Actuator2 = 90;
        position.Actuators.Actuator3 = 90;
        position.Actuators.Actuator4 = 90;
        position.Actuators.Actuator5 = 90;
        position.Actuators.Actuator6 = -90;
    }
    void testFakeJointVelocities(AngularPosition& velocity) { // NOLINT
        velocity.Actuators.Actuator1 = -5;
        velocity.Actuators.Actuator2 = 5;
        velocity.Actuators.Actuator3 = 5;
        velocity.Actuators.Actuator4 = 5;
        velocity.Actuators.Actuator5 = 5;
        velocity.Actuators.Actuator6 = -5;
    }
    void testFakeJointForceTorques(AngularPosition& torque) { // NOLINT
        torque.Actuators.Actuator1 = -5;
        torque.Actuators.Actuator2 = 5;
        torque.Actuators.Actuator3 = 5;
        torque.Actuators.Actuator4 = 5;
        torque.Actuators.Actuator5 = 5;
        torque.Actuators.Actuator6 = -5;
    }
    void testFakeArmPosition(CartesianPosition& position) { // NOLINT
        position.Coordinates.X = -0.3f;
        position.Coordinates.Y = -0.3f;
        position.Coordinates.Z = 0.2f;
        position.Coordinates.ThetaX = 0.1f;
        position.Coordinates.ThetaY = 0.2f;
        position.Coordinates.ThetaZ = 0.2f;
    }

    void configureMock() {
        ON_CALL(*this, initEthernetAPI(_)).WillByDefault(Invoke(
            [this](EthernetCommConfig& config) {
                return 1;
            }));
        ON_CALL(*this, closeAPI()).WillByDefault(Invoke(
            [this] {
                return 1;
            }));
        ON_CALL(*this, startControlAPI()).WillByDefault(Invoke(
            [this] {
                return 1;
            }));
        ON_CALL(*this, stopControlAPI()).WillByDefault(Invoke(
            [this] {
                return 1;
            }));
        ON_CALL(*this, refresDevicesList()).WillByDefault(Invoke(
            [this] {
                return 1;
            }));
        ON_CALL(*this, getDevices(_, _)).WillByDefault(Invoke(
            [this](KinovaDevice devices[MAX_KINOVA_DEVICE], int &result) {
                return testAddDevicesList(devices, result);
            }));
        ON_CALL(*this, setActiveDevice(_)).WillByDefault(Invoke(
            [this](KinovaDevice device) {
                return 1;
            }));
        ON_CALL(*this, initFingers()).WillByDefault(Invoke(
            [this] {
                return 1;
            }));
        ON_CALL(*this, eraseAllTrajectories()).WillByDefault(Invoke(
            [this] {
                return 1;
            }));
        ON_CALL(*this, activateSingularityAutomaticAvoidance(_)).WillByDefault(Invoke(
            [this](int state) {
                return 1;
            }));
        ON_CALL(*this, activateCollisionAutomaticAvoidance(_)).WillByDefault(Invoke(
            [this](int state) {
                return 1;
            }));
        ON_CALL(*this, getAngularPosition(_)).WillByDefault(Invoke(
            [this](AngularPosition& response) {
                testFakeJointPositions(response);
                return 1;
            }));
        ON_CALL(*this, getAngularVelocity(_)).WillByDefault(Invoke(
            [this](AngularPosition& response) {
                testFakeJointVelocities(response);
                return 1;
            }));
        ON_CALL(*this, getAngularForce(_)).WillByDefault(Invoke(
            [this](AngularPosition& response) {
                testFakeJointForceTorques(response);
                return 1;
            }));
        ON_CALL(*this, getAngularForceGravityFree(_)).WillByDefault(Invoke(
            [this](AngularPosition& response) {
                testFakeJointForceTorques(response);
                return 1;
            }));
        ON_CALL(*this, getTaskPose(_)).WillByDefault(Invoke(
            [this](CartesianPosition& response) {
                testFakeArmPosition(response);
                return 1;
            }));
        ON_CALL(*this, sendAdvanceTrajectory(_)).WillByDefault(Invoke(
            [this](const TrajectoryPoint& trajectory) {
                return 1;
            }));
        ON_CALL(*this, setTaskControl()).WillByDefault(Invoke(
            [this] {
                controlMode_ = 0;
                return 1;
            }));
        ON_CALL(*this, setAngularControl()).WillByDefault(Invoke([this] {
                controlMode_ = 1;
                return 1;
            }));
        ON_CALL(*this, moveHome()).WillByDefault(Invoke(
            [this] {
                return 1;
            }));
        ON_CALL(*this, getControlType(_)).WillByDefault(Invoke(
            [this](int& response) {
                response = controlMode_;
                return 1;
            }));
    }

 private:
    int controlMode_;
    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::communication::kinovajacoapi
