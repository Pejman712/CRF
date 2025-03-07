#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>

#include "KinovaJacoAPI/IKinovaJacoAPIInterface.hpp"

namespace crf::communication::kinovajacoapi {

class KinovaJacoAPIInterfaceMock : public IKinovaJacoAPIInterface {
 public:
    MOCK_METHOD(int, initEthernetAPI, (EthernetCommConfig& config), (override));  // NOLINT
    MOCK_METHOD(int, closeAPI, (), (override));
    MOCK_METHOD(int, startControlAPI, (), (override));
    MOCK_METHOD(int, stopControlAPI, (), (override));
    MOCK_METHOD(int, refresDevicesList, (), (override));
    MOCK_METHOD(int, getDevices, (KinovaDevice devices[MAX_KINOVA_DEVICE], int& result), (override));  // NOLINT
    MOCK_METHOD(int, setActiveDevice, (KinovaDevice device), (override));
    MOCK_METHOD(int, moveHome, (), (override));
    MOCK_METHOD(int, initFingers, (), (override));
    MOCK_METHOD(int, eraseAllTrajectories, (), (override));
    MOCK_METHOD(int, activateSingularityAutomaticAvoidance, (int state), (override));
    MOCK_METHOD(int, activateCollisionAutomaticAvoidance, (int state), (override));
    MOCK_METHOD(int, getAngularPosition, (AngularPosition& response), (override));  // NOLINT
    MOCK_METHOD(int, getAngularVelocity, (AngularPosition& response), (override));  // NOLINT
    MOCK_METHOD(int, getAngularForce, (AngularPosition& response), (override));  // NOLINT
    MOCK_METHOD(int, getAngularForceGravityFree, (AngularPosition& response), (override));  // NOLINT
    MOCK_METHOD(int, getTaskPose, (CartesianPosition& response), (override));  // NOLINT
    MOCK_METHOD(int, getTaskForce, (CartesianPosition& response), (override));  // NOLINT
    MOCK_METHOD(int, getSensorInfo, (SensorsInfo& response), (override));  // NOLINT
    MOCK_METHOD(int, sendAdvanceTrajectory, (TrajectoryPoint trajectory), (override));
    MOCK_METHOD(int, setTaskControl, (), (override));
    MOCK_METHOD(int, setAngularControl, (), (override));
    MOCK_METHOD(int, getControlType, (int& response), (override));  // NOLINT
    MOCK_METHOD(int, setTorqueZero, (int ActuatorAdress), (override));
    MOCK_METHOD(int, startForceControl, (), (override));
    MOCK_METHOD(int, stopForceControl, (), (override));
    MOCK_METHOD(int, setAngularInertiaDamping, (AngularInfo inertia, AngularInfo damping), (override));  // NOLINT
    MOCK_METHOD(int, setAngularTorqueMinMax, (AngularInfo min, AngularInfo max), (override));
    MOCK_METHOD(int, setTaskInertiaDamping, (CartesianInfo inertia, CartesianInfo damping), (override));  // NOLINT
    MOCK_METHOD(int, setTaskForceMinMax, (CartesianInfo min, CartesianInfo max), (override));
};

}  // namespace crf::communication::kinovajacoapi
