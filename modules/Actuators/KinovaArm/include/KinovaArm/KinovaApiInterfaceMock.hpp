#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>

#include "KinovaArm/IKinovaApiInterface.hpp"

namespace crf::actuators::kinovaarm {

class KinovaApiInterfaceMock : public IKinovaApiInterface {
 public:
    MOCK_METHOD1(ethernetInitEthernetAPI,
        int(EthernetCommConfig& config));  // NOLINT
    MOCK_METHOD0(ethernetCloseAPI,
        int());
    MOCK_METHOD0(ethernetStartControlAPI,
        int());
    MOCK_METHOD0(ethernetStopControlAPI,
        int());
    MOCK_METHOD0(ethernetRefresDevicesList,
        int());
    MOCK_METHOD2(ethernetGetDevices,
        int(KinovaDevice devices[MAX_KINOVA_DEVICE], int& result));  // NOLINT
    MOCK_METHOD1(ethernetSetActiveDevice,
        int(KinovaDevice device));
    MOCK_METHOD0(ethernetMoveHome,
        int());
    MOCK_METHOD0(ethernetInitFingers,
        int());
    MOCK_METHOD0(ethernetEraseAllTrajectories,
        int());
    MOCK_METHOD1(ethernetActivateSingularityAutomaticAvoidance,
        int(int state));
    MOCK_METHOD1(ethernetActivateCollisionAutomaticAvoidance,
        int(int state));
    MOCK_METHOD1(ethernetGetAngularPosition,
        int(AngularPosition& response));  // NOLINT
    MOCK_METHOD1(ethernetGetAngularVelocity,
        int(AngularPosition& response));  // NOLINT
    MOCK_METHOD1(ethernetGetAngularForce,
        int(AngularPosition& response));  // NOLINT
    MOCK_METHOD1(ethernetGetAngularForceGravityFree,
        int(AngularPosition& response));  // NOLINT
    MOCK_METHOD1(ethernetGetTaskPose,
        int(CartesianPosition& response));  // NOLINT
    MOCK_METHOD1(ethernetGetTaskForce,
        int(CartesianPosition& response));  // NOLINT
    MOCK_METHOD1(ethernetGetSensorInfo,
        int(SensorsInfo& response));  // NOLINT
    MOCK_METHOD1(ethernetSendAdvanceTrajectory,
        int(TrajectoryPoint trajectory));
    MOCK_METHOD0(ethernetSetTaskControl,
        int());
    MOCK_METHOD0(ethernetSetAngularControl,
        int());
    MOCK_METHOD1(ethernetGetControlType,
        int(int& response));  // NOLINT
    MOCK_METHOD1(ethernetSetTorqueZero,
        int(int ActuatorAdress));
    MOCK_METHOD0(startForceControl,
        int());
    MOCK_METHOD0(stopForceControl,
        int());
    MOCK_METHOD2(setAngularInertiaDamping,
        int(AngularInfo inertia, AngularInfo damping));
    MOCK_METHOD2(setAngularTorqueMinMax,
        int(AngularInfo min, AngularInfo max));
    MOCK_METHOD2(setTaskInertiaDamping,
        int(CartesianInfo inertia, CartesianInfo damping));
    MOCK_METHOD2(setTaskForceMinMax,
        int(CartesianInfo min, CartesianInfo max));
};

}  // namespace crf::actuators::kinovaarm
