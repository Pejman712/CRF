#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "KinovaArm/IKinovaApiInterface.hpp"

namespace crf::actuators::kinovaarm {

class KinovaApiInterface: public IKinovaApiInterface {
 public:
    ~KinovaApiInterface() override = default;
    int ethernetInitEthernetAPI(EthernetCommConfig& config) override; // NOLINT
    int ethernetCloseAPI() override;
    int ethernetStartControlAPI() override;
    int ethernetStopControlAPI() override;
    int ethernetRefresDevicesList() override;
    int ethernetGetDevices(KinovaDevice devices[MAX_KINOVA_DEVICE], int& result) override; // NOLINT
    int ethernetSetActiveDevice(KinovaDevice device) override;
    int ethernetMoveHome() override;
    int ethernetInitFingers() override;
    int ethernetEraseAllTrajectories() override;
    int ethernetActivateSingularityAutomaticAvoidance(int state) override;
    int ethernetActivateCollisionAutomaticAvoidance(int state) override;
    int ethernetGetAngularPosition(AngularPosition& response) override; // NOLINT
    int ethernetGetAngularVelocity(AngularPosition& response) override; // NOLINT
    int ethernetGetAngularForce(AngularPosition& response) override; // NOLINT
    int ethernetGetAngularForceGravityFree(AngularPosition& response) override; // NOLINT
    int ethernetGetTaskPose(CartesianPosition& response) override; // NOLINT
    int ethernetGetTaskForce(CartesianPosition& response) override; // NOLINT
    int ethernetGetSensorInfo(SensorsInfo& response) override; // NOLINT
    int ethernetSendAdvanceTrajectory(TrajectoryPoint trajectory) override;
    int ethernetSetTaskControl() override;
    int ethernetSetAngularControl() override;
    int ethernetGetControlType(int& response) override; // NOLINT
    int ethernetSetTorqueZero(int ActuatorAdress) override;
    int startForceControl() override;
    int stopForceControl() override;
    int setAngularInertiaDamping(AngularInfo inertia, AngularInfo damping) override;
    int setAngularTorqueMinMax(AngularInfo min, AngularInfo max) override;
    int setTaskInertiaDamping(CartesianInfo inertia, CartesianInfo damping) override;
    int setTaskForceMinMax(CartesianInfo min, CartesianInfo max) override;
};

}  // namespace crf::actuators::kinovaarm
