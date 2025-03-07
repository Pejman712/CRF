/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#pragma once

#include "KinovaJacoAPI/IKinovaJacoAPIInterface.hpp"

namespace crf::communication::kinovajacoapi {

class KinovaJacoAPIInterface: public IKinovaJacoAPIInterface {
 public:
    ~KinovaJacoAPIInterface() override = default;
    int initEthernetAPI(EthernetCommConfig& config) override; // NOLINT
    int closeAPI() override;
    int startControlAPI() override;
    int stopControlAPI() override;
    int refresDevicesList() override;
    int getDevices(KinovaDevice devices[MAX_KINOVA_DEVICE], int& result) override; // NOLINT
    int setActiveDevice(KinovaDevice device) override;
    int moveHome() override;
    int initFingers() override;
    int eraseAllTrajectories() override;
    int activateSingularityAutomaticAvoidance(int state) override;
    int activateCollisionAutomaticAvoidance(int state) override;
    int getAngularPosition(AngularPosition& response) override; // NOLINT
    int getAngularVelocity(AngularPosition& response) override; // NOLINT
    int getAngularForce(AngularPosition& response) override; // NOLINT
    int getAngularForceGravityFree(AngularPosition& response) override; // NOLINT
    int getTaskPose(CartesianPosition& response) override; // NOLINT
    int getTaskForce(CartesianPosition& response) override; // NOLINT
    int getSensorInfo(SensorsInfo& response) override; // NOLINT
    int sendAdvanceTrajectory(TrajectoryPoint trajectory) override;
    int setTaskControl() override;
    int setAngularControl() override;
    int getControlType(int& response) override; // NOLINT
    int setTorqueZero(int ActuatorAdress) override;
    int startForceControl() override;
    int stopForceControl() override;
    int setAngularInertiaDamping(AngularInfo inertia, AngularInfo damping) override;
    int setAngularTorqueMinMax(AngularInfo min, AngularInfo max) override;
    int setTaskInertiaDamping(CartesianInfo inertia, CartesianInfo damping) override;
    int setTaskForceMinMax(CartesianInfo min, CartesianInfo max) override;
};

}  // namespace crf::communication::kinovajacoapi
