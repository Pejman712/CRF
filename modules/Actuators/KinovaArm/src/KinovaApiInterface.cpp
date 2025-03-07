/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <KinovaTypes.h>
#include <Kinova.API.EthCommandLayerUbuntu.h>

#include "KinovaArm/KinovaApiInterface.hpp"

namespace crf::actuators::kinovaarm {

int KinovaApiInterface::ethernetInitEthernetAPI(EthernetCommConfig& config) {
    return Ethernet_InitEthernetAPI(config);
}

int KinovaApiInterface::ethernetCloseAPI() {
    return Ethernet_CloseAPI();
}

int KinovaApiInterface::ethernetStartControlAPI() {
    return Ethernet_StartControlAPI();
}

int KinovaApiInterface::ethernetStopControlAPI() {
    return Ethernet_StopControlAPI();
}

int KinovaApiInterface::ethernetRefresDevicesList() {
    return Ethernet_RefresDevicesList();
}

int KinovaApiInterface::ethernetGetDevices(KinovaDevice devices[MAX_KINOVA_DEVICE], int& result) {
    return Ethernet_GetDevices(devices, result);
}

int KinovaApiInterface::ethernetSetActiveDevice(KinovaDevice device) {
    return Ethernet_SetActiveDevice(device);
}

int KinovaApiInterface::ethernetMoveHome() {
    return Ethernet_MoveHome();
}

int KinovaApiInterface::ethernetInitFingers() {
    return Ethernet_InitFingers();
}

int KinovaApiInterface::ethernetEraseAllTrajectories() {
    return Ethernet_EraseAllTrajectories();
}

int KinovaApiInterface::ethernetActivateSingularityAutomaticAvoidance(int state) {
    return Ethernet_ActivateSingularityAutomaticAvoidance(state);
}

int KinovaApiInterface::ethernetActivateCollisionAutomaticAvoidance(int state) {
    return Ethernet_ActivateCollisionAutomaticAvoidance(state);
}

int KinovaApiInterface::ethernetGetAngularPosition(AngularPosition& response) {
    return Ethernet_GetAngularPosition(response);
}

int KinovaApiInterface::ethernetGetAngularVelocity(AngularPosition& response) {
    return Ethernet_GetAngularVelocity(response);
}

int KinovaApiInterface::ethernetGetAngularForce(AngularPosition& response) {
    return Ethernet_GetAngularForce(response);
}

int KinovaApiInterface::ethernetGetAngularForceGravityFree(AngularPosition& response) {
    return Ethernet_GetAngularForceGravityFree(response);
}

int KinovaApiInterface::ethernetGetTaskPose(CartesianPosition& response) {
    return Ethernet_GetCartesianPosition(response);
}

int KinovaApiInterface::ethernetGetTaskForce(CartesianPosition& response) {
    return Ethernet_GetCartesianForce(response);
}

int KinovaApiInterface::ethernetGetSensorInfo(SensorsInfo& response) {
    return Ethernet_GetSensorsInfo(response);
}

int KinovaApiInterface::ethernetSendAdvanceTrajectory(TrajectoryPoint trajectory) {
    return Ethernet_SendAdvanceTrajectory(trajectory);
}

int KinovaApiInterface::ethernetSetTaskControl() {
    return Ethernet_SetCartesianControl();
}

int KinovaApiInterface::ethernetSetAngularControl() {
    return Ethernet_SetAngularControl();
}

int KinovaApiInterface::ethernetGetControlType(int& response) {
    return Ethernet_GetControlType(response);
}

int KinovaApiInterface::ethernetSetTorqueZero(int ActuatorAdress) {
    return Ethernet_SetTorqueZero(ActuatorAdress);
}

int KinovaApiInterface::startForceControl() {
    return Ethernet_StartForceControl();
}

int KinovaApiInterface::stopForceControl() {
    return Ethernet_StartForceControl();
}

int KinovaApiInterface::setAngularInertiaDamping(AngularInfo inertia, AngularInfo damping) {
    return Ethernet_SetAngularInertiaDamping(inertia, damping);
}

int KinovaApiInterface::setAngularTorqueMinMax(AngularInfo min, AngularInfo max) {
    return Ethernet_SetAngularTorqueMinMax(min, max);
}

int KinovaApiInterface::setTaskInertiaDamping(CartesianInfo inertia, CartesianInfo damping) {
    return Ethernet_SetCartesianInertiaDamping(inertia, damping);
}

int KinovaApiInterface::setTaskForceMinMax(CartesianInfo min, CartesianInfo max) {
    return Ethernet_SetCartesianForceMinMax(min, max);
}

}  // namespace crf::actuators::kinovaarm
