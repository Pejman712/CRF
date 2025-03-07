/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <KinovaTypes.h>
#include <Kinova.API.EthCommandLayerUbuntu.h>

#include "KinovaJacoAPI/KinovaJacoAPIInterface.hpp"

namespace crf::communication::kinovajacoapi {

int KinovaJacoAPIInterface::initEthernetAPI(EthernetCommConfig& config) {
    return Ethernet_InitEthernetAPI(config);
}

int KinovaJacoAPIInterface::closeAPI() {
    return Ethernet_CloseAPI();
}

int KinovaJacoAPIInterface::startControlAPI() {
    return Ethernet_StartControlAPI();
}

int KinovaJacoAPIInterface::stopControlAPI() {
    return Ethernet_StopControlAPI();
}

int KinovaJacoAPIInterface::refresDevicesList() {
    return Ethernet_RefresDevicesList();
}

int KinovaJacoAPIInterface::getDevices(KinovaDevice devices[MAX_KINOVA_DEVICE], int& result) {
    return Ethernet_GetDevices(devices, result);
}

int KinovaJacoAPIInterface::setActiveDevice(KinovaDevice device) {
    return Ethernet_SetActiveDevice(device);
}

int KinovaJacoAPIInterface::moveHome() {
    return Ethernet_MoveHome();
}

int KinovaJacoAPIInterface::initFingers() {
    return Ethernet_InitFingers();
}

int KinovaJacoAPIInterface::eraseAllTrajectories() {
    return Ethernet_EraseAllTrajectories();
}

int KinovaJacoAPIInterface::activateSingularityAutomaticAvoidance(int state) {
    return Ethernet_ActivateSingularityAutomaticAvoidance(state);
}

int KinovaJacoAPIInterface::activateCollisionAutomaticAvoidance(int state) {
    return Ethernet_ActivateCollisionAutomaticAvoidance(state);
}

int KinovaJacoAPIInterface::getAngularPosition(AngularPosition& response) {
    return Ethernet_GetAngularPosition(response);
}

int KinovaJacoAPIInterface::getAngularVelocity(AngularPosition& response) {
    return Ethernet_GetAngularVelocity(response);
}

int KinovaJacoAPIInterface::getAngularForce(AngularPosition& response) {
    return Ethernet_GetAngularForce(response);
}

int KinovaJacoAPIInterface::getAngularForceGravityFree(AngularPosition& response) {
    return Ethernet_GetAngularForceGravityFree(response);
}

int KinovaJacoAPIInterface::getTaskPose(CartesianPosition& response) {
    return Ethernet_GetCartesianPosition(response);
}

int KinovaJacoAPIInterface::getTaskForce(CartesianPosition& response) {
    return Ethernet_GetCartesianForce(response);
}

int KinovaJacoAPIInterface::getSensorInfo(SensorsInfo& response) {
    return Ethernet_GetSensorsInfo(response);
}

int KinovaJacoAPIInterface::sendAdvanceTrajectory(TrajectoryPoint trajectory) {
    return Ethernet_SendAdvanceTrajectory(trajectory);
}

int KinovaJacoAPIInterface::setTaskControl() {
    return Ethernet_SetCartesianControl();
}

int KinovaJacoAPIInterface::setAngularControl() {
    return Ethernet_SetAngularControl();
}

int KinovaJacoAPIInterface::getControlType(int& response) {
    return Ethernet_GetControlType(response);
}

int KinovaJacoAPIInterface::setTorqueZero(int ActuatorAdress) {
    return Ethernet_SetTorqueZero(ActuatorAdress);
}

int KinovaJacoAPIInterface::startForceControl() {
    return Ethernet_StartForceControl();
}

int KinovaJacoAPIInterface::stopForceControl() {
    return Ethernet_StartForceControl();
}

int KinovaJacoAPIInterface::setAngularInertiaDamping(AngularInfo inertia, AngularInfo damping) {
    return Ethernet_SetAngularInertiaDamping(inertia, damping);
}

int KinovaJacoAPIInterface::setAngularTorqueMinMax(AngularInfo min, AngularInfo max) {
    return Ethernet_SetAngularTorqueMinMax(min, max);
}

int KinovaJacoAPIInterface::setTaskInertiaDamping(CartesianInfo inertia, CartesianInfo damping) {
    return Ethernet_SetCartesianInertiaDamping(inertia, damping);
}

int KinovaJacoAPIInterface::setTaskForceMinMax(CartesianInfo min, CartesianInfo max) {
    return Ethernet_SetCartesianForceMinMax(min, max);
}

}  // namespace crf::communication::kinovajacoapi
