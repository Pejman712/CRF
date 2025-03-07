/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include "UniversalRobotRTDE/UniversalRobotRTDEInterface.hpp"

namespace crf::communication::universalrobotrtde {

void UniversalRobotRTDEInterface::initRtdeReceiveInterface(std::string IP) {
    rtdeReceive_.reset(new ur_rtde::RTDEReceiveInterface(IP));
}

std::vector<double> UniversalRobotRTDEInterface::getActualQ() {
    return rtdeReceive_->getActualQ();
}

std::vector<double> UniversalRobotRTDEInterface::getActualQd() {
    return rtdeReceive_->getActualQd();
}

std::vector<double> UniversalRobotRTDEInterface::getActualTCPPose() {
    return rtdeReceive_->getActualTCPPose();
}

std::vector<double> UniversalRobotRTDEInterface::getActualTCPSpeed() {
    return rtdeReceive_->getActualTCPSpeed();
}

std::vector<double> UniversalRobotRTDEInterface::getActualTCPForce() {
    return rtdeReceive_->getFtRawWrench();
}

uint32_t UniversalRobotRTDEInterface::getRobotStatus() {
    return rtdeReceive_->getRobotStatus();
}

int32_t UniversalRobotRTDEInterface::getRobotMode() {
    return rtdeReceive_->getRobotMode();
}

uint32_t UniversalRobotRTDEInterface::getSafetyStatusBits() {
    return rtdeReceive_->getSafetyStatusBits();
}

void UniversalRobotRTDEInterface::initRtdeControlInterface(std::string IP) {
    rtdeControl_.reset(new ur_rtde::RTDEControlInterface(IP));
}

bool UniversalRobotRTDEInterface::servoJ(std::vector<double> qDes, double maxVel, double maxAcc,
    double loopTime, double lookAheadTime, double gain) {
    return rtdeControl_->servoJ(qDes, maxVel, maxAcc, loopTime, lookAheadTime, gain);
}

bool UniversalRobotRTDEInterface::speedJ(std::vector<double> qdDes, double maxAcceleration,
    double loopTime) {
    return rtdeControl_->speedJ(qdDes, maxAcceleration, loopTime);
}

bool UniversalRobotRTDEInterface::speedL(std::vector<double> zdDes, double maxAcceleration,
    double loopTime) {
    return rtdeControl_->speedL(zdDes, maxAcceleration, loopTime);
}

bool UniversalRobotRTDEInterface::moveJ(std::vector<double> qDes, double speed,
    double acceleration, bool async) {
    return rtdeControl_->moveJ(qDes, speed, acceleration, async);
}

bool UniversalRobotRTDEInterface::moveL(std::vector<double> zDes, double speed,
    double acceleration, bool async) {
    return rtdeControl_->moveL(zDes, speed, acceleration, async);
}

bool UniversalRobotRTDEInterface::setGravity(const std::vector<double>& gravity) {
    return rtdeControl_->setGravity(gravity);
}

bool UniversalRobotRTDEInterface::zeroFtSensor() {
    return rtdeControl_->zeroFtSensor();
}

bool UniversalRobotRTDEInterface::forceMode(std::vector<double> forceFrame,
    std::vector<int> complianceSelector, std::vector<double> desiredForceTorque, int type,
    std::vector<double> limits) {
    return rtdeControl_->forceMode(forceFrame, complianceSelector, desiredForceTorque, type,
        limits);
}

std::vector<double> UniversalRobotRTDEInterface::getJointForceTorques() {
    return rtdeControl_->getJointTorques();
}

bool UniversalRobotRTDEInterface::speedStop() {
    return rtdeControl_->speedStop();
}

bool UniversalRobotRTDEInterface::servoStop() {
    return rtdeControl_->servoStop();
}

void UniversalRobotRTDEInterface::stopScript() {
    rtdeControl_->stopScript();
}

}  // namespace crf::communication::universalrobotrtde
