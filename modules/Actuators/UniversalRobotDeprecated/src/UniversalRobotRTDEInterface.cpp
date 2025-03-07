/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include "UniversalRobot/UniversalRobotRTDEInterface.hpp"

namespace crf::actuators::universalrobot {

// Interfaces for rtdeReceive

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

// Interfaces for rtdeControl

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

bool UniversalRobotRTDEInterface::speedStop() {
    return rtdeControl_->speedStop();
}

bool UniversalRobotRTDEInterface::servoStop() {
    return rtdeControl_->servoStop();
}

void UniversalRobotRTDEInterface::stopScript() {
    rtdeControl_->stopScript();
}

}  // namespace crf::actuators::universalrobot
