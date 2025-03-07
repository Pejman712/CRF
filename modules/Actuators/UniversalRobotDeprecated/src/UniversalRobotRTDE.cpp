/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <exception>
#include <vector>
#include <cmath>
#include <memory>
#include <string>
#include <fstream>
#include <arpa/inet.h>
#include <boost/optional.hpp>
#include <nlohmann/json.hpp>

#include "UniversalRobot/UniversalRobotRTDE.hpp"
#include "UniversalRobot/UniversalRobotRTDEInterface.hpp"
#include "UniversalRobot/IUniversalRobotRTDEInterface.hpp"
#include "UniversalRobot/UniversalRobotConfiguration.hpp"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

namespace crf::actuators::universalrobot {

UniversalRobotRTDE::UniversalRobotRTDE(
    std::shared_ptr<IUniversalRobotRTDEInterface> urRtdeInterface,
    const nlohmann::json& robotConfigFile):
    urRtdeInterface_(urRtdeInterface),
    robotConfigFile_(robotConfigFile),
    logger_("UniversalRobot"),
    configuration_(new UniversalRobotConfiguration),
    isInitialized_(false) {
    logger_->debug("CTor");
}

UniversalRobotRTDE::~UniversalRobotRTDE() {
    logger_->debug("DTor");
}

bool UniversalRobotRTDE::initialize() {
    logger_->debug("initialize");

    if (isInitialized_) {
        logger_->error("The API is already initialized");
        return false;
    }
    if (!configuration_->parse(robotConfigFile_)) {
        logger_->error("Failed to read the configuration file");
        return false;
    }
    logger_->info("Configuration file loaded");

    urRtdeInterface_->initRtdeControlInterface(configuration_->getIPAddress());
    logger_->info("RTDE control connection established with {}", configuration_->getIPAddress());
    urRtdeInterface_->initRtdeReceiveInterface(configuration_->getIPAddress());
    logger_->info("RTDE receive connection established with {}", configuration_->getIPAddress());

    isInitialized_ = true;
    return true;
}

bool UniversalRobotRTDE::deinitialize() {
    logger_->debug("deinitialize");
    if (!isInitialized_) {
        logger_->error("The RTDE driver hasn't been initialized");
        return false;
    }
    this->stopArm();
    urRtdeInterface_->stopScript();
    logger_->info("The UR Script has been stopped");
    isInitialized_ = false;
    return true;
}

boost::optional<utility::types::JointPositions> UniversalRobotRTDE::getJointPositions() {
    logger_->debug("getJointPositions");
    if (!isInitialized_) {
        logger_->error("The RTDE connection hasn't been initialized - Returning empty vector");
        return boost::none;
    }
    std::vector<double> qActDouble = urRtdeInterface_->getActualQ();
    utility::types::JointPositions qAct({qActDouble[0], qActDouble[1], qActDouble[2],
        qActDouble[3], qActDouble[4], qActDouble[5]});
    return qAct;
}

boost::optional<utility::types::JointVelocities> UniversalRobotRTDE::getJointVelocities() {
    if (!isInitialized_) {
        logger_->error("The RTDE connection hasn't been initialized - Returning empty vector");
        return boost::none;
    }
    std::vector<double> qdActDouble = urRtdeInterface_->getActualQd();
    utility::types::JointVelocities qdAct({qdActDouble[0], qdActDouble[1], qdActDouble[2],
        qdActDouble[3], qdActDouble[4], qdActDouble[5]});
    return qdAct;
}

boost::optional<crf::utility::types::JointForceTorques> UniversalRobotRTDE::getJointForceTorques() {
    logger_->warn("getJointForceTorques not implemented");
    return boost::none;
}

boost::optional<utility::types::TaskPose> UniversalRobotRTDE::getTaskPose() {
    if (!isInitialized_) {
        logger_->error("The RTDE connection hasn't been initialized - Returning empty matrix");
        return boost::none;
    }
    std::vector<double> taskPosAct = urRtdeInterface_->getActualTCPPose();
    return crf::utility::types::TaskPose(
        {taskPosAct[0], taskPosAct[1], taskPosAct[2]},
        crf::math::rotation::CardanXYZ({taskPosAct[3], taskPosAct[4], taskPosAct[5]}));
}

boost::optional<utility::types::TaskVelocity> UniversalRobotRTDE::getTaskVelocity() {
    logger_->warn("getTaskVelocity not supported");
    return boost::none;
}

boost::optional<crf::utility::types::TaskForceTorque> UniversalRobotRTDE::getTaskForceTorque() {
    logger_->warn("getTaskForceTorque not implemented");
    return boost::none;
}

bool UniversalRobotRTDE::setJointPositions(const utility::types::JointPositions& jointPositions) {
    if (!isInitialized_) {
        logger_->error("The RTDE connection hasn't been initialized");
        return false;
    }

    const uint nDoF = configuration_->getNumberOfJoints();

    if ( jointPositions.size() != nDoF ) {
        logger_->error("Input position size not valid");
        return false;
    }
    // Safety Check: Joint angles within Limits?
    for (unsigned int jointID = 0; jointID < nDoF; jointID++) {
        float minJointPos = configuration_->getJointsConfiguration()[jointID].minimumPosition;
        float maxJointPos = configuration_->getJointsConfiguration()[jointID].maximumPosition;
        if (jointPositions[jointID] < minJointPos) {
            logger_->error(
                "The requested position ({}) for the joint {} is smaller than the minimum limit",
                jointPositions[jointID], jointID);
            return false;
        }
        if (jointPositions[jointID] > maxJointPos) {
            logger_->error(
                "The requested position ({}) for the joint {} is bigger than the maximum limit",
                jointPositions[jointID], jointID);
            return false;
        }
    }

    std::vector<double> qDes(nDoF, 0);
    for (unsigned int i = 0; i < nDoF; i++) {
      qDes[i] = jointPositions[i];
    }
    return urRtdeInterface_->servoJ(qDes, 0, 0,
        std::chrono::duration<double>(configuration_->getRTLoopTime()).count(),
        configuration_->getLookAheadTime(), configuration_->getGain());
}

bool UniversalRobotRTDE::setJointVelocities(
    const utility::types::JointVelocities& jointVelocities) {
  logger_->warn("setJointVelocities without acceleration not supported");
  return false;
}

bool UniversalRobotRTDE::setJointForceTorques(
    const crf::utility::types::JointForceTorques& jointForceTorques) {
    logger_->warn("setJointForceTorques not supported");
    return false;
}

bool UniversalRobotRTDE::setTaskPose(const utility::types::TaskPose& cartPos) {
    logger_->warn("setTaskPose not supported due to an API bug");
    return false;
}

bool UniversalRobotRTDE::setTaskVelocity(const utility::types::TaskVelocity& velocity, bool) {
    logger_->warn("setTaskVelocity not supported");
    return false;
}

bool UniversalRobotRTDE::stopArm() {
    logger_->debug("stopArm");
    if (!isInitialized_) {
        logger_->error("The RTDE connection hasn't been initialized");
        return false;
    }
    urRtdeInterface_->speedStop();
    logger_->info("The move command speedJ has been stopped");
    logger_->info("The robot arm has stopped");
    return true;
}

bool UniversalRobotRTDE::enableBrakes() {
    logger_->warn("enableBrakes not supported");
    return false;
}

bool UniversalRobotRTDE::disableBrakes() {
    logger_->warn("disableBrakes not supported");
    return false;
}

std::shared_ptr<robotarm::RobotArmConfiguration> UniversalRobotRTDE::getConfiguration() {
    return configuration_;
}

bool UniversalRobotRTDE::setJointVelocities(const utility::types::JointVelocities& jointVelocities,
    double maxAcceleration) {
    if (!isInitialized_) {
        logger_->error("The API hasn't been initialized");
        return false;
    }

    const uint nDoF{configuration_->getNumberOfJoints()};

    if (jointVelocities.size() != nDoF) {
        logger_->error("Input velocity size not valid");
        return false;
    }
    // Check if the values of velocity are within the robot limits
    for (unsigned int jointID = 0; jointID < configuration_->getNumberOfJoints(); jointID++) {
        if (std::fabs(jointVelocities[jointID]) >
            configuration_->getJointsConfiguration()[jointID].maximumVelocity) {
            logger_->error(
                "The requested velocity ({}) for the joint {} is bigger than the maximum limit",
                jointVelocities[jointID], jointID);
            return false;
        }
    }

    std::vector<double> qdDes(nDoF, 0);
    for (unsigned int i = 0; i < nDoF; i++) {
      qdDes[i] = jointVelocities[i];
    }
    logger_->debug("Converted JointVelocities into Vector");
    return urRtdeInterface_->speedJ(qdDes, maxAcceleration,
        std::chrono::duration<double>(configuration_->getRTLoopTime()).count());
}

}  // namespace crf::actuators::universalrobot
