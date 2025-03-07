/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <string>
#include <exception>

#include <nlohmann/json.hpp>

#include "Robot/RobotConfiguration.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"

namespace crf::actuators::robot {

RobotConfiguration::RobotConfiguration(const nlohmann::json& robotConfig):
    robotJSON_(robotConfig),
    lenghtsParsed_(false),
    forwardKinematicsParsed_(false),
    jacobianParsed_(false),
    logger_("RobotConfiguration") {
        logger_->debug("CTor");
        parse(robotJSON_);
}

uint64_t RobotConfiguration::getJointSpaceDoF() const {
    return jointSpaceDoF_;
}

uint64_t RobotConfiguration::getTaskSpaceDoF() const {
    return taskSpaceDoF_;
}

std::chrono::milliseconds RobotConfiguration::getRobotControllerLoopTime() const {
    return robotControllerLoopTime_;
}

JointLimits RobotConfiguration::getJointLimits() const {
    return jointLimits_;
}

TaskLimits RobotConfiguration::getTaskLimits() const {
    return taskLimits_;
}

ProfileParameters RobotConfiguration::getProfileParameters() const {
    return profileParameters_;
}

RobotLenghts RobotConfiguration::getRobotLengths() const {
    if (!lenghtsParsed_) throw std::runtime_error(
        "The configuration file provided has no Lengths!");
    return robotLengths_;
}

std::shared_ptr<IForwardKinematics> RobotConfiguration::getForwardKinematics() const {
    if (!forwardKinematicsParsed_) throw std::runtime_error(
        "The configuration file provided has no Math Expressions!");
    return forwardKinematics_;
}

std::shared_ptr<IJacobian> RobotConfiguration::getJacobian() const {
    if (!jacobianParsed_) throw std::runtime_error(
        "The configuration file provided has no Jacobian!");
    return jacobian_;
}

nlohmann::json RobotConfiguration::getConfigurationFile() const {
    return robotJSON_;
}

TaskSpace RobotConfiguration::getTaskSpace() const {
    return taskSpace_;
}

// Private

void RobotConfiguration::parse(const nlohmann::json& robotConfig) {
    logger_->debug("parse");
    try {
        // Mandatory parameters
        jointSpaceDoF_ = robotConfig["JointSpaceDegreeOfFreedom"].get<unsigned int>();
        robotControllerLoopTime_ = std::chrono::milliseconds(
            robotConfig["ControllerLoopTimeMs"].get<unsigned int>());
        parseJointLimits(robotConfig["JointLimits"]);
        parseTaskLimits(robotConfig["TaskLimits"]);
        parseProfileParameters(robotConfig["ProfileParameters"]);

        // Optional parameters
        if (!robotConfig.contains("TaskSpace")) {
            logger_->info("No TaskSpace configuration found defaulting to full task space.");
            taskSpace_ = TaskSpace();
        } else {
            logger_->info("TaskSpace configuration found");
            TaskSpace taskSpace = robotConfig["TaskSpace"].get<TaskSpace>();
            taskSpace_ = TaskSpace(taskSpace);
        }
        taskSpaceDoF_ = taskSpace_.dimension();
        if (!robotConfig.contains("Kinematics")) {
            logger_->info("No kinematics configuration found");
        } else {
            logger_->info("Kinematics configuration found");

            std::string type = robotConfig["Kinematics"]["Type"].get<std::string>();

            if (type == "MathExpressions") {
                parseExpressionLengths(robotConfig["Kinematics"]["MathExpressions"]["Lengths"]);
                parseExpressionFK(
                    robotConfig["Kinematics"]["MathExpressions"]["ForwardKinematics"]);
                parseExpressionJacobian(robotConfig["Kinematics"]["MathExpressions"]["Jacobian"]);
            } else if (type == "URDF") {
                parseKinChainJacobian(robotConfig["Kinematics"]["URDF"]);
                parseKinChainFK(robotConfig["Kinematics"]["URDF"]);
            } else if (type == "Internal") {
                forwardKinematics_ = nullptr;
                forwardKinematicsParsed_ = true;
            } else {
                throw std::invalid_argument(
                    "Invalid kinematics type in the Robot Configuration file");
            }
        }
    } catch (const std::exception& e) {
        logger_->error("Failed to parse because: {}", e.what());
        throw std::invalid_argument(std::string(
            "The configuration file provided could not be parsed in RobotConfiguration: ")
            + e.what());
    }
}

void RobotConfiguration::parseJointLimits(const nlohmann::json& limits) {
    logger_->debug("parseJointLimits");
    jointLimits_.maxPosition = limits["MaxPosition"].get<JointPositions>();
    jointLimits_.minPosition = limits["MinPosition"].get<JointPositions>();
    jointLimits_.maxVelocity = limits["MaxVelocity"].get<JointVelocities>();
    jointLimits_.maxAcceleration = limits["MaxAcceleration"].get<JointAccelerations>();
    jointLimits_.maxTorque = limits["MaxTorque"].get<JointForceTorques>();
    if (jointLimits_.checkDimensions() != jointSpaceDoF_) {
        throw std::invalid_argument("The size of all joint limits, is not equal "
            "to JointSpaceDegreeOfFreedom = " + std::to_string(jointSpaceDoF_));
    }
}

void RobotConfiguration::parseTaskLimits(const nlohmann::json& limits) {
    logger_->debug("parseTaskLimits");
    taskLimits_.maxVelocity = limits["MaxVelocity"].get<TaskVelocity>();
    taskLimits_.maxAcceleration = limits["MaxAcceleration"].get<TaskAcceleration>();
}

void RobotConfiguration::parseProfileParameters(const nlohmann::json& params) {
    logger_->debug("parseProfileParameters");
    profileParameters_.jointVelocities = params["JointVelocities"].get<JointVelocities>();
    profileParameters_.jointAccelerations = params["JointAccelerations"].get<JointAccelerations>();
    profileParameters_.taskVelocity = params["TaskVelocity"].get<TaskVelocity>();
    profileParameters_.taskAcceleration = params["TaskAcceleration"].get<TaskAcceleration>();
    if (profileParameters_.checkDimensions() != jointSpaceDoF_) {
        throw std::invalid_argument(
            "The size of all joint profile values, is not equal to the joint or task dimensions");
    }
}

void RobotConfiguration::parseExpressionLengths(const nlohmann::json& lengths) {
    logger_->debug("parseExpressionLengths");
    robotLengths_.lx = lengths["lx"].get<std::vector<double>>();
    robotLengths_.ly = lengths["ly"].get<std::vector<double>>();
    robotLengths_.lz = lengths["lz"].get<std::vector<double>>();
    if (robotLengths_.checkDimensions() != jointSpaceDoF_) {
        throw std::invalid_argument(
            "The number of lengths, does not match the number of joints");
    }
    lenghtsParsed_ = true;
}

void RobotConfiguration::parseExpressionFK(const nlohmann::json& expressions) {
    logger_->debug("parseExpressionFK");
    forwardKinematics_ =
        std::make_shared<MathExprForwardKinematics>(
            expressions, robotLengths_.lx, robotLengths_.ly, robotLengths_.lz);
    forwardKinematicsParsed_ = true;
}

void RobotConfiguration::parseExpressionJacobian(const nlohmann::json& jacobian) {
    logger_->debug("parseExpressionJacobian");
    jacobian_ = std::make_shared<MathExprJacobian>(
        jacobian, robotLengths_.lx, robotLengths_.ly, robotLengths_.lz);
    jacobianParsed_ = true;
}

void RobotConfiguration::parseKinChainJacobian(const nlohmann::json& URDF) {
    logger_->debug("parseKinChainJacobian");

    std::string path = __FILE__;
    std::string robotPath;
    std::string toolPathEnd;
    std::string toolPath;
    path = path.substr(0, path.find("cpproboticframework"));
    robotPath = path + URDF["RobotURDFPath"].get<std::string>();
    if (URDF.contains("ToolURDFPath")) {
        toolPathEnd = URDF["ToolURDFPath"].get<std::string>();
    } else {
        toolPathEnd = "";
    }
    if (toolPathEnd != "") {
        toolPath = path + URDF["ToolURDFPath"].get<std::string>();
    } else {
        toolPath = toolPathEnd;
    }

    std::shared_ptr<URDFKinematicChain> kinematicChain = std::make_shared<URDFKinematicChain>(
        robotPath, URDF["EndEffectorName"].get<std::string>(), toolPath);
    jacobian_ = std::make_shared<KinChainJacobian>(kinematicChain, taskSpace_);
    jacobianParsed_ = true;
}

void RobotConfiguration::parseKinChainFK(const nlohmann::json& URDF) {
    logger_->debug("parseKinChainFK");

    std::string path = __FILE__;
    std::string robotPath;
    std::string toolPathEnd;
    std::string toolPath;
    path = path.substr(0, path.find("cpproboticframework"));
    robotPath = path + URDF["RobotURDFPath"].get<std::string>();
    if (URDF.contains("ToolURDFPath")) {
        toolPathEnd = URDF["ToolURDFPath"].get<std::string>();
    } else {
        toolPathEnd = "";
    }
    if (toolPathEnd != "") {
        toolPath = path + URDF["ToolURDFPath"].get<std::string>();
    } else {
        toolPath = toolPathEnd;
    }

    std::shared_ptr<URDFKinematicChain> kinematicChain = std::make_shared<URDFKinematicChain>(
        robotPath, URDF["EndEffectorName"].get<std::string>(), toolPath);
    forwardKinematics_ = std::make_shared<KinChainForwardKinematics>(kinematicChain);
    forwardKinematicsParsed_ = true;
}

}  // namespace crf::actuators::robot
