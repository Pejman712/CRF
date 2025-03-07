/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *         Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <chrono>
#include <vector>
#include <string>
#include <exception>

#include <nlohmann/json.hpp>

#include "RobotArm/RobotArmConfiguration.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"

namespace crf::actuators::robotarm {

RobotArmConfiguration::RobotArmConfiguration():
    logger_("RobotArmConfiguration"),
    rtLoopTime_(),
    kinematicChain_(),
    joints_(),
    jointsDirection_(),
    jointsOffset_(),
    limits_(),
    pids_(),
    robotLengths_(),
    fkJson_(),
    jacobianJson_() {
}

bool RobotArmConfiguration::parse(const nlohmann::json& robotJSON) {
    cleanup();
    try {
        unsigned int jointsCount = robotJSON["NumberOfJoints"].get<unsigned int>();
        rtLoopTime_ = std::chrono::milliseconds(robotJSON["LoopTimeMs"].get<unsigned int>());

        // Get the joint related parameters
        nlohmann::json dhParametersJson = robotJSON["KinematicChain"]["parameters"];
        nlohmann::json jointsParametersJson = robotJSON["Joints"];
        if (dhParametersJson.size() != jointsCount) {
            throw std::runtime_error(
                "The number of DH parameters, does not match the number of joints");
        }
        if (jointsParametersJson.size() != jointsCount) {
            throw std::runtime_error(
                "The number of joint limits, does not match the number of joints");
        }
        for (unsigned int i=0; i < jointsCount; i++) {
            // Denavit Hartenberg Parameters
            DHParameter dhLink;
            dhLink.d = dhParametersJson[i]["D"].get<double>();
            dhLink.theta = dhParametersJson[i]["Theta"].get<double>();
            dhLink.a = dhParametersJson[i]["A"].get<double>();
            dhLink.alpha = dhParametersJson[i]["Alpha"].get<double>();

            // Optional Parameter in the DH configuration
            if (dhParametersJson[i].find("Type") != dhParametersJson[i].end()) {
                auto typeStr = dhParametersJson[i]["Type"].get<std::string>();
                if (typeStr == "Rotational") {
                    dhLink.type = DHParameter::JointType::Rotational;
                } else if (typeStr == "Linear") {
                    dhLink.type = DHParameter::JointType::Linear;
                } else {
                    logger_->warn("JointType: {} is not supported", typeStr);
                    throw std::runtime_error("Incorrect JointType");
                }
            } else {
                dhLink.type = DHParameter::JointType::Rotational;
            }
            kinematicChain_.push_back(dhLink);

            // Joint Limits
            JointLimits joint;
            joint.minimumPosition = jointsParametersJson[i]["Minimum"].get<double>();
            joint.maximumPosition = jointsParametersJson[i]["Maximum"].get<double>();
            joint.maximumVelocity = fabs(jointsParametersJson[i]["MaximumVelocity"].get<double>());
            joint.maximumAcceleration = fabs(
                jointsParametersJson[i]["MaximumAcceleration"].get<double>());
            // Optional parameter
            if (jointsParametersJson[i].find("MaximumTorque") != jointsParametersJson[i].end()) {
                joint.maximumTorque =
                    fabs(jointsParametersJson.at(i).at("MaximumTorque").get<double>());
            }
            joints_.push_back(joint);
            // Direction - Optional Parameter
            int direction = 1;
            if (jointsParametersJson[i].find("Direction") != jointsParametersJson[i].end()) {
                direction = jointsParametersJson[i]["Direction"].get<int>();
                if ((direction != -1) && (direction != 1)) {
                    logger_->warn("Failed to parse because: joint direction must be 1 or -1");
                    throw std::runtime_error("Incorrect joint direction");
                }
            }
            jointsDirection_.push_back(direction);

            // Offset - Optional Parameter
            double offset = 0.0f;
            if (jointsParametersJson[i].find("Offset") != jointsParametersJson[i].end()) {
                offset = jointsParametersJson[i]["Offset"].get<double>();
            }
            jointsOffset_.push_back(offset);

            // PID Parameters
            pids_.KpJoint.push_back(jointsParametersJson[i]["Kp"].get<double>());
            pids_.KiJoint.push_back(jointsParametersJson[i]["Ki"].get<double>());
            pids_.KdJoint.push_back(jointsParametersJson[i]["Kd"].get<double>());
        }

        // Get robot lengths - Optional Parameter
        if (robotJSON.contains("Lengths")) {
            if (robotJSON["Lengths"].size() != 3 ||
                !robotJSON["Lengths"].contains("lx") ||
                !robotJSON["Lengths"].contains("ly") ||
                !robotJSON["Lengths"].contains("lz")) {
                throw std::runtime_error("In each joint, a length in x, y and z is needed. "
                    "Check the names.");
            }
            if (robotJSON["Lengths"]["lx"].size() != jointsCount ||
                robotJSON["Lengths"]["ly"].size() != jointsCount ||
                robotJSON["Lengths"]["lz"].size() != jointsCount) {
                throw std::runtime_error(
                    "The number of lengths, does not match the number of joints");
            }
            for (unsigned int i = 0; i < 3; i++) {
                robotLengths_[i].resize(jointsCount);
            }
            for (unsigned int j = 0; j < jointsCount; j++) {
                robotLengths_[0][j] = robotJSON["Lengths"]["lx"][j];
                robotLengths_[1][j] = robotJSON["Lengths"]["ly"][j];
                robotLengths_[2][j] = robotJSON["Lengths"]["lz"][j];
            }
        }

        // Get forward kinematics mathematical expressions - Optional Parameter
        if (robotJSON["KinematicChain"].contains("format")) {
            if (robotJSON["KinematicChain"]["format"] == "ReadingMathExpressions") {
                if (!robotJSON["KinematicChain"].contains("parameters2") ||
                    !robotJSON.contains("Lengths")) {
                    throw std::runtime_error("The format ReadingMathExpressions needs "
                        "parameters and lengths");
                }
                fkJson_ = robotJSON["KinematicChain"]["parameters2"];
            }
        }

        // Get jacobian matrix expressions - Optional Parameter
        if (robotJSON.contains("Jacobian")) {
            if (!robotJSON.contains("Lengths")) {
                throw std::runtime_error("It is also necessary to read the robot lenghts");
            }
            jacobianJson_ = robotJSON["Jacobian"];
        }

        // Get the Task related parameters
        nlohmann::json taskMaxVelocity = robotJSON.at("Task").at("MaximumVelocity");
        nlohmann::json taskMaxAcceleration = robotJSON.at("Task").at("MaximumAcceleration");
        nlohmann::json TaskPosePIDKp = robotJSON.at("Task").at("Kp");
        nlohmann::json TaskPosePIDKi = robotJSON.at("Task").at("Ki");
        nlohmann::json TaskPosePIDKd = robotJSON.at("Task").at("Kd");

        unsigned int size = taskMaxVelocity.size();
        if (taskMaxAcceleration.size() != size || TaskPosePIDKp.size() != size ||
            TaskPosePIDKi.size() != size || TaskPosePIDKd.size() != size) {
            throw std::runtime_error("The number of task velocity or acceleration limits or the "
                "number of the parameters in the task PID is wrong");
        }
        for (unsigned int i=0; i < size; i++) {
            limits_.maximumVelocity[i] = taskMaxVelocity[i].get<double>();
            limits_.maximumAcceleration[i] = taskMaxAcceleration[i].get<double>();
            pids_.KpTask.push_back(TaskPosePIDKp[i].get<double>());
            pids_.KiTask.push_back(TaskPosePIDKi[i].get<double>());
            pids_.KdTask.push_back(TaskPosePIDKd[i].get<double>());
        }
    } catch (const std::exception& e) {
        logger_->error("Failed to parse because: {}", e.what());
        cleanup();
        return false;
    }
    return true;
}

void RobotArmConfiguration::cleanup() {
    rtLoopTime_ = std::chrono::milliseconds(0);
    kinematicChain_.clear();
    joints_.clear();
    jointsDirection_.clear();
    jointsOffset_.clear();
    pids_.KpJoint.clear();
    pids_.KiJoint.clear();
    pids_.KdJoint.clear();
    limits_.maximumVelocity = crf::utility::types::TaskVelocity();
    limits_.maximumAcceleration = crf::utility::types::TaskAcceleration();
    pids_.KpTask.clear();
    pids_.KiTask.clear();
    pids_.KdTask.clear();
    robotLengths_.fill(std::vector<double>());
    jacobianJson_.clear();
    fkJson_.clear();
}

std::vector<JointLimits> RobotArmConfiguration::getJointsConfiguration() {
    return joints_;
}

std::vector<int> RobotArmConfiguration::getJointsDirection() {
    return jointsDirection_;
}

std::vector<double> RobotArmConfiguration::getJointsOffset() {
    return jointsOffset_;
}

std::vector<DHParameter> RobotArmConfiguration::getKinematicChain() {
    return kinematicChain_;
}

unsigned int RobotArmConfiguration::getNumberOfJoints() {
    return joints_.size();
}

TaskLimits RobotArmConfiguration::getTaskLimits() {
    return limits_;
}

std::chrono::milliseconds RobotArmConfiguration::getRTLoopTime() {
    return rtLoopTime_;
}

parametersPID RobotArmConfiguration::getParametersPIDs() {
    return pids_;
}

std::array<std::vector<double>, 3> RobotArmConfiguration::getRobotLengths() {
    return robotLengths_;
}

nlohmann::json RobotArmConfiguration::getFKMathExpressions() {
    return fkJson_;
}

nlohmann::json RobotArmConfiguration::getJacobianMathExpressions() {
    return jacobianJson_;
}

}  // namespace crf::actuators::robotarm
