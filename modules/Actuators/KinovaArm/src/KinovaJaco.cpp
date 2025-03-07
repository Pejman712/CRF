/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#define ACTUATOR1_ADDRESS 16

#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>

#include <exception>
#include <vector>
#include <cmath>
#include <memory>
#include <string>
#include <fstream>
#include <arpa/inet.h>
#include <boost/optional.hpp>
#include <nlohmann/json.hpp>

#include "KinovaArm/KinovaJaco.hpp"
#include "KinovaArm/KinovaApiInterface.hpp"

namespace crf::actuators::kinovaarm {

KinovaJaco::KinovaJaco(std::shared_ptr<IKinovaApiInterface> apiInterface,
    const nlohmann::json& robotConfigFile):
    apiInterface_(apiInterface),
    robotConfigFile_(robotConfigFile),
    logger_("KinovaJaco"),
    configuration_(new KinovaArmConfiguration),
    isInitialized_(false),
    jointsDirection_(),
    jointPositionsOffset_(),
    movedHome_(false) {
    logger_->debug("CTor");
    if (!apiInterface_) {
        logger_->info("Creating a default Kinova API interface");
        apiInterface_.reset(new KinovaApiInterface);
    }
}

KinovaJaco::~KinovaJaco() {
    logger_->debug("DTor");
}

bool KinovaJaco::initialize() {
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

    EthernetCommConfig ethConfig;
    ethConfig.localIpAddress =  inet_addr(
        configuration_->getNetworkConfiguration().localAddressIP.c_str());
    ethConfig.robotIpAddress = inet_addr(
        configuration_->getNetworkConfiguration().robotAddressIP.c_str());
    ethConfig.subnetMask = inet_addr(configuration_->getNetworkConfiguration().subnetMask.c_str());
    ethConfig.localCmdport = 25015;
    ethConfig.localBcastPort = 25025;
    ethConfig.robotPort = configuration_->getNetworkConfiguration().port;
    ethConfig.rxTimeOutInMs = 1000;
    int result = apiInterface_->ethernetInitEthernetAPI(ethConfig);
    if (result != 1) {
        logger_->error("Failed to initialize the API - Error Code {}", result);
        return false;
    }
    logger_->info("API initialized correctly");
    result = apiInterface_->ethernetRefresDevicesList();
    if (result != 1) {
        logger_->error("Failed to refresh the devices list - Error Code {}", result);
        return false;
    }
    logger_->info("The devices list has been refreshed");
    KinovaDevice devicesList[MAX_KINOVA_DEVICE] = {};
    int robotsNum = apiInterface_->ethernetGetDevices(devicesList, result);
    if (robotsNum < 1) {
        logger_->error("Failed to get the devices");
        return false;
    }
    logger_->info("{} arms found", robotsNum);
    auto selectedDevice = std::find_if(std::begin(devicesList), std::end(devicesList),
        [sn = configuration_->getSerialNumber()](const KinovaDevice& dev) {
            return std::string(dev.SerialNumber) == sn;
        });
    if (selectedDevice == std::end(devicesList)) {
        logger_->error("Desired robot arm {} not found", configuration_->getSerialNumber());
        return false;
    }
    if (apiInterface_->ethernetSetActiveDevice(*selectedDevice) != 1) {
        logger_->error("Failed to set the device {} - {}",
            selectedDevice->Model, configuration_->getSerialNumber());
        return false;
    }
    logger_->info("The device {} - {} was set", selectedDevice->Model,
        configuration_->getSerialNumber());
    if (apiInterface_->ethernetStartControlAPI() != 1) {
        logger_->error("Failed to start API control");
        return false;
    }

    jointsDirection_ = configuration_->getJointsDirection();
    jointPositionsOffset_ = configuration_->getJointsOffset();

    logger_->info("Calculating angular offset to only allow one turn on each joint, staying in ");
    AngularPosition position;
    if (apiInterface_->ethernetGetAngularPosition(position) != 1) {
        logger_->warn("Failed to get the joints position - Returning empty vector");
        return false;
    }
    utility::types::JointPositions currentJointPositions(configuration_->getNumberOfJoints());
    currentJointPositions = {deg2rad(position.Actuators.Actuator1),
        deg2rad(position.Actuators.Actuator2),
        deg2rad(position.Actuators.Actuator3),
        deg2rad(position.Actuators.Actuator4),
        deg2rad(position.Actuators.Actuator5),
        deg2rad(position.Actuators.Actuator6)};
    for (unsigned int i=0; i < currentJointPositions.size(); i++) {
        currentJointPositions[i] *= jointsDirection_[i];
        while (currentJointPositions[i] + jointPositionsOffset_[i] > M_PI) {
            jointPositionsOffset_[i] -= 2*M_PI;
        }
        while (currentJointPositions[i] + jointPositionsOffset_[i] < -M_PI) {
            jointPositionsOffset_[i] += 2*M_PI;
        }
    }
    isInitialized_ = true;
    return true;
}

bool KinovaJaco::deinitialize() {
    logger_->debug("deinitialize");
    if (!isInitialized_) {
        logger_->error("The API hasn't been initialized");
        return false;
    }
    if (stopArm() != 1) {
        logger_->error("Failed to stop the robot arm");
        return false;
    }
    logger_->info("The arm has been stopped");
    if (apiInterface_->ethernetStopControlAPI() != 1) {
        logger_->error("Failed to stop API control");
        return false;
    }
    logger_->info("API control stoped");
    if (apiInterface_->ethernetCloseAPI() != 1) {
        logger_->error("Failed to deinitialize - API can't be closed");
        return false;
    }
    logger_->info("The API has been closed");
    isInitialized_ = false;
    return true;
}

boost::optional<utility::types::JointPositions> KinovaJaco::getJointPositions() {
    logger_->debug("getJointPositions");
    if (!isInitialized_) {
        logger_->error("The API hasn't been initialized - Returning empty vector");
        return boost::none;
    }
    AngularPosition position;
    if (apiInterface_->ethernetGetAngularPosition(position) != 1) {
        logger_->warn("Failed to get the joints position - Returning empty vector");
        return boost::none;
    }
    utility::types::JointPositions result(configuration_->getNumberOfJoints());
    result = {deg2rad(position.Actuators.Actuator1),
        deg2rad(position.Actuators.Actuator2),
        deg2rad(position.Actuators.Actuator3),
        deg2rad(position.Actuators.Actuator4),
        deg2rad(position.Actuators.Actuator5),
        deg2rad(position.Actuators.Actuator6)};
    for (unsigned int i = 0; i < 6; i++) {
        result[i] *= jointsDirection_[i];
        result[i] += jointPositionsOffset_[i];
    }
    return result;
}

boost::optional<utility::types::JointVelocities> KinovaJaco::getJointVelocities() {
    if (!isInitialized_) {
        logger_->error("The API hasn't been initialized - Returning empty vector");
        return boost::none;
    }
    AngularPosition velocity;
    if (apiInterface_->ethernetGetAngularVelocity(velocity) != 1) {
        logger_->warn("Failed to get the joints velocity - Returning empty vector");
        return boost::none;
    }
    utility::types::JointVelocities result(configuration_->getNumberOfJoints());
    result = {deg2rad(velocity.Actuators.Actuator1),
        deg2rad(velocity.Actuators.Actuator2),
        deg2rad(velocity.Actuators.Actuator3),
        deg2rad(velocity.Actuators.Actuator4),
        deg2rad(velocity.Actuators.Actuator5),
        deg2rad(velocity.Actuators.Actuator6)};
    for (unsigned int i=0; i < result.size(); i++) {
        result[i] *= jointsDirection_[i];
        result[i] *= velocityAPICorrectionFactor_;
    }
    return result;
}

boost::optional<crf::utility::types::JointForceTorques> KinovaJaco::getJointForceTorques() {
    if (!isInitialized_) {
        logger_->error("The API hasn't been initialized - Returning empty vector");
        return boost::none;
    }
    AngularPosition torque;
    if (apiInterface_->ethernetGetAngularForceGravityFree(torque) != 1) {
        logger_->warn("Failed to get the joints torque - Returning empty vector");
        return boost::none;
    }
    crf::utility::types::JointForceTorques result(configuration_->getNumberOfJoints());
    result = {static_cast<double>(torque.Actuators.Actuator1),
        static_cast<double>(torque.Actuators.Actuator2),
        static_cast<double>(torque.Actuators.Actuator3),
        static_cast<double>(torque.Actuators.Actuator4),
        static_cast<double>(torque.Actuators.Actuator5),
        static_cast<double>(torque.Actuators.Actuator6)};
    for (unsigned int i=0; i < result.size(); i++) {
        result[i] *= jointsDirection_[i];
    }
    return result;
}

boost::optional<utility::types::TaskPose> KinovaJaco::getTaskPose() {
    if (!isInitialized_) {
        logger_->error("The API hasn't been initialized - Returning empty matrix");
        return boost::none;
    }
    CartesianPosition pose;
    if (apiInterface_->ethernetGetTaskPose(pose) != 1) {
        logger_->warn("Failed to get the end efector position - Returning empty matrix");
        return boost::none;
    }
    Eigen::Vector3d position(
        {pose.Coordinates.X,
         pose.Coordinates.Y,
         pose.Coordinates.Z});
    crf::math::rotation::CardanXYZ cardanXYZ(
        {static_cast<double>(pose.Coordinates.ThetaX),
         static_cast<double>(pose.Coordinates.ThetaY),
         static_cast<double>(pose.Coordinates.ThetaZ)});
    return crf::utility::types::TaskPose(position, cardanXYZ);
}

boost::optional<utility::types::TaskVelocity> KinovaJaco::getTaskVelocity() {
    logger_->warn("getTaskVelocity not supported");
    return boost::none;
}

boost::optional<crf::utility::types::TaskForceTorque> KinovaJaco::getTaskForceTorque() {
    if (!isInitialized_) {
        logger_->error("The API hasn't been initialized - Returning empty vector");
        return boost::none;
    }
    CartesianPosition torque;
    if (apiInterface_->ethernetGetTaskForce(torque) != 1) {
        logger_->warn("Failed to get the task torque - Returning empty vector");
        return boost::none;
    }
    return crf::utility::types::TaskForceTorque({static_cast<double>(torque.Coordinates.X),
        static_cast<double>(torque.Coordinates.Y),
        static_cast<double>(torque.Coordinates.Z),
        static_cast<double>(torque.Coordinates.ThetaX),
        static_cast<double>(torque.Coordinates.ThetaY),
        static_cast<double>(torque.Coordinates.ThetaZ)});
}

bool KinovaJaco::setJointPositions(const utility::types::JointPositions& jointPositions) {
    if (!isInitialized_) {
        logger_->error("The API hasn't been initialized");
        return false;
    }
    if (!movedHome_) {
        logger_->warn("you need to invoke moveHomePosition() before setJointPositions");
        return false;
    }

    // Check if the values of position are within the robot limits
    for (unsigned int jointID = 0; jointID < configuration_->getNumberOfJoints(); jointID++) {
        double minJointPos = configuration_->getJointsConfiguration()[jointID].minimumPosition;
        double maxJointPos = configuration_->getJointsConfiguration()[jointID].maximumPosition;
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

    utility::types::JointPositions convertedJointPositions(configuration_->getNumberOfJoints());
    for (unsigned int i = 0; i < configuration_->getNumberOfJoints(); i++) {
        convertedJointPositions[i] = jointPositions[i];
        convertedJointPositions[i] -= jointPositionsOffset_[i];
        convertedJointPositions[i] /= jointsDirection_[i];
    }
    int controlType = 0;
    if (apiInterface_->ethernetGetControlType(controlType) != 1) {
        logger_->error("Failed to get the control type");
        return false;
    }
    if (controlType != 1) {
        if (apiInterface_->ethernetSetAngularControl() != 1) {
            logger_->error("Failed to set the angular control");
            return false;
        }
    }
    TrajectoryPoint point = {};
    point.InitStruct();
    point.Position.Type = ANGULAR_POSITION;
    point.Position.HandMode = HAND_NOMOVEMENT;
    point.LimitationsActive = 1;
    point.Limitations.speedParameter1 = rad2deg(
        configuration_->getJointsConfiguration()[0].maximumVelocity);
    point.Limitations.speedParameter2 = rad2deg(
        configuration_->getJointsConfiguration()[0].maximumVelocity);
    point.Position.Actuators.Actuator1 = rad2deg(convertedJointPositions[0]);
    point.Position.Actuators.Actuator2 = rad2deg(convertedJointPositions[1]);
    point.Position.Actuators.Actuator3 = rad2deg(convertedJointPositions[2]);
    point.Position.Actuators.Actuator4 = rad2deg(convertedJointPositions[3]);
    point.Position.Actuators.Actuator5 = rad2deg(convertedJointPositions[4]);
    point.Position.Actuators.Actuator6 = rad2deg(convertedJointPositions[5]);
    if (apiInterface_->ethernetSendAdvanceTrajectory(point) != 1) {
        logger_->error("Failed to send the joints position");
        return false;
    }
    return true;
}

bool KinovaJaco::setJointVelocities(const utility::types::JointVelocities& jointVelocities) {
    if (!isInitialized_) {
        logger_->error("The API hasn't been initialized");
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

    utility::types::JointPositions convertedJointVelocities(configuration_->getNumberOfJoints());
    for (unsigned int i = 0; i < configuration_->getNumberOfJoints(); i++) {
        convertedJointVelocities[i] = jointVelocities[i];
        convertedJointVelocities[i] /= jointsDirection_[i];
    }
    int controlType = 0;
    if (apiInterface_->ethernetGetControlType(controlType) != 1) {
        logger_->error("Failed to get the control type");
        return false;
    }
    if (controlType != 1) {
        if (apiInterface_->ethernetSetAngularControl() != 1) {
            logger_->error("Failed to set the angular control");
            return false;
        }
    }
    TrajectoryPoint point = {};
    point.InitStruct();
    point.Position.Type = ANGULAR_VELOCITY;
    point.Position.HandMode = HAND_NOMOVEMENT;
    point.Position.Actuators.Actuator1 = rad2deg(convertedJointVelocities[0]);
    point.Position.Actuators.Actuator2 = rad2deg(convertedJointVelocities[1]);
    point.Position.Actuators.Actuator3 = rad2deg(convertedJointVelocities[2]);
    point.Position.Actuators.Actuator4 = rad2deg(convertedJointVelocities[3]);
    point.Position.Actuators.Actuator5 = rad2deg(convertedJointVelocities[4]);
    point.Position.Actuators.Actuator6 = rad2deg(convertedJointVelocities[5]);
    if (apiInterface_->ethernetSendAdvanceTrajectory(point) != 1) {
        logger_->error("Failed to send the joints velocity");
        return false;
    }
    return true;
}

bool KinovaJaco::setJointForceTorques(
    const crf::utility::types::JointForceTorques& jointForceTorques) {
    logger_->warn("setJointForceTorques not supported");
    return false;
}

bool KinovaJaco::setTaskPose(const utility::types::TaskPose& taskPos) {
    logger_->warn("setTaskPose not supported due to an API bug");
    return false;
}

bool KinovaJaco::setTaskVelocity(const utility::types::TaskVelocity& velocity, bool) {
    logger_->warn("setTaskVelocity not supported");
    return false;
}

bool KinovaJaco::stopArm() {
    logger_->debug("stopArm");
    if (!isInitialized_) {
        logger_->error("The API hasn't been initialized");
        return false;
    }
    if (apiInterface_->ethernetEraseAllTrajectories() != 1) {
        logger_->error("Failed to stop - Trajectories on the FIFO can't be deleted");
        return false;
    }
    logger_->info("The Trajectories on the FIFO has been deleted");
    if (!setJointVelocities(utility::types::JointVelocities(configuration_->getNumberOfJoints()))) {
        logger_->error("Failed to stop the robot arm");
        return false;
    }
    logger_->info("The robot arm has stopped");
    return true;
}

bool KinovaJaco::enableBrakes() {
    logger_->warn("enableBrakes not supported");
    return false;
}

bool KinovaJaco::disableBrakes() {
    logger_->warn("disableBrakes not supported");
    return false;
}

std::shared_ptr<robotarm::RobotArmConfiguration> KinovaJaco::getConfiguration() {
    return configuration_;
}

double KinovaJaco::deg2rad(float angle) {
    return static_cast<double>(angle*(M_PI/180));
}

float KinovaJaco::rad2deg(double angle) {
    return static_cast<float>(angle*(180/M_PI));
}

bool KinovaJaco::moveHomePosition() {
    if (apiInterface_->ethernetMoveHome() != 1) {
        logger_->error("Failed to move to home position!");
        return false;
    }
    movedHome_ = true;
    return true;
}

bool KinovaJaco::zeroJointForceTorques() {
    for (unsigned int i = 0; i < 6; ++i) {
        if (apiInterface_->ethernetSetTorqueZero(ACTUATOR1_ADDRESS + i) != 1) {
            logger_->error("Failed to zeroJointForceTorques() on motor : {}", i);
            return false;
        }
    }
    return true;
}

}  // namespace crf::actuators::kinovaarm
