/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
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
#include <nlohmann/json.hpp>

#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>

#include "Robot/KinovaJaco2/KinovaJaco2.hpp"

namespace crf::actuators::robot {

KinovaJaco2::KinovaJaco2(
    std::shared_ptr<IKinovaJacoAPIInterface> apiInterface,
    const KinovaJaco2Configuration& robotConfigFile):
    apiInterface_(apiInterface),
    configuration_(robotConfigFile),
    initialized_(false),
    errorCode_(crf::Code::OK),
    logger_("KinovaJaco2") {
    logger_->debug("CTor");
}

KinovaJaco2::~KinovaJaco2() {
    logger_->debug("DTor");
}

bool KinovaJaco2::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->error("The API is already initialized");
        errorCode_ = crf::Code::AlreadyInitialized;
        return false;
    }

    // Ethernet Configuration
    EthernetCommConfig ethConfig;
    ethConfig.localIpAddress =  inet_addr(
        configuration_.getNetworkConfiguration().localAddressIP.c_str());
    ethConfig.robotIpAddress = inet_addr(
        configuration_.getNetworkConfiguration().robotAddressIP.c_str());
    ethConfig.subnetMask = inet_addr(configuration_.getNetworkConfiguration().subnetMask.c_str());
    ethConfig.localCmdport = localCommandPort_;
    ethConfig.localBcastPort = localBroadcastPort_;
    ethConfig.robotPort = configuration_.getNetworkConfiguration().port;
    ethConfig.rxTimeOutInMs = RXTimeoutMs_;

    int result = apiInterface_->initEthernetAPI(ethConfig);
    if (result != 1) {
        errorCode_ = parseErrorCode(result);
        logger_->error("Failed to initialize the API - Error Code {}",
            crf::ResponseCode(errorCode_));
        return false;
    }
    result = apiInterface_->refresDevicesList();
    if (result != 1) {
        errorCode_ = parseErrorCode(result);
        logger_->error("Failed to refresh the devices list - Error Code {}",
            crf::ResponseCode(errorCode_));
        return false;
    }
    KinovaDevice devicesList[MAX_KINOVA_DEVICE] = {};
    int robotsNum = apiInterface_->getDevices(devicesList, result);
    if (robotsNum < 1) {
        errorCode_ = parseErrorCode(result);
        logger_->error("Failed to get the devices, {}", crf::ResponseCode(errorCode_));
        return false;
    }
    logger_->info("{} arms found", robotsNum);
    auto selectedDevice = std::find_if(std::begin(devicesList), std::end(devicesList),
        [sn = configuration_.getSerialNumber()](const KinovaDevice& dev) {
            return std::string(dev.SerialNumber) == sn;
        });
    if (selectedDevice == std::end(devicesList)) {
        logger_->error("Desired robot arm {} not found", configuration_.getSerialNumber());
        errorCode_ = crf::Code::KJ_ErrorNoDeviceFound;
        return false;
    }
    result = apiInterface_->setActiveDevice(*selectedDevice);
    if (result != 1) {
        logger_->error("Failed to set the device {} - {}",
            selectedDevice->Model, configuration_.getSerialNumber());
        errorCode_ = parseErrorCode(result);
        return false;
    }
    logger_->info("The device {} - {} was set", selectedDevice->Model,
        configuration_.getSerialNumber());
    result = apiInterface_->startControlAPI();
    if (result != 1) {
        logger_->error("Failed to start API control");
        errorCode_ = parseErrorCode(result);
        return false;
    }
    initialized_ = true;
    errorCode_ = crf::Code::OK;
    return true;
}

bool KinovaJaco2::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->error("The API hasn't been initialized");
        errorCode_ = crf::Code::NotInitialized;
        return false;
    }
    if (!softStop()) {
        logger_->error("Failed to stop the robot arm");
        return false;
    }
    int result = apiInterface_->stopControlAPI();
    if (result != 1) {
        logger_->error("Failed to stop API control");
        errorCode_ = parseErrorCode(result);
        return false;
    }
    result = apiInterface_->closeAPI();
    if (result != 1) {
        logger_->error("Failed to deinitialize - API can't be closed");
        errorCode_ = parseErrorCode(result);
        return false;
    }
    initialized_ = false;
    errorCode_ = crf::Code::OK;
    return true;
}

crf::expected<crf::utility::types::JointPositions> KinovaJaco2::getJointPositions() {
    logger_->debug("getJointPositions");
    if (!initialized_) {
        logger_->error("The API hasn't been initialized - Returning empty vector");
        return crf::Code::NotInitialized;
    }
    AngularPosition position;
    if (apiInterface_->getAngularPosition(position) != 1) {
        logger_->warn("Failed to get the joints position - Returning empty vector");
        return crf::Code::RequestToDeviceFailed;
    }
    utility::types::JointPositions result(configuration_.getJointSpaceDoF());
    result = {deg2rad(position.Actuators.Actuator1),
        deg2rad(position.Actuators.Actuator2),
        deg2rad(position.Actuators.Actuator3),
        deg2rad(position.Actuators.Actuator4),
        deg2rad(position.Actuators.Actuator5),
        deg2rad(position.Actuators.Actuator6)};
    return result;
}

crf::expected<crf::utility::types::JointVelocities> KinovaJaco2::getJointVelocities() {
    if (!initialized_) {
        logger_->error("The API hasn't been initialized - Returning empty vector");
        return crf::Code::NotInitialized;
    }
    AngularPosition velocity;
    if (apiInterface_->getAngularVelocity(velocity) != 1) {
        logger_->warn("Failed to get the joints velocity - Returning empty vector");
        return crf::Code::RequestToDeviceFailed;
    }
    utility::types::JointVelocities result(configuration_.getJointSpaceDoF());
    result = {deg2rad(velocity.Actuators.Actuator1),
        deg2rad(velocity.Actuators.Actuator2),
        deg2rad(velocity.Actuators.Actuator3),
        deg2rad(velocity.Actuators.Actuator4),
        deg2rad(velocity.Actuators.Actuator5),
        deg2rad(velocity.Actuators.Actuator6)};
    return result;
}

crf::expected<crf::utility::types::JointAccelerations> KinovaJaco2::getJointAccelerations() {
    logger_->debug("getJointAccelerations not supported");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::JointForceTorques> KinovaJaco2::getJointForceTorques() {
    if (!initialized_) {
        logger_->error("The API hasn't been initialized - Returning empty vector");
        return crf::Code::NotInitialized;
    }
    AngularPosition torque;
    if (apiInterface_->getAngularForceGravityFree(torque) != 1) {
        logger_->warn("Failed to get the joints torque - Returning empty vector");
        return crf::Code::RequestToDeviceFailed;
    }
    crf::utility::types::JointForceTorques result(configuration_.getJointSpaceDoF());
    result = {static_cast<double>(torque.Actuators.Actuator1),
        static_cast<double>(torque.Actuators.Actuator2),
        static_cast<double>(torque.Actuators.Actuator3),
        static_cast<double>(torque.Actuators.Actuator4),
        static_cast<double>(torque.Actuators.Actuator5),
        static_cast<double>(torque.Actuators.Actuator6)};
    return result;
}

crf::expected<crf::utility::types::TaskPose> KinovaJaco2::getTaskPose() {
    if (!initialized_) {
        logger_->error("The API hasn't been initialized - Returning empty matrix");
        return crf::Code::NotInitialized;
    }
    CartesianPosition position;
    if (apiInterface_->getTaskPose(position) != 1) {
        logger_->warn("Failed to get the end efector position - Returning empty matrix");
        return crf::Code::RequestToDeviceFailed;
    }
    return crf::utility::types::TaskPose({
        static_cast<double>(position.Coordinates.X),
        static_cast<double>(position.Coordinates.Y),
        static_cast<double>(position.Coordinates.Z)},
        crf::math::rotation::CardanXYZ({
            static_cast<double>(position.Coordinates.ThetaX),
            static_cast<double>(position.Coordinates.ThetaY),
            static_cast<double>(position.Coordinates.ThetaZ)}));
}

crf::expected<crf::utility::types::TaskVelocity> KinovaJaco2::getTaskVelocity() {
    logger_->debug("getTaskVelocity not supported");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::TaskAcceleration> KinovaJaco2::getTaskAcceleration() {
    logger_->debug("getTaskAcceleration not supported");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::TaskForceTorque> KinovaJaco2::getTaskForceTorque() {
    if (!initialized_) {
        logger_->error("The API hasn't been initialized - Returning empty vector");
        return crf::Code::NotInitialized;
    }
    CartesianPosition torque;
    if (apiInterface_->getTaskForce(torque) != 1) {
        logger_->warn("Failed to get the task torque - Returning empty vector");
        return crf::Code::RequestToDeviceFailed;
    }
    return crf::utility::types::TaskForceTorque({static_cast<double>(torque.Coordinates.X),
        static_cast<double>(torque.Coordinates.Y),
        static_cast<double>(torque.Coordinates.Z),
        static_cast<double>(torque.Coordinates.ThetaX),
        static_cast<double>(torque.Coordinates.ThetaY),
        static_cast<double>(torque.Coordinates.ThetaZ)});
}

crf::expected<bool> KinovaJaco2::setJointPositions(const bool& isSmoothTrajectory,
    const crf::utility::types::JointPositions& jointPositions,
    const crf::utility::types::JointVelocities& jointVelocities,
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->debug("setJointPositions");
    return crf::Code::NotImplemented;
}

crf::expected<bool> KinovaJaco2::setJointVelocities(const bool& isSmoothTrajectory,
    const crf::utility::types::JointVelocities& jointVelocities,
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->debug("setJointVelocities");
    if (!initialized_) {
        logger_->error("The API hasn't been initialized");
        return crf::Code::NotInitialized;
    }

    // Check if the values of velocity are within the robot limits
    for (unsigned int jointID = 0; jointID < configuration_.getJointSpaceDoF(); jointID++) {
        if (std::fabs(jointVelocities[jointID]) >
            configuration_.getJointLimits().maxVelocity[jointID]) {
            logger_->error(
                "The requested velocity ({}) for the joint {} is bigger than the maximum limit",
                jointVelocities[jointID], jointID);
            return crf::Code::BadRequest;
        }
    }

    int controlType = 0;
    if (apiInterface_->getControlType(controlType) != 1) {
        logger_->error("Failed to get the control type");
        return crf::Code::RequestToDeviceFailed;
    }
    if (controlType != 1) {
        if (apiInterface_->setAngularControl() != 1) {
            logger_->error("Failed to set the angular control");
            return crf::Code::RequestToDeviceFailed;
        }
    }
    TrajectoryPoint point = {};
    point.InitStruct();
    point.Position.Type = ANGULAR_VELOCITY;
    point.Position.HandMode = HAND_NOMOVEMENT;
    point.Position.Actuators.Actuator1 = rad2deg(jointVelocities[0]);
    point.Position.Actuators.Actuator2 = rad2deg(jointVelocities[1]);
    point.Position.Actuators.Actuator3 = rad2deg(jointVelocities[2]);
    point.Position.Actuators.Actuator4 = rad2deg(jointVelocities[3]);
    point.Position.Actuators.Actuator5 = rad2deg(jointVelocities[4]);
    point.Position.Actuators.Actuator6 = rad2deg(jointVelocities[5]);
    if (apiInterface_->sendAdvanceTrajectory(point) != 1) {
        logger_->error("Failed to send the joints velocity");
        return crf::Code::RequestToDeviceFailed;
    }
    return true;
}

crf::expected<bool> KinovaJaco2::setJointForceTorques(const bool& isSmoothTrajectory,
    const crf::utility::types::JointForceTorques& jointForceTorques) {
    logger_->warn("setJointForceTorques not supported");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> KinovaJaco2::setTaskPose(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskPose& taskPose,
    const crf::utility::types::TaskVelocity& taskVelocity,
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->warn("setTaskPose not supported due to an API bug");
    return crf::Code::NotImplemented;
}

crf::expected<bool> KinovaJaco2::setTaskVelocity(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskVelocity& taskVelocity,
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->warn("setTaskVelocity not supported");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> KinovaJaco2::setTaskForceTorque(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskForceTorque& taskForceTorque) {
    logger_->warn("setTaskForceTorque not supported");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::JointVelocities> KinovaJaco2::getProfileJointVelocities() {
    logger_->warn("getProfileJointVelocities not supported");
    return crf::Code::NotImplemented;
}

crf::expected<crf::utility::types::JointAccelerations> KinovaJaco2::getProfileJointAccelerations() {
    logger_->warn("getProfileJointAccelerations not supported");
    return crf::Code::NotImplemented;
}

crf::expected<crf::utility::types::TaskVelocity> KinovaJaco2::getProfileTaskVelocity() {
    logger_->warn("getProfileTaskVelocity not supported");
    return crf::Code::NotImplemented;
}

crf::expected<crf::utility::types::TaskAcceleration> KinovaJaco2::getProfileTaskAcceleration() {
    logger_->warn("getProfileTaskAcceleration not supported");
    return crf::Code::NotImplemented;
}

crf::expected<bool> KinovaJaco2::setProfileJointVelocities(
    const crf::utility::types::JointVelocities& jointVelocities) {
    logger_->warn("setProfileJointVelocities not supported");
    return crf::Code::NotImplemented;
}

crf::expected<bool> KinovaJaco2::setProfileJointAccelerations(
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->warn("setProfileJointAccelerations not supported");
    return crf::Code::NotImplemented;
}

crf::expected<bool> KinovaJaco2::setProfileTaskVelocity(
    const crf::utility::types::TaskVelocity& taskVelocity) {
    logger_->warn("setProfileTaskVelocity not supported");
    return crf::Code::NotImplemented;
}

crf::expected<bool> KinovaJaco2::setProfileTaskAcceleration(
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->warn("setProfileTaskAcceleration not supported");
    return crf::Code::NotImplemented;
}

crf::expected<bool> KinovaJaco2::setGravity(const std::array<double, 3>& gravity) {
    if (!initialized_) {
        logger_->error("The API hasn't been initialized");
        return crf::Code::NotInitialized;
    }
    return crf::Code::NotImplemented;
}

crf::expected<bool> KinovaJaco2::softStop() {
    logger_->debug("softStop");
    if (!initialized_) {
        logger_->error("The API hasn't been initialized");
        return crf::Code::NotInitialized;
    }
    if (apiInterface_->eraseAllTrajectories() != 1) {
        logger_->error("Failed to stop - Trajectories on the FIFO can't be deleted");
        return crf::Code::RequestToDeviceFailed;
    }
    logger_->info("The Trajectories on the FIFO has been deleted");
    return setJointVelocities(false,
        crf::utility::types::JointVelocities(configuration_.getJointSpaceDoF()));
}

crf::expected<bool> KinovaJaco2::hardStop() {
    return softStop();
}

crf::expected<bool> KinovaJaco2::setBrakes(std::vector<bool> brakesStatus) {
    logger_->warn("Kinova Jaco 2 has no brakes");
    return crf::Code::MethodNotAllowed;
}

crf::expected<std::vector<bool>> KinovaJaco2::getBrakes() {
    logger_->warn("Kinova Jaco 2 has no brakes");
    return crf::Code::MethodNotAllowed;
}

std::set<Code> KinovaJaco2::robotStatus() {
    logger_->debug("robotStatus");
    std::set<Code> statusList;

    if (!initialized_) {
        statusList.insert(crf::Code::NotInitialized);
    }
    statusList.insert(errorCode_);
    return statusList;
}

crf::expected<bool> KinovaJaco2::resetFaultState() {
    logger_->warn("resetFaultState not supported");
    return crf::Code::MethodNotAllowed;
}

std::shared_ptr<RobotConfiguration> KinovaJaco2::getConfiguration() {
    return std::make_shared<RobotConfiguration>(configuration_);
}

// Private

double KinovaJaco2::deg2rad(float angle) {
    return static_cast<double>(angle*(M_PI/180));
}

float KinovaJaco2::rad2deg(double angle) {
    return static_cast<float>(angle*(180/M_PI));
}

crf::Code KinovaJaco2::parseErrorCode(const int& ec) {
    switch (ec) {
        case 2002:
            return crf::Code::KJ_ErrorLoadCommunicationDll;
            break;
        case 2006:
            return crf::Code::KJ_ErrorInitCommunicationMethod;
            break;
        case 2007:
            return crf::Code::KJ_ErrorCloseMethod;
            break;
        case 2008:
            return crf::Code::KJ_ErrorGetDeviceCountMethod;
            break;
        case 2009:
            return crf::Code::KJ_ErrorSendPacketMethod;
            break;
        case 2010:
            return crf::Code::KJ_ErrorSetActiveDeviceMethod;
            break;
        case 2011:
            return crf::Code::KJ_ErrorGetDevicesListMethod;
            break;
        case 2013:
            return crf::Code::KJ_ErrorScanForNewDevice;
            break;
        case 2014:
            return crf::Code::KJ_ErrorGetActiveDeviceMethod;
            break;
        case 2015:
            return crf::Code::KJ_ErrorOpenRS485Activate;
            break;
        case -1:
            return crf::Code::KJ_InputOutputError;
            break;
        case -2:
            return crf::Code::KJ_InvalidParameter;
            break;
        case -3:
            return crf::Code::KJ_AccessDenied;
            break;
        case -4:
            return crf::Code::KJ_NoSuchDevice;
            break;
        case -5:
            return crf::Code::KJ_EntityNotFound;
            break;
        case -6:
            return crf::Code::KJ_ResourceBusy;
            break;
        case -7:
            return crf::Code::KJ_OperationTimedOut;
            break;
        case -8:
            return crf::Code::KJ_Overflow;
            break;
        case -9:
            return crf::Code::KJ_PipeError;
            break;
        case -10:
            return crf::Code::KJ_SystemCallInterrupted;
            break;
        case -11:
            return crf::Code::KJ_InsufficientMemory;
            break;
        case -12:
            return crf::Code::KJ_OperationNotSupported;
            break;
        case 1015:
            return crf::Code::KJ_ErrorNoDeviceFound;
            break;
        case 2101:
            return crf::Code::KJ_ErrorAPINotInitialize;
            break;
        default:
            return crf::Code::KJ_OtherError;
            break;
    }
}

}  // namespace crf::actuators::robot
