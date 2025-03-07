/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 *
 * Contributors: Alessandro Mosca CERN EN/STI/ECE, Giacomo Lunghi CERN EN/STI/ECE,
 * Jorge Camarero Vera CERN EN/STI/ECE, Carlos Veiga Almagro CERN EN/STI/ECE,
 * David Blanco Mulero CERN EN/STI/ECE,
 *  ==================================================================================================
 */

#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>

#include "XsensMT/XsensMT.hpp"

#define DEVICE_ID "027000CD"  // Hardcoded particular XSENS ID

namespace crf {
namespace sensors {
namespace imu {

XsOnDataAvailableCallback::XsOnDataAvailableCallback(UpdateFunction updateFunction):
    logger_("XsOnDataAvailableCallback"),
    updateFunction_(updateFunction) {
    logger_->debug("CTor");
    if (!updateFunction_) {
        logger_->error("Received empty updateFunction!");
    }
}

XsOnDataAvailableCallback::~XsOnDataAvailableCallback() {
    logger_->debug("DTor");
}

void XsOnDataAvailableCallback::onDataAvailable(XsDevice* dev, const XsDataPacket* packet) {
    logger_->debug("onDataAvailable: received new data from device: {}", dev->productCode());
    Packets::IMUDataPacket imuPacket = translate(packet);
    logger_->debug("IMUDataPacket: {}", imuPacket);
    updateFunction_(imuPacket);
}

IMUDataPacket XsOnDataAvailableCallback::translate(const XsDataPacket* packet) {
    IMUDataPacket imuPacket{};
    bool copyLla = true;
    bool copyVelocity = true;
    bool copyAcc = true;
    bool copyGyro = true;

    if (packet->empty()) {
        logger_->warn("Received empty XsDataPacket");
        return imuPacket;
    }

    if (!packet->containsPositionLLA()) {
        logger_->debug("Packet does not contain LLA");
        copyLla = false;
    }
    XsVector lla = packet->positionLLA();
    if (!packet->containsVelocity()) {
        logger_->debug("Packet does not contain velocity");
        copyVelocity = false;
    }
    XsVector velocity = packet->velocity();
    if (!packet->containsCalibratedAcceleration()) {
        logger_->debug("Packet does not contain calibrated acceleration");
        copyAcc = false;
    }
    XsVector acceleration = packet->calibratedAcceleration();
    if (!packet->containsCalibratedGyroscopeData()) {
        logger_->debug("Packet does not contain calibrated gyro data");
        copyGyro = false;
    }
    XsVector gyro = packet->calibratedGyroscopeData();
    XsEuler euler = packet->orientationEuler();
    XsMatrix orientationMatrix = packet->orientationMatrix();
    XsQuaternion orientationQuaternion = packet->orientationQuaternion();

    for (int i=0; i < 3; i++) {
        if (copyLla) imuPacket.position[i] = lla[i];
        if (copyVelocity) imuPacket.velocity[i] = velocity[i];
        if (copyAcc) imuPacket.acceleration[i] = acceleration[i];
        if (copyGyro) imuPacket.gyro[i] = gyro[i];
    }
    imuPacket.roll = euler.roll();
    imuPacket.pitch = euler.pitch();
    imuPacket.yaw = euler.yaw();
    for (int i=0; i < 3; i++) {
        for (int j=0; j < 3; j++) {
            imuPacket.rot_matrix[i][j] = orientationMatrix[i][j];
        }
    }
    for (int i=0; i < 4; i++) {
        imuPacket.quaternion[i] = orientationQuaternion[i];
    }
    return imuPacket;
}

XsensMT::XsensMT(XsControl* xsControl):
    logger_("XsensMT"),
    xsControl_(xsControl),
    devicePtr_(nullptr),
    onDataAvailableCallback_(std::bind(&XsensMT::updateRecentData,
        this, std::placeholders::_1)),
    recentData_(),
    hasUnreadData_(false),
    isInitialized_(false) {
    logger_->debug("CTor");

    if (!xsControl_) {
        logger_->error("Got invalid pointer to XsControl object. XsensMT CTor failed.");
        return;
    }
}

XsensMT::~XsensMT() {
    logger_->debug("DTor");
    deinitialize();
    if (xsControl_) {
        xsControl_->destruct();
    }
}

bool XsensMT::initialize() {
    if (!xsControl_) {
        logger_->error("Incorrect XsControl interface!");
        return false;
    }

    if (isInitialized_) {
        logger_->warn("This adapter is already initialized");
        return false;
    }

    logger_->info("Setting up the connection ...");
    logger_->debug("Scanning all ports ... ");

    XsPortInfoArray portsArray = XsScanner::scanPorts();

    if (portsArray.empty()) {
        logger_->warn("Unfortunately no ports detected ... Returning directly.");
        return false;
    }

    for (const auto& port : portsArray) {
        logger_->debug("Detected port:");
        logger_->debug("\t name: {}", port.portName());
        logger_->debug("\t deviceId: {}", port.deviceId());
    }

    auto portInfoIt = std::find_if(portsArray.begin(), portsArray.end(),
        [](const XsPortInfo& portInfo) {
            return portInfo.deviceId().toString() == DEVICE_ID; });

    if (portInfoIt == portsArray.end()) {
        logger_->warn("Hardcoded device: {} not found", DEVICE_ID);
        return false;
    }

    if (!xsControl_->openPort(*portInfoIt)) {
        logger_->warn("Unable to open port: {}", portInfoIt->portName());
        return false;
    }

    XsDevicePtrArray devicesArray = xsControl_->mainDevices();

    if (devicesArray.empty()) {
        logger_->warn("No devices available.");
        return false;
    }

    for (auto device : devicesArray) {
        logger_->debug("Detected device:");
        logger_->debug("\t portName: {}", device->portName());
        logger_->debug("\t deviceId: {}", device->deviceId());
        logger_->debug("\t productCode: {}", device->productCode());
    }

    devicePtr_ = devicesArray[0];  // let's assume we only have one ...
    isInitialized_ = enableMeasurement();
    return isInitialized_;
}

bool XsensMT::deinitialize() {
    logger_->info("Going to close hardware");
    if (!isInitialized_) {
        logger_->debug("Device is uninitialized");
        return false;
    }
    // We don't care about the result of gotoConfig; This operation cannot fail;
    disableMeasurement();
    logger_->info("Going to close port: {}", devicePtr_->portName());
    xsControl_->closePort(devicePtr_);
    isInitialized_ = false;
    return !isInitialized_;
}


IMUDataPacket XsensMT::getIMUData() {
    hasUnreadData_ = false;
    return recentData_;
}


bool XsensMT::hasFreshData() const {
    return hasUnreadData_;
}

bool XsensMT::enableMeasurement() {
    logger_->info("Registering a callback handler and going to measurement mode");
    if (!devicePtr_) {
        logger_->warn("Cannot enableMeasurement, no device!");
        return false;
    }
    devicePtr_->addCallbackHandler(&onDataAvailableCallback_);
    return devicePtr_->gotoMeasurement();
}

bool XsensMT::disableMeasurement() {
    if (!devicePtr_) {
        logger_->warn("Cannot disableMeasurement, no device!");
        return false;
    }
    logger_->info("Disabling measurement");
    devicePtr_->clearCallbackHandlers();
    return devicePtr_->gotoConfig();
}

void XsensMT::updateRecentData(const IMUDataPacket& packet) {
    recentData_ = packet;
    hasUnreadData_ = true;
}

}  // namespace imu
}  // namespace sensors
}  // namespace crf
