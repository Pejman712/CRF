/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <algorithm>
#include <memory>
#include <vector>

#include "CERNBot2/CERNBot2.hpp"

#include "CANOpenDevices/CANOpenContext.hpp"
#include "CANOpenDevices/CANOpenMotors/MaxonEPOS2.hpp"
#include "LinearStage/LinearStageConfiguration.hpp"
#include "LinearStage/CANOpenLinearStage/CANOpenLinearStage.hpp"
#include "RobotBase/RobotBaseDefaultKinematics.hpp"

#define AMPS_TO_MILLIAMPS 1000

namespace crf::actuators::robotbase {

CERNBot2::CERNBot2(std::shared_ptr<communication::cansocket::ICANSocket> socket,
    const nlohmann::json& configuration) :
    logger_("CERNBot2"),
    initialized_(false),
    radToRPMConverter_(60/(2*M_PI)),
    motorsDirection_({1, -1, 1, -1}),
    gearBoxReduction_(36),
    socket_(socket),
    motors_(),
    maximumTaskVelocity_(),
    maximumWheelsVelocity_(0),
    maximumWheelsAcceleration_(0) {
    logger_->debug("CTor");
    configuration_ = std::make_shared<RobotBaseConfiguration>();
    if (!configuration_->parse(configuration)) {
        logger_->error("Provided configuration file is not valid");
        throw std::invalid_argument("Provided configuration file is not valid");
    }

    auto linStageConfiguration = std::make_shared<linearstage::LinearStageConfiguration>();
    if ((configuration.find("LinearStage") == configuration.end()) ||
        (!linStageConfiguration->parse(configuration["LinearStage"]))) {
        logger_->error("Provided configuration file is not valid for linear stage");
        throw std::invalid_argument("Provided configuration file is not valid for linear stage");
    }

    ctx_ = std::make_shared<devices::canopendevices::CANOpenContext>(socket_);
    ctx_->setSyncFrequency(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::microseconds(configuration_->getRTLoopTime())));
    ctx_->setGuardFrequency(std::chrono::milliseconds(150));
    auto motorFrontLeft = std::make_shared<devices::canopendevices::MaxonEPOS2>(0x03, socket_);
    auto motorFrontRight = std::make_shared<devices::canopendevices::MaxonEPOS2>(0x04, socket_);
    auto motorBackLeft = std::make_shared<devices::canopendevices::MaxonEPOS2>(0x06, socket_);
    auto motorBackRight = std::make_shared<devices::canopendevices::MaxonEPOS2>(0x05, socket_);
    auto motorLinStage = std::make_shared<devices::canopendevices::MaxonEPOS2>(0x07, socket_);

    motors_.push_back(motorFrontLeft);
    motors_.push_back(motorFrontRight);
    motors_.push_back(motorBackLeft);
    motors_.push_back(motorBackRight);

    linearStage_ = std::make_shared<linearstage::CANOpenLinearStage>(motorLinStage,
        linStageConfiguration);

    ctx_->addDevice(motorFrontLeft);
    ctx_->addDevice(motorFrontRight);
    ctx_->addDevice(motorBackLeft);
    ctx_->addDevice(motorBackRight);
    ctx_->addDevice(motorLinStage);

    kinematics_.reset(new RobotBaseDefaultKinematics(*configuration_));

    maximumTaskVelocity_ = configuration_->getTaskLimits().maximumVelocity;
    maximumWheelsVelocity_ = configuration_->getRobotParameters().maximumWheelsVelocity;
    maximumWheelsAcceleration_ = configuration_->getRobotParameters().maximumWheelsAcceleration;
}

CERNBot2::CERNBot2(const nlohmann::json& configuration,
    const std::vector<std::shared_ptr<devices::canopendevices::ICANOpenMotor>>& motors,
    std::shared_ptr<devices::canopendevices::ICANOpenContext> ctx,
    std::shared_ptr<linearstage::ILinearStage> linearStage) :
    logger_("CERNBot2"),
    initialized_(false),
    radToRPMConverter_(60/(2*M_PI)),
    motorsDirection_({1, -1, 1, -1}),
    gearBoxReduction_(36),
    motors_(motors),
    ctx_(ctx),
    linearStage_(linearStage),
    maximumTaskVelocity_(),
    maximumWheelsVelocity_(0),
    maximumWheelsAcceleration_(0) {
    logger_->debug("CTor");
    configuration_ = std::make_shared<RobotBaseConfiguration>();
    if (!configuration_->parse(configuration)) {
        logger_->error("Provided configuration file is not valid");
        throw std::invalid_argument("Provided configuration file is not valid");
    }

    kinematics_.reset(new RobotBaseDefaultKinematics(*configuration_));

    maximumTaskVelocity_ = configuration_->getTaskLimits().maximumVelocity;
    maximumWheelsVelocity_ = configuration_->getRobotParameters().maximumWheelsVelocity;
    maximumWheelsAcceleration_ = configuration_->getRobotParameters().maximumWheelsAcceleration;
}

CERNBot2::~CERNBot2() {
    logger_->debug("DTor");
    if (initialized_) {
        deinitialize();
    }
}

bool CERNBot2::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }
    if (!ctx_->initialize()) {
        logger_->warn("Could not initialize context");
        return false;
    }

    uint32_t maximumAccelerationWheel = static_cast<uint32_t>(
        maximumWheelsAcceleration_*radToRPMConverter_*gearBoxReduction_);

    for (size_t i=0; i < motors_.size(); i++) {
        if (!motors_[i]->initialize()) {
            ctx_->deinitialize();
            logger_->warn("Could not initialize motor {}", i);
            return false;
        }
        if (motors_[i]->inFault() && !motors_[i]->faultReset()) {
            ctx_->deinitialize();
            logger_->warn("Could not reset the fault state for motor {}", i);
            return false;
        }
        if (!motors_[i]->setMaxAcceleration(maximumAccelerationWheel*3)) {
            ctx_->deinitialize();
            logger_->warn("Could not set max acceleration for motor {}", i);
            return false;
        }
        if (!motors_[i]->setQuickstopDeceleration(maximumAccelerationWheel*3)) {
            ctx_->deinitialize();
            logger_->warn("Could not set quickstop deceleration for motor {}", i);
            return false;
        }
        if (!motors_[i]->setProfileAcceleration(maximumAccelerationWheel)) {
            ctx_->deinitialize();
            logger_->warn("Could not set profile acceleration for motor {}", i);
            return false;
        }
        if (!motors_[i]->setProfileDeceleration(maximumAccelerationWheel)) {
            ctx_->deinitialize();
            logger_->warn("Could not set profile deceleration for motor {}", i);
            return false;
        }
    }

    for (size_t i=0; i < motors_.size(); i++) {
        if (!motors_[i]->shutdown()) {
            ctx_->deinitialize();
            logger_->warn("Could not shutdown motor {}", i);
            return false;
        }
        if (!motors_[i]->enableOperation()) {
            ctx_->deinitialize();
            logger_->warn("Could not enable motor {}", i);
            return false;
        }
    }

    if (!linearStage_->initialize()) {
        logger_->warn("Could not initialize linear stage");
        ctx_->deinitialize();
        return false;
    }

    initialized_ = true;
    return true;
}

bool CERNBot2::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    linearStage_->deinitialize();
    for (size_t i=0; i < motors_.size(); i++) {
        motors_[i]->disableOperation();
        motors_[i]->deinitialize();
    }

    ctx_->deinitialize();
    initialized_ = false;
    return true;
}

boost::optional<crf::utility::types::TaskPose> CERNBot2::getTaskPose() {
    return boost::none;
}

boost::optional<crf::utility::types::TaskVelocity> CERNBot2::getTaskVelocity() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    auto velocities = getMotorsVelocities();
    if (!velocities) {
        return boost::none;
    }
    return kinematics_->getTaskVelocity(velocities.value());
}

boost::optional<std::vector<float> > CERNBot2::getMotorsVelocities() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    if (errorActive()) {
        logger_->error("One of the motors is not enabled or not alive");
        return boost::none;
    }
    std::vector<float> velocities;
    for (size_t i=0; i < motors_.size(); i++) {
        auto velOpt = motors_[i]->getVelocity();
        if (!velOpt) {
            logger_->warn("Failed to get motor velocity for wheel {}", i);
            return boost::none;
        }

        float vel = motorsDirection_[i]*static_cast<float>(velOpt.value())/
            (gearBoxReduction_*radToRPMConverter_);
        velocities.push_back(vel);
    }

    return velocities;
}

boost::optional<std::vector<float> > CERNBot2::getMotorsCurrent() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    if (errorActive()) {
        logger_->error("One of the motors is not enabled or not alive");
        return boost::none;
    }
    std::vector<float> currents;
    for (size_t i=0; i < motors_.size(); i++) {
        auto curOpt = motors_[i]->getCurrent();
        if (!curOpt) {
            return boost::none;
        }
        float cur = static_cast<float>(curOpt.value())/AMPS_TO_MILLIAMPS;
        currents.push_back(cur);
    }
    return currents;
}

bool CERNBot2::setTaskVelocity(const crf::utility::types::TaskVelocity& velocity) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (errorActive()) {
        logger_->error("One of the motors is not enabled or not alive");
        return false;
    }
    float maxDivisionFactor = 1;
    for (int i=0; i < 6; i++) {
        if (fabs(velocity[i]) > maximumTaskVelocity_[i]) {
            float divisionFactor = maximumTaskVelocity_[i]/fabs(velocity[i]);
            if (divisionFactor < maxDivisionFactor) {
                maxDivisionFactor = divisionFactor;
            }
        }
    }
    auto nextVelocities = velocity;
    auto motorsVelocities = kinematics_->getWheelsVelocity(
        crf::utility::types::TaskVelocity(nextVelocities.raw()*maxDivisionFactor));
    return setWheelsVelocity(motorsVelocities.get());
}

bool CERNBot2::setWheelsVelocity(const std::vector<float> velocity) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (errorActive()) {
        logger_->error("One of the motors is not enabled or not alive");
        return false;
    }
    if (velocity.size() != configuration_->getNumberOfWheels()) {
        logger_->error("Input vector wrong size");
        return false;
    }

    float maxVelocity = 0;
    float scalingFactor = 1;
    for (size_t i=0; i < motors_.size(); i++) {
        maxVelocity = fabs(velocity[i]) > maxVelocity ? fabs(velocity[i]) : maxVelocity;
    }
    if (maxVelocity > maximumWheelsVelocity_) {
        scalingFactor = maximumWheelsVelocity_/maxVelocity;
    }
    for (size_t i=0; i < motors_.size(); i++) {
        float setpoint = velocity[i]*radToRPMConverter_
            *motorsDirection_[i]*gearBoxReduction_*scalingFactor;
        if (!motors_[i]->setVelocity(static_cast<int32_t>(setpoint))) {
            stopBase();
            return false;
        }
    }
    return true;
}

bool CERNBot2::stopBase() {
    logger_->debug("stopBase");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    for (size_t i=0; i < motors_.size(); i++) {
        motors_[i]->setVelocity(0);
    }
    return true;
}

bool CERNBot2::errorActive() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    bool result = false;
    for (size_t i=0; i < motors_.size(); i++) {
        result = result || !motors_[i]->isAlive() || !motors_[i]->isEnabled();
    }
    return result;
}

bool CERNBot2::acknowledgeError() {
    logger_->debug("acknowledgeError");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    for (size_t i=0; i < motors_.size(); i++) {
        if (motors_[i]->isEnabled()) {
            continue;
        }
        if (motors_[i]->inFault()) {
            if (!motors_[i]->faultReset()) {
                logger_->warn("Could not reset fault of motor {}", i);
                return false;
            }
        }
        if (!motors_[i]->enableOperation()) {
            logger_->warn("Could not enable operation of motor {}", i);
            return false;
        }
    }
    return !errorActive();
}

std::shared_ptr<RobotBaseConfiguration> CERNBot2::getConfiguration() {
    logger_->debug("getConfiguration");
    return configuration_;
}

std::shared_ptr<linearstage::ILinearStage> CERNBot2::getLinearStage() {
    logger_->debug("getLinearStage");
    return linearStage_;
}

bool CERNBot2::setMaximumWheelsAcceleration(float acceleration)  {
    logger_->debug("setMaximumWheelsAcceleration");
    if (acceleration > configuration_->getRobotParameters().maximumWheelsAcceleration) {
        logger_->warn("CAN't set higher wheels acceleration than the maximum one");
        return false;
    }
    if (acceleration < 0) {
        logger_->warn("Requested acceleration must be higher than 0");
        return false;
    }
    maximumWheelsAcceleration_ = acceleration;
    if (initialized_) {
        uint32_t maximumAccelerationWheel = static_cast<uint32_t>(
            maximumWheelsAcceleration_*radToRPMConverter_*gearBoxReduction_);
        for (size_t i = 0; i < motors_.size(); i++) {
            if (!motors_[i]->setProfileAcceleration(maximumAccelerationWheel)) {
                logger_->warn("Could not set profile acceleration");
                return false;
            }
            if (!motors_[i]->setProfileDeceleration(maximumAccelerationWheel)) {
                logger_->warn("Could not set profile acceleration");
                return false;
            }
        }
    }
    return true;
}

bool CERNBot2::setMaximumWheelsVelocity(float velocity)  {
    logger_->debug("setMaximumWheelsVelocity");
    if (velocity > configuration_->getRobotParameters().maximumWheelsVelocity) {
        logger_->warn("CAN't set higher wheels velocity than the maximum one");
        return false;
    }
    if (velocity < 0) {
        logger_->warn("Requested velocity must be higher than 0");
        return false;
    }
    maximumWheelsVelocity_ = velocity;
    return true;
}

bool CERNBot2::setMaximumTaskVelocity(const utility::types::TaskVelocity& velocity)  {
    logger_->debug("setMaximumTaskVelocity");
    for (int i = 0; i < 6; i++) {
        if (velocity[i] > configuration_->getTaskLimits().maximumVelocity[i]) {
            logger_->warn("CAN't set higher wheels velocity than the maximum one");
            return false;
        }
        if (velocity[i] < 0) {
            logger_->warn("Requested velocity must be higher than 0");
            return false;
        }
    }
    maximumTaskVelocity_ = velocity;
    return true;
}

float CERNBot2::getMaximumWheelsAcceleration() const {
    return maximumWheelsAcceleration_;
}

float CERNBot2::getMaximumWheelsVelocity() const {
    return maximumWheelsVelocity_;
}

utility::types::TaskVelocity CERNBot2::getMaximumTaskVelocity() const {
    return maximumTaskVelocity_;
}

}  // namespace crf::actuators::robotbase
