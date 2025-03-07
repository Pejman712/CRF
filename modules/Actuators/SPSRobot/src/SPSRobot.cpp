/* © Copyright CERN 2020.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi EN/SMM/MRO
 *
 *  ==================================================================================================
*/

#define NUMBER_OF_ATTEMPTS 10

#include <algorithm>
#include <memory>
#include <vector>

#include "SPSRobot/SPSRobot.hpp"

#include "RobotBase/RobotBaseDefaultKinematics.hpp"

#define AMPS_TO_MILLIAMPS 1000

namespace crf {
namespace actuators {
namespace robotbase {

SPSRobot::SPSRobot(std::string portName,
    const nlohmann::json& configuration) :
        logger_("SPSRobot"),
        initialized_(false),
        portName_(portName),
        motors_(),
        maximumTaskVelocity_(),
        maximumWheelsVelocity_(0),
        maximumWheelsAcceleration_(0) {
            logger_->info("CTor");
            configuration_ = std::make_shared<RobotBaseConfiguration>();
            if (!configuration_->parse(configuration)) {
                logger_->error("Provided configuration file is not valid");
                throw std::invalid_argument("Provided configuration file is not valid");
            }
            base_.reset(new EtherCATRobotBase(portName_));
            kinematics_.reset(new RobotBaseDefaultKinematics(*configuration_));

            maximumTaskVelocity_ = configuration_->getTaskLimits().maximumVelocity;
            maximumWheelsVelocity_ = configuration_->getRobotParameters().maximumWheelsVelocity;
            maximumWheelsAcceleration_ =
                configuration_->getRobotParameters().maximumWheelsAcceleration;
}

SPSRobot::SPSRobot(const nlohmann::json& configuration,
    const std::shared_ptr<actuators::robotbase::IEtherCATRobotBase> base) :
        logger_("SPSRobot"),
        initialized_(false),
        motors_(),
        base_(base),
        maximumTaskVelocity_(),
        maximumWheelsVelocity_(0),
        maximumWheelsAcceleration_(0) {
            logger_->info("CTor");
            configuration_ = std::make_shared<RobotBaseConfiguration>();
            if (!configuration_->parse(configuration)) {
                logger_->error("Provided configuration file is not valid");
                throw std::invalid_argument("Provided configuration file is not valid");
            }
            kinematics_.reset(new RobotBaseDefaultKinematics(*configuration_));

            maximumTaskVelocity_ = configuration_->getTaskLimits().maximumVelocity;
            maximumWheelsVelocity_ = configuration_->getRobotParameters().maximumWheelsVelocity;
            maximumWheelsAcceleration_ =
                configuration_->getRobotParameters().maximumWheelsAcceleration;
}

SPSRobot::~SPSRobot() {
    logger_->info("DTor");
    if (initialized_) deinitialize();
}

bool SPSRobot::initialize() {
    logger_->info("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    if (base_->initialize()) {
        logger_->info("EtherCAT Base initialized\n");
    } else {
        logger_->error("Can't initialize the EtherCAT Base\n");
        return false;
    }

    auto tempMotor = base_->getMotorFrontLeft();
    if (!tempMotor) {
        logger_->error("Can't retrieve Motor Front Left\n");
        return false;
    }
    auto motorFrontLeft = tempMotor.get();

    tempMotor = base_->getMotorFrontRight();
    if (!tempMotor) {
        logger_->error("Can't retrieve Motor Front Right\n");
        return false;
    }
    auto motorFrontRight = tempMotor.get();

    tempMotor = base_->getMotorBackLeft();
    if (!tempMotor) {
        logger_->error("Can't retrieve Motor Back Left\n");
        return false;
    }
    auto motorBackLeft = tempMotor.get();

    tempMotor = base_->getMotorBackRight();
    if (!tempMotor) {
        logger_->error("Can't retrieve Motor Back Right\n");
        return false;
    }
    auto motorBackRight = tempMotor.get();

    motors_.push_back(motorFrontLeft);
    motors_.push_back(motorFrontRight);
    motors_.push_back(motorBackLeft);
    motors_.push_back(motorBackRight);

    uint32_t maximumAccelerationWheel = static_cast<uint32_t>(
        maximumWheelsAcceleration_*radToCount_);

    for (size_t i=0; i < motors_.size(); i++) {
        if (!motors_[i]->setProfileAcceleration(maximumAccelerationWheel)) {
            base_->deinitialize();
            logger_->error("Could not set ProfileAcceleration parameter"\
            "for Motor of wheel {}", i + 1);
            return false;
        }
        sleep(1);
        if (!motors_[i]->setProfileDeceleration(maximumAccelerationWheel)) {
            base_->deinitialize();
            logger_->error("Could not set ProfileDeceleration parameter"\
            "for Motor of wheel {}", i + 1);
            return false;
        }
        sleep(1);
        if (!motors_[i]->setQuickstopDeceleration(maximumAccelerationWheel*3)) {
                base_->deinitialize();
                logger_->error("Could not set QuickstopDeceleration parameter"\
                "for Motor of wheel {}", i + 1);
                return false;
        }
        sleep(1);
    }

    for (size_t i=0; i < motors_.size(); i++) {
        sleep(1);
        if (!motors_[i]->setMaxCurrent(50000)) {
            base_->deinitialize();
            logger_->error("Could not set MaxCurrent parameter for Motor of wheel {}", i + 1);
            return false;
        }
	if (!motors_[i]->setMaxTorque(50000)) {
	    base_->deinitialize();
	    return false;
	}
        sleep(1);
        if (!motors_[i]->setModeOfOperation(
            crf::devices::ethercatdevices::modesofoperation::ProfileVelocityMode)) {
            base_->deinitialize();
            logger_->error("Could not set MaxCurrent parameter for Motor of wheel {}", i + 1);
            return false;
            }
    }
    
    for (size_t i=0; i < motors_.size(); i++) {
        sleep(1);
        if (!motors_[i]->setMaxCurrent(50000)) {
            base_->deinitialize();
            logger_->error("Could not set MaxCurrent parameter for Motor of wheel {}", i + 1);
            return false;
        }
        sleep(1);
        if (!motors_[i]->setModeOfOperation(
            crf::devices::ethercatdevices::modesofoperation::ProfileVelocityMode)) {
            base_->deinitialize();
            logger_->error("Could not set MaxCurrent parameter for Motor of wheel {}", i + 1);
            return false;
            }
    }

    for (size_t i=0; i < motors_.size(); i++) {
        for (int k=0; k < NUMBER_OF_ATTEMPTS; k++) {
            auto isInFault = motors_[i]->inFault();
            if (!isInFault) {
                base_->deinitialize();
                logger_->error("Could not get if Motor of wheel {} is in fault", i + 1);
                return false;
            }

            if ((isInFault.value()) && !motors_[i]->faultReset()) {
                logger_->error("Could not reset the fault of Motor of wheel {}", i + 1);
                continue;
            }

            if (!motors_[i]->shutdown()) {
                logger_->error("Could not shutdown Motor of wheel {}", i + 1);
                continue;
            }

            if (!motors_[i]->enableOperation()) {
                logger_->error("Could not enable operation of Motor of wheel {}", i + 1);
                continue;
            } else {
                break;
            }
        }
    }

    for (size_t i=0; i < motors_.size(); i++) {
        auto enabled = motors_[i]->isEnabled();
        if (!enabled) {
            logger_->error("Could not check if Motor of wheel {} is enabled", i + 1);
            base_->deinitialize();
            return false;
        }

        if (!enabled.value()) {
            logger_->error("Motor of wheel {} did not enable", i + 1);
            base_->deinitialize();
            return false;
        }
    }

    initialized_ = true;
    return true;
}

bool SPSRobot::deinitialize() {
    logger_->info("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    base_->deinitialize();
    initialized_ = false;

    return true;
}

boost::optional<crf::utility::types::TaskPose> SPSRobot::getTaskPose() {
    logger_->error("getTaskPose has no implementaiton");
    return boost::none;
}

boost::optional<crf::utility::types::TaskVelocity> SPSRobot::getTaskVelocity() {
    if (!initialized_) {
        logger_->error("SPS Robot not initialized");
        return boost::none;
    }

    auto velocities = getMotorsVelocities();
    if (!velocities) {
        return boost::none;
    }

    return kinematics_->getTaskVelocity(velocities.get());
}

boost::optional<std::vector<float> > SPSRobot::getMotorsVelocities() {
    if (!initialized_) {
        logger_->error("SPS Robot not initialized");
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
            logger_->error("Failed to get motor velocity for wheel {}", i + 1);
            return boost::none;
        }

        float vel = static_cast<float>(velOpt.value())/(radToCount_);
        velocities.push_back(vel);
    }

    return velocities;
}

boost::optional<std::vector<float> > SPSRobot::getMotorsCurrent() {
    if (!initialized_) {
        logger_->error("SPS Robot not initialized");
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

bool SPSRobot::setTaskVelocity(const crf::utility::types::TaskVelocity& velocity){
    if (!initialized_) {
        logger_->error("SPS Robot not initialized");
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

bool SPSRobot::setWheelsVelocity(const std::vector<float> velocity) {
     if (!initialized_) {
        logger_->error("SPS Robot not initialized");
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
        float setpoint = velocity[i]*radToCount_*scalingFactor;
        if (!motors_[i]->setVelocity(static_cast<int32_t>(setpoint))) {
                stopBase();
                return false;
            }
    }

    return true;
}

bool SPSRobot::stopBase() {
    logger_->info("stopBase");
    if (!initialized_) {
        logger_->error("SPS Robot not initialized");
        return false;
    }

    for (size_t i=0; i < motors_.size(); i++) {
        motors_[i]->setVelocity(0);
    }
    return true;
}

bool SPSRobot::errorActive() {
    if (!initialized_) {
        logger_->error("SPS Robot not initialized");
        return false;
    }

    bool result = false;
    for (size_t i=0; i < motors_.size(); i++) {
        if (!motors_[i]->isAlive() || !motors_[i]->isEnabled().value()) {
            logger_->error("Motor of wheel {} not enabled or not alive", i + 1);
            result = true;
        }
    }

    return result;
}

bool SPSRobot::acknowledgeError() {
    logger_->info("acknowledgeError");
    if (!initialized_) {
        logger_->error("SPS Robot not initialized");
        return false;
    }

    for (size_t i=0; i < motors_.size(); i++) {
        if (!motors_[i]->isEnabled().value()) {
            if (motors_[i]->inFault().value()) {
                if (!motors_[i]->faultReset()) {
                    logger_->error("Could not reset fault of Motor of wheel {}", i + 1);
                    return false;
                }
            }

            if (!motors_[i]->enableOperation()) {
                logger_->error("Could not enable operation of Motor of wheel {}", i + 1);
                return false;
            }
        }
    }

    return !errorActive();
}

std::shared_ptr<RobotBaseConfiguration> SPSRobot::getConfiguration() {
    logger_->info("getConfiguration");
    return configuration_;
}

bool SPSRobot::setMaximumWheelsAcceleration(float acceleration)  {
    logger_->info("setMaximumWheelsAcceleration");
    if (acceleration > configuration_->getRobotParameters().maximumWheelsAcceleration) {
        logger_->error("Can't set higher wheels acceleration than the maximum one");
        return false;
    }

    if (acceleration < 0) {
        logger_->error("Requested acceleration must be higher than 0");
        return false;
    }

    maximumWheelsAcceleration_ = acceleration;

    if (initialized_) {
        uint32_t maximumAccelerationWheel = static_cast<uint32_t>(
            maximumWheelsAcceleration_*radToCount_);
        for (size_t i = 0; i < motors_.size(); i++) {
            if (!motors_[i]->setProfileAcceleration(maximumAccelerationWheel)) {
                logger_->error("Could not set profile acceleration");
                return false;
            }
            if (!motors_[i]->setProfileDeceleration(maximumAccelerationWheel)) {
                logger_->error("Could not set profile acceleration");
                return false;
            }
        }
    }
    return true;
}

bool SPSRobot::setMaximumWheelsVelocity(float velocity)  {
    logger_->info("setMaximumWheelsVelocity");
    if (velocity > configuration_->getRobotParameters().maximumWheelsVelocity) {
        logger_->error("Can't set higher wheels velocity than the maximum one");
        return false;
    }

    if (velocity < 0) {
        logger_->error("Requested velocity must be higher than 0");
        return false;
    }
    maximumWheelsVelocity_ = velocity;

    return true;
}

bool SPSRobot::setMaximumTaskVelocity(const utility::types::TaskVelocity& velocity)  {
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

float SPSRobot::getMaximumWheelsAcceleration() const {
    return maximumWheelsAcceleration_;
}

float SPSRobot::getMaximumWheelsVelocity() const {
    return maximumWheelsVelocity_;
}

utility::types::TaskVelocity SPSRobot::getMaximumTaskVelocity() const {
    return maximumTaskVelocity_;
}


}  // namespace robotbase
}  // namespace actuators
}  // namespace crf
