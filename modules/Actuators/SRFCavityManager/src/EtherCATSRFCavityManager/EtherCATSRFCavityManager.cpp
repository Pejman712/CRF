/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Adrien Luthi CERN EN/SMM/MRO 2023
 *
 *  ===============================================================================================
 */

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <optional>
#include <chrono>

#include "SRFCavityManager/EtherCATSRFCavityManager/EtherCATSRFCavityManager.hpp"
#include "EtherCATDevices/EtherCATMotor.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"

using crf::devices::ethercatdevices::modesofoperation::ProfilePositionMode;
using crf::devices::ethercatdevices::modesofoperation::ProfileVelocityMode;

namespace crf::actuators::srfcavityManager {

EtherCATSRFCavityManager::EtherCATSRFCavityManager(const nlohmann::json& configJson,
    std::shared_ptr<devices::ethercatdevices::IEtherCATMotor> cavityMotor):
    logger_("EtherCATSRFCavityManager"),
    cavityMotor_(cavityMotor),
    isInitialized_(false),
    motorEnabled_(false),
    cavType_(CavityType::Undefined),
    cavOr_(CavityOrientation::Undefined),
    atOrigin_(false) {
    logger_->debug("CTor");
    try {
        unitFactor_ = configJson.at("unitFactor").get<float>();
        velocityProfile_ = configJson.at("velocityProfile").get<float>();
        accelerationProfile_ = configJson.at("accelerationProfile").get<float>();
        decelerationProfile_ = configJson.at("decelerationProfile").get<float>();
        referenceCavEncoder_ = configJson.at("cavityReferenceEncoder").get<int>();
        maxVelocity_ = configJson.at("maxVelocity").get<float>();
        minVelocity_ = configJson.at("minVelocity").get<float>();
        maxCurrent_ = configJson.at("maxCurrentMotor").get<int>();
        maxTorque_ = configJson.at("maxTorqueMotor").get<int>();
        orientation_ = configJson.at("motorOrientation").get<int>();
        // TODO(@adluthi): Remove these variables and calculate them according to the profiles.
        posCommandTimeout_ = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::seconds(configJson.at("positionCommandTimeoutSeconds").get<int>()));
        velCommandTimeout_ = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::seconds(configJson.at("velocityCommandTimeoutSeconds").get<int>()));
        stopCommandTimeout_ = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::seconds(configJson.at("stopCommandTimeoutSeconds").get<int>()));
        updateInterval_ = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::milliseconds(configJson.at("updateIntervalMilliseconds").get<int>()));
    } catch (const std::exception& e) {
        logger_->error("Failed to parse because: {}", e.what());
        throw std::runtime_error("Failed to parse ARISConfig configuration json file");
    }
    if (unitFactor_ <= 0) {
        throw std::invalid_argument("unitfactor_ has to be bigger than 0.");
    }
    if (velocityProfile_ <= 0) {
        throw std::invalid_argument("velocityProfile_ has to be bigger than 0.");
    }
    if (accelerationProfile_ <= 0) {
        throw std::invalid_argument("accelerationProfile_ has to be bigger than 0.");
    }
    if (decelerationProfile_ <= 0) {
        throw std::invalid_argument("decelerationProfile_ has to be bigger than 0.");
    }
    if (referenceCavEncoder_ < 0) {
        throw std::invalid_argument("referenceCavEncoder_ has to be bigger or equal to 0.");
    }
    if (maxCurrent_ <= 0) {
        throw std::invalid_argument("maxCurrent_ has to be bigger than 0.");
    }
    if (maxTorque_ <= 0) {
        throw std::invalid_argument("maxTorque_ has to be bigger than 0.");
    }
    if (std::abs(orientation_) != 1) {
        throw std::invalid_argument("orientation_ can only be equal to -1 or 1.");
    }
    if (posCommandTimeout_ <= std::chrono::seconds(0) ||
        velCommandTimeout_ <= std::chrono::seconds(0) ||
        stopCommandTimeout_ <= std::chrono::seconds(0) ||
        updateInterval_ <= std::chrono::seconds(0)) {
        throw std::invalid_argument("*Timeout and updateInterval_ have to be bigger than 0.");
    }

    oneTurnEnc_ = std::round(2*M_PI*unitFactor_);
    velocityProfile_ *= unitFactor_;
    accelerationProfile_ *= unitFactor_;
    decelerationProfile_ *= unitFactor_;
}

EtherCATSRFCavityManager::~EtherCATSRFCavityManager() {
    logger_->debug("DTor");
    deinitialize();
}

bool EtherCATSRFCavityManager::initialize() {
    logger_->debug("initialize");
    if (isInitialized_) {
        logger_->warn("Already initialized");
        return false;
    }
    if (!cavityMotor_->setMaxCurrent(maxCurrent_)) {
        logger_->error("Max current can't be set");
        return false;
    }
    if (!cavityMotor_->setMaxTorque(maxTorque_)) {
        logger_->error("Max current can't be set \n");
        return false;
    }
    isInitialized_ = true;
    // We call this getPosition() here to check if the cavitiy is at 0 and by so setting atOrigin_
    crf::expected<double> position = getPosition();
    if (!position) {
        return false;
    }
    return true;
}

bool EtherCATSRFCavityManager::deinitialize() {
    logger_->debug("deinitialize");
    if (!isInitialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!cavityMotor_->deinitialize()) {
        logger_->error("Motor can't be deinitialized");
        return false;
    }
    logger_->info("Motor deinitialized");
    isInitialized_ = false;
    motorEnabled_ = false;
    return true;
}

CavityType EtherCATSRFCavityManager::getCavityType() {
    logger_->debug("getCavityType()");
    return cavType_;
}

bool EtherCATSRFCavityManager::setCavityType(double cavLength) {
    logger_->debug("setCavityType()");
    if (cavityLenghtLHC < cavLength && cavLength <= cavityLenghtFiveCells) {
        cavType_ = CavityType::FiveCells;
        logger_->info("cavType is FIVECELL!");
    } else if (cavityLenghtFCC < cavLength && cavLength <= cavityLenghtLHC) {
        cavType_ = CavityType::LHC;
        logger_->info("cavType is LHC!");
    } else if (cavityLenghtFCC / 2 < cavLength && cavLength <= cavityLenghtFCC) {
        cavType_ = CavityType::FCC;
        logger_->info("cavType is FCC!");
    } else {
        cavType_ = CavityType::Undefined;
        return false;
    }
    return true;
}

CavityOrientation EtherCATSRFCavityManager::getCavityOrientation() {
    logger_->debug("getCavityOrientation()");
    return cavOr_;
}

void EtherCATSRFCavityManager::setCavityOrientation(CavityOrientation orientation) {
    logger_->debug("setCavityOrientation()");
    cavOr_ = orientation;
}

crf::expected<bool> EtherCATSRFCavityManager::enableMotor() {
    logger_->debug("enableMotor()");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    if (motorEnabled_) {
        return true;
    }
    crf::expected<bool> res = switchOpMode(ProfileVelocityMode);
    if (!res) {
        return res.get_response();
    }
    if (!resetMotorIfInFault()) {
        return crf::Code::RequestToDeviceFailed;
    }
    if (!cavityMotor_->shutdown()) {
        logger_->error("Shutdown can't be performed");
        return false;
    }
    if (!cavityMotor_->enableOperation()) {
        logger_->error("Motor can't be enabled for operations!");
        return false;
    }
    motorEnabled_ = true;
    return true;
}

crf::expected<double> EtherCATSRFCavityManager::getPosition() {
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    std::optional<int32_t> encoderPosition = cavityMotor_->getPosition();
    if (!encoderPosition) {
        return crf::Code::RequestToDeviceFailed;
    }
    double position = setEncPosInCavFrame(encoderPosition.value())/unitFactor_;
    // This comparison is needed because when the position is very close to the end of a turn and
    // we convert it from encoder ticks to radians the result can be higher than 2*PI due to the
    // decimals precision.
    if (std::abs(position - 2*M_PI) <= posResolution_) {
        position = 0;
    }
    if (position == 0) {
        atOrigin_ = true;
    }
    return position;
}

crf::expected<bool> EtherCATSRFCavityManager::setPosition(double targetPos, bool absolute) { // NOLINT
    logger_->debug("setPosition()");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    if (!motorEnabled_) {
        return crf::Code::ECMotorNotEnabled;
    }
    if (!resetMotorIfInFault()) {
        return crf::Code::RequestToDeviceFailed;;
    }
    if (std::abs(targetPos) > 2*M_PI) {
        logger_->error("Position requested ({}) exceeds [-2pi; 2pi]", targetPos);
        return crf::Code::BadRequest;
    }

    // Move to origin if the cavity is not already at its origin
    if (targetPos == 0 && absolute) {
        logger_->info("Target position is 0 deg.");
        return moveToOrigin();
    } else if (targetPos == 0) {
        return true;
    }

    if (absolute) {
        crf::expected<double> currPos = getPosition();
        if (!currPos) {
            return currPos.get_response();
        }
        double diff = targetPos - currPos.value();
        if (std::abs(diff) > M_PI) {
            if (targetPos < currPos.value()) {
                targetPos = diff + 2*M_PI;
            } else {
                targetPos = diff - 2*M_PI;
            }
        } else {
            targetPos = diff;
        }
    }
    crf::expected<bool> result = switchOpMode(ProfilePositionMode);
    if (!result) {
        return result.get_response();
    }
    result = setECMotorPosition(targetPos);
    if (result) {
        atOrigin_ = false;
    }
    return result;
}

crf::expected<bool> EtherCATSRFCavityManager::moveToOrigin() {
    logger_->debug("moveToOrigin()");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    if (atOrigin_) {
        return true;
    }
    if (!motorEnabled_) {
        return crf::Code::ECMotorNotEnabled;
    }
    std::optional<int32_t> temp = cavityMotor_->getPosition();
    if (!temp) {
        return crf::Code::RequestToDeviceFailed;
    }
    double encoderPosInCavFrame = setEncPosInCavFrame(temp.value());
    crf::expected<bool> result;
    if (encoderPosInCavFrame > oneTurnEnc_/2) {
        result = setECMotorPosition((oneTurnEnc_ - encoderPosInCavFrame)/unitFactor_);
        if (result) atOrigin_ = true;
        return result;
    }
    result = setECMotorPosition(-encoderPosInCavFrame/unitFactor_);
    if (result) atOrigin_ = true;
    return result;
}

crf::expected<double> EtherCATSRFCavityManager::getVelocity() {
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    if (!motorEnabled_) {
        return crf::Code::ECMotorNotEnabled;
    }
    auto temp = cavityMotor_->getVelocity();
    if (temp == std::nullopt) {
        return crf::Code::RequestToDeviceFailed;
    }
    return -temp.value()/unitFactor_;
}

crf::expected<bool> EtherCATSRFCavityManager::setVelocity(double velocity) {
    logger_->debug("setVelocity()");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    if (!motorEnabled_) {
        return crf::Code::ECMotorNotEnabled;
    }
    if (!resetMotorIfInFault()) {
        return crf::Code::RequestToDeviceFailed;;
    }
    if (velocity > maxVelocity_ || velocity < minVelocity_) {
        logger_->error("Velocity requested not allowed [{} - {}].", minVelocity_, maxVelocity_);
        return crf::Code::BadRequest;
    }
    crf::expected<bool> result = switchOpMode(ProfilePositionMode);
    if (!result) {
        return result.get_response();
    }
    //  orientation_ allows to chose the rotation's direction of the motor.
    if (!cavityMotor_->setVelocity(orientation_*velocity*unitFactor_, accelerationProfile_,
        decelerationProfile_)) {
        logger_->error("Impossible to setVelocity to the cavity motor!");
        return crf::Code::RequestToDeviceFailed;
    }
    return waitUntilVelocityReached(velocity);
}

crf::expected<bool> EtherCATSRFCavityManager::isTurning() {
    logger_->debug("isTurning()");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    if (!motorEnabled_) {
        return crf::Code::ECMotorNotEnabled;
    }
    crf::expected<double> res = getVelocity();
    if (!res) {
        return res.get_response();
    }
    if (res.value() < motorVelocityNoise_) {
        logger_->debug("curr vel = {}", res.value());
        return false;
    }
    return true;
}

crf::expected<bool> EtherCATSRFCavityManager::stop() {
    logger_->debug("stop()");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    if (!motorEnabled_) {
        return crf::Code::ECMotorNotEnabled;
    }
    if (!cavityMotor_->stop()) {
        return false;
    }
    crf::expected<bool> res;
    // Wait fot the motion to stop
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::microseconds>(end - start) < stopCommandTimeout_) {  // NOLINT
        end = std::chrono::high_resolution_clock::now();
        crf::expected<bool> result = isTurning();
        if (!result) {
            return result.get_response();
        }
        if (!(result.value())) {
            return true;
        }
        std::this_thread::sleep_for(updateInterval_);
    }
    logger_->error("The cavity did not stop after {} microseconds", stopCommandTimeout_.count());
    return crf::Code::RequestTimeout;
}

crf::expected<bool> EtherCATSRFCavityManager::setECMotorPosition(double position) {
    logger_->debug("setECMotorPosition()");
    if (!cavityMotor_->setPosition(orientation_*position*unitFactor_, velocityProfile_,
        accelerationProfile_, decelerationProfile_, true)) {
        logger_->error("Failed to set the position on the cavity motor!");
        return crf::Code::RequestToDeviceFailed;
    }
    // Wait for the motion to finish
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::microseconds>(end - start) < posCommandTimeout_) {  // NOLINT
        end = std::chrono::high_resolution_clock::now();
        std::optional<bool> result = cavityMotor_->targetReached();
        if (result) {
            if (result.value()) return true;
        }
        std::this_thread::sleep_for(updateInterval_);
    }
    logger_->error("The cavity did not reached the requested position {} after {} microseconds",
        position, posCommandTimeout_.count());
    // We stop for the motor for safety reasons.
    crf::expected<double> request = stop();
    if (!request) {
        return request.get_response();
    }
    return crf::Code::RequestTimeout;
}

crf::expected<bool> EtherCATSRFCavityManager::waitUntilVelocityReached(double targetVelocity) {
    logger_->debug("waitUntilVelocityReached()");
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::microseconds>(end - start) < velCommandTimeout_) {  // NOLINT
        end = std::chrono::high_resolution_clock::now();
        crf::expected<double>currentVelocity = getVelocity();
        if (!currentVelocity) {
            return currentVelocity.get_response();
        }
        if (std::abs(targetVelocity-currentVelocity.value()) < velResolution_) {
            logger_->info("Constant velocity reached.");
            return true;
        }
        std::this_thread::sleep_for(updateInterval_);
    }
    logger_->error("The cavity did not reached the requested velocity {} after {} microseconds",
        targetVelocity, velCommandTimeout_.count());
    // We stop for the motor for safety reasons.
    crf::expected<double> request = stop();
    if (!request) {
        return request.get_response();
    }
    return crf::Code::RequestTimeout;
}

double EtherCATSRFCavityManager::setEncPosInCavFrame(int encoderPos) {
    double numberTurnDone = static_cast<double>(encoderPos - referenceCavEncoder_)/oneTurnEnc_;
    double numberTurnFractional = numberTurnDone - std::floor(numberTurnDone);
    double posCavityEncoder = numberTurnFractional*oneTurnEnc_;
    if (posCavityEncoder < 0) {
        posCavityEncoder += oneTurnEnc_;
    }
    return oneTurnEnc_ - posCavityEncoder;
}

crf::expected<bool> EtherCATSRFCavityManager::switchOpMode(int8_t opMode) {
    std::optional<int8_t>  operationMode = cavityMotor_->getModeOfOperation();
    if (!operationMode) {
        return crf::Code::RequestToDeviceFailed;
    }
    if (operationMode.value() == opMode) {
        return true;
    }
    if (!cavityMotor_->setModeOfOperation(opMode)) {
        logger_->error("Failed to set the mode of operation");
        return crf::Code::RequestToDeviceFailed;
    }
    return true;
}

bool EtherCATSRFCavityManager::resetMotorIfInFault() {
    logger_->debug("resetMotorIfInFault()");
    std::optional<bool> result = cavityMotor_->inFault();
    if (!result) {
        logger_->error("Communication problem with the motor.");
        return false;
    }
    if (!result.value()) {
        logger_->info("Motor not in fault.");
        return true;
    }
    logger_->warn("Motor in fault. Trying to reset the fault...");
    if (!cavityMotor_->faultReset()) {
        logger_->error("Fault reset can't be performed.");
        return false;
    }
    return true;
}

}  // namespace crf::actuators::srfcavityManager
