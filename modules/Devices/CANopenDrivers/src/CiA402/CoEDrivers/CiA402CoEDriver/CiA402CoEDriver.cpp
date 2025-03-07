/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <string>
#include <thread>
#include <future>
#include <cstdint>
#include <any>

#include <nlohmann/json.hpp>

#include "CANopenDrivers/CiA402/CoEDrivers/CiA402CoEDriver/CiA402CoEDriver.hpp"

namespace crf::devices::canopendrivers {

CiA402CoEDriver::CiA402CoEDriver(
    std::shared_ptr<CoEMaster> master,
    const uint64_t& id,
    const nlohmann::json& j) :
    ethercatdrivers::BasicEtherCATDriver(master, id),
    master_(master),
    id_(id),
    config_(j),
    logger_("CiA402CoEDriver - " + std::to_string(id)),
    ppmAvailable_(false),
    vomAvailable_(false),
    pvmAvailable_(false),
    ptmAvailable_(false),
    ipmAvailable_(false),
    cspAvailable_(false),
    csvAvailable_(false),
    cstAvailable_(false) {
    logger_->debug("CTor");
    positionUnit_ = config_.getPositionUnitConversion();
    velocityUnit_ = config_.getVelocityUnitConversion();
    accelerationUnit_ = config_.getAccelerationUnitConversion();
    jerkUnit_ = config_.getTorqueUnitConversion();
    torqueUnit_ = config_.getTorqueUnitConversion();
    currentUnit_ = config_.getCurrentUnitConversion();
}

CiA402CoEDriver::~CiA402CoEDriver() {
    logger_->debug("DTor");
    deinitialize();
}

bool CiA402CoEDriver::initialize() {
    logger_->debug("initialize");

    if (!BasicEtherCATDriver::initialize()) {
        logger_->warn("BasicDriver initialize failed");
        return false;
    }

    if (getStatusWord() == StatusWord::OperationEnabled) {
        logger_->info("The driver is already in operation enabled mode");
        crf::expected<bool> res = setUpGeneralRegisters();
        if (!res) {
            logger_->error("Setting up general registers failed: {}", res.get_response());
            return false;
        }
        setUpPossibleModesOfOperation();
        return true;
    }

    if (getStatusWord() == StatusWord::FaultReactionActive) {
        logger_->warn("The driver is currently in fault reaction, waiting for it to finish");
        if (!waitTransition(StatusWord::Fault)) {
            if (getStatusWord() == StatusWord::FaultReactionActive) {
                logger_->error("The driver is still in fault reaction state");
                return false;
            }
            // Otherwise the driver might have move to another (valid) state
        }
    }

    if (getStatusWord() == StatusWord::Fault) {
        logger_->info("The driver is already in fault, reseting");
        if (!resetFault()) {
            logger_->error("Could not reset fault in the driver");
            return false;
        }
    }

    // Transition from NotReady to SwitchONDisabled is done while the motors are
    // turning on, that is why we have to wait for the state change
    // transition 1 on the driver state machine
    if (getStatusWord() == StatusWord::NotReady) {
        if (!waitTransition(StatusWord::SwitchONDisabled)) return false;
        logger_->debug("The driver is now in switch on disabled mode");
    }

    // transition 2 on the driver state machine
    if (getStatusWord() == StatusWord::SwitchONDisabled) {
        if (!triggerTransition(StatusWord::Ready, ControlWord::Shutdown)) return false;
        logger_->debug("The driver is now in ready to switch on mode");
    }

    // transition 3 on the driver state machine
    if (getStatusWord() == StatusWord::Ready) {
        if (!triggerTransition(StatusWord::SwitchedON, ControlWord::SwitchON)) return false;
        logger_->debug("The driver is now in switched on mode");
    }

    // transition 4 on the driver state machine
    StatusWord sw = getStatusWord();
    if (sw == StatusWord::SwitchedON || sw == StatusWord::QuickStopActive) {
        if (!triggerTransition(StatusWord::OperationEnabled,
            static_cast<ControlWord>(ControlWord::EnableOperation | ControlWord::Halt))) {
            return false;
        }
        logger_->debug("The driver is now in operation enabled mode");
        crf::expected<bool> res = setUpGeneralRegisters();
        if (!res) {
            logger_->error("Setting up general registers failed: {}", res.get_response());
            return false;
        }
        setUpPossibleModesOfOperation();
        return true;
    }
    logger_->error("Couldn't initialize the driver, status word {}", sw);
    return false;
}

bool CiA402CoEDriver::deinitialize() {
    logger_->debug("deinitialize");

    if (!ioMapBound_) return true;

    if (maxTorqueThread_.joinable()) {
        stopMaxTorqueCheck_ = true;
        maxTorqueThread_.join();
    }

    if (getStatusWord() == StatusWord::FaultReactionActive) {
        logger_->warn("The driver is currently in fault reaction, waiting for it to finish");
        if (!waitTransition(StatusWord::Fault)) {
            if (getStatusWord() == StatusWord::FaultReactionActive) {
                logger_->error("The driver is still in fault reaction state");
                return false;
            }
            // Otherwise the driver might have moved to another (valid) state
        }
    }

    if (getStatusWord() == StatusWord::Fault) {
        logger_->info("The driver is already in fault, reseting");
        if (!resetFault()) {
            logger_->error("Could not reset fault in the driver");
            return false;
        }
    }

    // transition 7, 9, 10, 12, 15 on the driver state machine
    if (getStatusWord() != (StatusWord::SwitchONDisabled)) {
        if (!triggerTransition(StatusWord::SwitchONDisabled, ControlWord::DisableVoltage)) {
            logger_->warn("Transition to Switch ON disabled not sucessful");
            return false;
        }
    }
    if (!BasicEtherCATDriver::deinitialize()) {
        logger_->warn("BasicDriver deinitialize failed");
        return false;
    }
    return true;
}

bool CiA402CoEDriver::inFault() {
    logger_->debug("inFault");
    return getStatusWord() == StatusWord::Fault;
}

bool CiA402CoEDriver::resetFault() {
    logger_->debug("resetFault");
    if (getStatusWord() != StatusWord::Fault)  return false;

    // transition 14 on the driver state machine
    *controlWord_ = ControlWord::FaultResetPrepare;
    master_->waitForPDOExchange();
    // transition 15 on the driver state machine
    *controlWord_ = ControlWord::FaultResetActive;
    master_->waitForPDOExchange();
    return waitTransition(StatusWord::SwitchONDisabled);
}

bool CiA402CoEDriver::quickStop() {
    logger_->debug("quickStop");
    if (getStatusWord() != StatusWord::OperationEnabled) return false;
    // transition 11 on the driver state machine
    if (!triggerTransition(StatusWord::QuickStopActive, ControlWord::QuickStop)) {
        *targetPosition_ = *positionActualValue_;
        *targetVelocity_ = 0x00000000;
        *targetTorque_ = 0x0000;
        return false;
    }
    *targetPosition_ = *positionActualValue_;
    *targetVelocity_ = 0x00000000;
    *targetTorque_ = 0x0000;
    return true;
}

void CiA402CoEDriver::stop() {
    logger_->debug("stop");
    if (getModeOfOperation() == ModeOfOperation::ProfilePositionMode) {
        *controlWord_ = ppm_->deactivateWord();
    }
    if (getModeOfOperation() == ModeOfOperation::VelocityMode) {
        *controlWord_ = vom_->deactivateWord();
        *targetVelocity_ = 0x00000000;
    }
    if (getModeOfOperation() == ModeOfOperation::ProfileVelocityMode) {
        *controlWord_ = pvm_->deactivateWord();
        *targetVelocity_ = 0x00000000;
    }
    if (getModeOfOperation() == ModeOfOperation::ProfileTorqueMode) {
        *controlWord_ = ptm_->deactivateWord();
        *targetTorque_ = 0x0000;
    }
    if (getModeOfOperation() == ModeOfOperation::InterpolatedPositionMode) {
        *controlWord_ = ipm_->deactivateWord();
    }
    // Cyclic modes have no normal stop
    if (getModeOfOperation() == ModeOfOperation::CyclicSyncPositionMode) {
        logger_->warn("No way to soft stop CSP");
    }
    if (getModeOfOperation() == ModeOfOperation::CyclicSyncVelocityMode) {
        logger_->warn("No way to soft stop CSV");
    }
    if (getModeOfOperation() == ModeOfOperation::CyclicSyncTorqueMode) {
        logger_->warn("No way to soft stop CST");
    }
}

bool CiA402CoEDriver::inQuickStop() {
    logger_->debug("inQuickStop");
    return getStatusWord() == StatusWord::QuickStopActive;
}

bool CiA402CoEDriver::resetQuickStop() {
    logger_->debug("resetQuickStop");
    if (getStatusWord() != StatusWord::QuickStopActive) return false;
    return triggerTransition(StatusWord::OperationEnabled,
        static_cast<ControlWord>(ControlWord::EnableOperation | ControlWord::Halt));
}

std::vector<crf::ResponseCode> CiA402CoEDriver::getMotorStatus() {
    logger_->debug("getMotorStatus");
    std::vector<crf::ResponseCode> codes;

    while (ec_iserror()) {
        ec_errort error;
        ec_poperror(&error);

        if (error.Etype == EC_ERR_TYPE_EMERGENCY) {
            logger_->warn("EMCY error detected");
            logger_->warn("Error code: 0x{:04X}, Error Register 0x{:02X}",
                error.ErrorCode, error.ErrorReg);
            logger_->warn("Man Specific: b1:0x{:02X}, w1:0x{:04X}, w2:0x{:04X}",
                error.b1, error.w1, error.w2);
            codes.emplace_back(crf::Code::CiA301EMCY, error.ErrorCode);
        }
    }

    if (inFault()) codes.emplace_back(crf::Code::CiA402MotorInFault);
    if (inQuickStop()) codes.emplace_back(crf::Code::CiA402MotorInQuickStop);

    uint16_t statusWord = *statusWord_;

    switch (getModeOfOperation()) {
        case ModeOfOperation::ProfilePositionMode:
            if (ppm_->isFollowingError(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::PPM_FollowingError));
            if (ppm_->isTargetReached(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::PPM_TargetReached));
            if (ppm_->isSetPointAcknowledged(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::PPM_SetPointAck));
            break;
        case ModeOfOperation::ProfileVelocityMode:
            if (pvm_->isTargetReached(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::PVM_TargetReached));
            if (pvm_->isSpeedZero(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::PVM_SpeedZero));
            if (pvm_->isSpeedLimited(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::PVM_SpeedLimited));
            if (pvm_->isMaxSlippageReached(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::PVM_MaxSlippage));
            break;
        case ModeOfOperation::VelocityMode:
            if (vom_->isSpeedLimited(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::VOM_SpeedLimited));
            break;
        case ModeOfOperation::ProfileTorqueMode:
            if (ptm_->isTargetReached(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::PTM_TargetReached));
            if (ptm_->isTorqueLimited(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::PTM_TorqueLimited));
            break;
        case ModeOfOperation::InterpolatedPositionMode:
            if (ipm_->isTargetReached(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::IPM_TargetReached));
            if (ipm_->isIPMModeSet(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::IPM_ModeSet));
            if (ipm_->isFollowingError(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::IPM_FollowingError));
            break;
        case ModeOfOperation::CyclicSyncPositionMode:
            if (csp_->isDriveFollowingCommandValue(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::CSP_FollowingCommand));
            if (csp_->isFollowingError(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::CSP_FollowingError));
            break;
        case ModeOfOperation::CyclicSyncVelocityMode:
            if (csv_->isDriveFollowingCommandValue(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::CSV_FollowingCommand));
            break;
        case ModeOfOperation::CyclicSyncTorqueMode:
            if (cst_->isDriveFollowingCommandValue(statusWord))
                codes.push_back(crf::ResponseCode(crf::Code::CST_FollowingCommand));
            break;
        default:
            logger_->warn("No mode of operation detected");
            break;
    }
    return codes;
}

StatusWord CiA402CoEDriver::getStatusWord() {
    uint16_t statusWord = *statusWord_;
    logger_->debug("getStatusWord: 0x{:04X}", statusWord);
    return decodeStatusWord(statusWord);
}

crf::expected<double> CiA402CoEDriver::getPosition() {
    int32_t pos = *positionActualValue_;
    logger_->debug("getPosition: 0x{:08X}", pos);
    return static_cast<double>(pos / positionUnit_);
}

crf::expected<double> CiA402CoEDriver::getVelocity() {
    int32_t vel = *velocityActualValue_;
    logger_->debug("getVelocity: 0x{:08X}", vel);
    return static_cast<double>(vel / velocityUnit_);
}

crf::expected<double> CiA402CoEDriver::getTorque() {
    int16_t tor = *torqueActualValue_;
    logger_->debug("getTorque: 0x{:04X}", tor);
    return static_cast<double>(tor / torqueUnit_);
}

ModeOfOperation CiA402CoEDriver::getModeOfOperation() {
    int8_t mode = *modeOfOperationDisplay_;
    logger_->debug("getModeOfOperation: 0x{:02X}", mode);
    return static_cast<ModeOfOperation>(mode);
}

crf::expected<bool> CiA402CoEDriver::setModeOfOperation(const ModeOfOperation& mode) {
    logger_->debug("setModeOfOperation: 0x{:02X}", static_cast<int8_t>(mode));
    if (getModeOfOperation() == mode) return true;
    stop();
    if (mode == ModeOfOperation::ProfilePositionMode && ppmAvailable_) {
        crf::expected<bool> res = writeRegisters(ppm_->getConfigRegisters());
        if (!res) return res;
        return changeModeOfOperation(ModeOfOperation::ProfilePositionMode);
    }
    if (mode == ModeOfOperation::VelocityMode && vomAvailable_) {
        crf::expected<bool> res = writeRegisters(vom_->getConfigRegisters());
        if (!res) return res;
        return changeModeOfOperation(ModeOfOperation::VelocityMode);
    }
    if (mode == ModeOfOperation::ProfileVelocityMode && pvmAvailable_) {
        crf::expected<bool> res = writeRegisters(pvm_->getConfigRegisters());
        if (!res) return res;
        return changeModeOfOperation(ModeOfOperation::ProfileVelocityMode);
    }
    if (mode == ModeOfOperation::ProfileTorqueMode && ptmAvailable_) {
        crf::expected<bool> res = writeRegisters(ptm_->getConfigRegisters());
        if (!res) return res;
        return changeModeOfOperation(ModeOfOperation::ProfileTorqueMode);
    }
    if (mode == ModeOfOperation::InterpolatedPositionMode && ipmAvailable_) {
        crf::expected<bool> res = writeRegisters(ipm_->getConfigRegisters());
        if (!res) return res;
        return changeModeOfOperation(ModeOfOperation::InterpolatedPositionMode);
    }
    if (mode == ModeOfOperation::CyclicSyncPositionMode && cspAvailable_) {
        crf::expected<bool> res = writeRegisters(csp_->getConfigRegisters());
        if (!res) return res;
        return changeModeOfOperation(ModeOfOperation::CyclicSyncPositionMode);
    }
    if (mode == ModeOfOperation::CyclicSyncVelocityMode && csvAvailable_) {
        crf::expected<bool> res = writeRegisters(csv_->getConfigRegisters());
        if (!res) return res;
        return changeModeOfOperation(ModeOfOperation::CyclicSyncVelocityMode);
    }
    if (mode == ModeOfOperation::CyclicSyncTorqueMode && cstAvailable_) {
        crf::expected<bool> res = writeRegisters(cst_->getConfigRegisters());
        if (!res) return res;
        return changeModeOfOperation(ModeOfOperation::CyclicSyncTorqueMode);
    }
    return false;
}

crf::expected<bool> CiA402CoEDriver::setProfilePosition(
    double pos, double vel, double acc, double dec, PositionReference ref) {
    logger_->debug("setProfilePosition({}, {}, {}, {}, {})", pos, vel, acc, dec, ref);
    if (getStatusWord() == StatusWord::Fault) return crf::Code::CiA402MotorInFault;
    if (getStatusWord() != StatusWord::OperationEnabled) return crf::Code::NotInitialized;
    if (!ppmAvailable_) return crf::Code::MethodNotAllowed;
    crf::expected<bool> res = setModeOfOperation(ModeOfOperation::ProfilePositionMode);
    if (!res) return res;

    int32_t position = pos * positionUnit_;
    uint32_t velocity = vel * velocityUnit_;
    uint32_t acceleration = acc * accelerationUnit_;
    uint32_t deceleration = dec * accelerationUnit_;

    res = writeRegisters(ppm_->setTargetRegisters(velocity, acceleration, deceleration, ref));
    if (!res) return res;
    *targetPosition_ = position;
    master_->waitForPDOExchange();
    *controlWord_ = ppm_->assumeNewTargetPositonWord();
    master_->waitForPDOExchange();
    return true;
}

crf::expected<bool> CiA402CoEDriver::setVelocity(double vel, double deltaSpeedAcc,
    double deltaTimeAcc, double deltaSpeedDec, double deltaTimeDec) {
    logger_->debug("setVelocity({}, {}, {}, {}, {})",
        vel, deltaSpeedAcc, deltaTimeAcc, deltaSpeedDec, deltaTimeDec);
    if (getStatusWord() == StatusWord::Fault) return crf::Code::CiA402MotorInFault;
    if (getStatusWord() != StatusWord::OperationEnabled) return crf::Code::NotInitialized;
    if (!vomAvailable_) return crf::Code::MethodNotAllowed;
    crf::expected<bool> res = setModeOfOperation(ModeOfOperation::VelocityMode);
    if (!res) return res;

    int16_t velocity = vel * velocityUnit_;
    uint32_t deltaAcc = deltaSpeedAcc * accelerationUnit_;
    uint16_t timeAcc = deltaTimeAcc;
    uint32_t deltaDec = deltaSpeedDec* accelerationUnit_;
    uint16_t timeDec = deltaTimeDec;

    res = writeRegisters(vom_->setTargetRegisters(deltaAcc, timeAcc, deltaDec, timeDec));
    if (!res) return res;
    *targetVelocity_ = velocity;
    master_->waitForPDOExchange();
    *controlWord_ = vom_->activateWord();
    master_->waitForPDOExchange();
    return true;
}

crf::expected<bool> CiA402CoEDriver::setProfileVelocity(double vel, double acc, double dec) {
    logger_->debug("setProfileVelocity({}, {}, {})", vel, acc, dec);
    if (getStatusWord() == StatusWord::Fault) return crf::Code::CiA402MotorInFault;
    if (getStatusWord() != StatusWord::OperationEnabled) return crf::Code::NotInitialized;
    if (!pvmAvailable_) return crf::Code::MethodNotAllowed;
    crf::expected<bool> res = setModeOfOperation(ModeOfOperation::ProfileVelocityMode);
    if (!res) return res;

    int32_t velocity = vel * velocityUnit_;
    uint32_t acceleration = acc * accelerationUnit_;
    uint32_t deceleration = dec * accelerationUnit_;

    res = writeRegisters(pvm_->setTargetRegisters(acceleration, deceleration));
    if (!res) return res;
    *targetVelocity_ = velocity;
    master_->waitForPDOExchange();
    *controlWord_ = pvm_->activateWord();
    master_->waitForPDOExchange();
    return true;
}

crf::expected<bool> CiA402CoEDriver::setProfileTorque(double tor) {
    logger_->debug("setProfileTorque({})", tor);
    if (getStatusWord() == StatusWord::Fault) return crf::Code::CiA402MotorInFault;
    if (getStatusWord() != StatusWord::OperationEnabled) return crf::Code::NotInitialized;
    if (!ptmAvailable_) return crf::Code::MethodNotAllowed;
    crf::expected<bool> res = setModeOfOperation(ModeOfOperation::ProfileTorqueMode);
    if (!res) return res;

    int16_t torque = tor * torqueUnit_;

    *targetTorque_ = torque;
    master_->waitForPDOExchange();
    *controlWord_ = ptm_->activateWord();
    master_->waitForPDOExchange();
    return true;
}

crf::expected<bool> CiA402CoEDriver::setInterpolatedPosition(
    double pos, double vel, double acc, double dec) {
    logger_->debug("setInterpolatedPosition({}, {}, {}, {})", pos, vel, acc, dec);
    if (getStatusWord() == StatusWord::Fault) return crf::Code::CiA402MotorInFault;
    if (getStatusWord() != StatusWord::OperationEnabled) return crf::Code::NotInitialized;
    if (!ipmAvailable_) return crf::Code::MethodNotAllowed;
    crf::expected<bool> res = setModeOfOperation(ModeOfOperation::InterpolatedPositionMode);
    if (!res) return res;

    int32_t position = pos * positionUnit_;
    uint32_t velocity = vel * velocityUnit_;
    uint32_t acceleration = acc * accelerationUnit_;
    uint32_t deceleration = dec * accelerationUnit_;

    res = writeRegisters(ipm_->setTargetRegisters(velocity, acceleration, deceleration));
    if (!res) return res;
    *interpolatedPosition_ = position;
    master_->waitForPDOExchange();
    *controlWord_ = ipm_->setIPM();
    master_->waitForPDOExchange();
    return true;
}

crf::expected<bool> CiA402CoEDriver::setCyclicPosition(
    double pos, double posOffset, double velOffset, double torOffset) {
    logger_->debug("setCyclicPosition({}, {}, {}, {})", pos, posOffset, velOffset, torOffset);
    if (getStatusWord() == StatusWord::Fault) return crf::Code::CiA402MotorInFault;
    if (getStatusWord() != StatusWord::OperationEnabled) return crf::Code::NotInitialized;
    if (!cspAvailable_) return crf::Code::MethodNotAllowed;
    crf::expected<bool> res = setModeOfOperation(ModeOfOperation::CyclicSyncPositionMode);
    if (!res) return res;

    int32_t position = pos * positionUnit_;
    int32_t positionOffset = posOffset * positionUnit_;
    int32_t velocityOffset = velOffset * velocityUnit_;
    int16_t torqueOffset = torOffset * torqueUnit_;

    res = writeRegisters(csp_->setTargetRegisters(positionOffset, velocityOffset, torqueOffset));
    if (!res) return res;
    *targetPosition_ = position;
    return true;
}

crf::expected<bool> CiA402CoEDriver::setCyclicVelocity(
    double vel, double velOffset, double torOffset) {
    logger_->debug("setCyclicVelocity({}, {}, {})", vel, velOffset, torOffset);
    if (getStatusWord() == StatusWord::Fault) return crf::Code::CiA402MotorInFault;
    if (getStatusWord() != StatusWord::OperationEnabled) return crf::Code::NotInitialized;
    if (!csvAvailable_) return crf::Code::MethodNotAllowed;
    crf::expected<bool> res = setModeOfOperation(ModeOfOperation::CyclicSyncVelocityMode);
    if (!res) return res;

    int32_t velocity = vel * velocityUnit_;
    int32_t velocityOffset = velOffset * velocityUnit_;
    int16_t torqueOffset = torOffset * torqueUnit_;

    res = writeRegisters(csv_->setTargetRegisters(velocityOffset, torqueOffset));
    if (!res) return res;
    *targetVelocity_ = velocity;
    return true;
}

crf::expected<bool> CiA402CoEDriver::setCyclicTorque(double tor, double torOffset) {
    logger_->debug("setCyclicTorque({}, {})", tor, torOffset);
    if (getStatusWord() == StatusWord::Fault) return crf::Code::CiA402MotorInFault;
    if (getStatusWord() != StatusWord::OperationEnabled) return crf::Code::NotInitialized;
    if (!cstAvailable_) return crf::Code::MethodNotAllowed;
    crf::expected<bool> res = setModeOfOperation(ModeOfOperation::CyclicSyncTorqueMode);
    if (!res) return res;

    int16_t torque = tor * torqueUnit_;
    int16_t torqueOffset = torOffset * torqueUnit_;

    res = writeRegisters(cst_->setTargetRegisters(torqueOffset));
    if (!res) return res;
    *targetTorque_ = torque;
    return true;
}

crf::expected<bool> CiA402CoEDriver::setMaximumTorque(double torque) {
    logger_->debug("setMaximumTorque({})", torque);
    if (maxTorqueThread_.joinable()) {
        stopMaxTorqueCheck_ = true;
        maxTorqueThread_.join();
    }
    stopMaxTorqueCheck_ = false;
    maxTorqueThread_ = std::thread([this, torque] () {
        while (!stopMaxTorqueCheck_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            crf::expected<double> tqe = getTorque();
            if (!tqe) continue;
            if (std::abs(tqe.value()) < torque) continue;
            stop();
            stopMaxTorqueCheck_ = true;
        }
    });
    maxTorque_ = torque;
    return true;
}

crf::expected<double> CiA402CoEDriver::getMaximumTorque() {
    logger_->debug("getMaximumTorque");
    if (!stopMaxTorqueCheck_) return maxTorque_.load();
    crf::expected<uint16_t> maxTorque = master_->readSDO<uint16_t>(
        id_, CiA402::MaxTorque, Subindex::SUB0);
    if (!maxTorque) return maxTorque;
    return maxTorque.value() / torqueUnit_;
}

// Protected


bool CiA402CoEDriver::bindIOMap() {
    logger_->debug("bindIOMap");
    std::optional<uint8_t*> in = master_->retrieveInputs(id_);
    std::optional<uint8_t*> out = master_->retrieveOutputs(id_);
    if (!in || !out) {
        logger_->error("Can't retrieve Inputs or Outputs pointers from the IOMap");
        return false;
    }
    RxPDO* rxpdo = reinterpret_cast<RxPDO*>(in.value());
    TxPDO* txpdo = reinterpret_cast<TxPDO*>(out.value());

    // Bind references to the correspondent address
    controlWord_ = &txpdo->controlWord;
    modeOfOperation_ = &txpdo->modeOfOperation;
    targetPosition_ = &txpdo->targetPosition;
    targetVelocity_ = &txpdo->targetVelocity;
    targetTorque_ = &txpdo->targetTorque;

    statusWord_ = &rxpdo->statusWord;
    modeOfOperationDisplay_ = &rxpdo->modeOfOperationDisplay;
    positionActualValue_ = &rxpdo->positionActualValue;
    velocityActualValue_ = &rxpdo->velocityActualValue;
    torqueActualValue_ = &rxpdo->torqueActualValue;
    currentActualValue_ = &rxpdo->currentActualValue;

    *controlWord_ = ControlWord::Shutdown;
    *modeOfOperation_ = ModeOfOperation::ProfilePositionMode;
    *targetPosition_ = *positionActualValue_;
    *targetVelocity_ = 0x00000000;
    *targetTorque_ = 0x0000;

    logger_->info("PDOs binded");
    return true;
}

void CiA402CoEDriver::checkRemoteBit() {
    uint16_t statusWord = *statusWord_;
    if ((statusWord & 0x0200) == 0x0200) return;
    logger_->critical(
        "The remote bit is not active! Commands from the control word won't be effective");
}

crf::expected<bool> CiA402CoEDriver::setUpGeneralRegisters() {
    crf::expected<bool> res;
    if (config_.getPolarity()) {
        Polarity pol = config_.getPolarity().value();
        res = master_->writeSDO<uint16_t>(id_, CiA402::Polarity, Subindex::SUB0, pol);
        if (!res) return res;
    }
    if (config_.getMaxMotorSpeed()) {
        uint32_t speed = config_.getMaxMotorSpeed().value();
        res = master_->writeSDO<uint32_t>(id_, CiA402::MaxMotorSpeed, Subindex::SUB0, speed);
        if (!res) return res;
    }
    if (config_.getMaxMotorTorque()) {
        uint16_t tqe = config_.getMaxMotorTorque().value();
        res = master_->writeSDO<uint16_t>(id_, CiA402::MaxTorque, Subindex::SUB0, tqe);
        if (!res) return res;
    }
    if (config_.getMaxMotorCurrent()) {
        uint16_t crnt = config_.getMaxMotorCurrent().value();
        res = master_->writeSDO<uint16_t>(id_, CiA402::MaxCurrent, Subindex::SUB0, crnt);
        if (!res) return res;
    }
    if (config_.getMotorRatedTorque()) {
        uint32_t tqe = config_.getMotorRatedTorque().value();
        res = master_->writeSDO<uint32_t>(id_, CiA402::MotorRatedTorque, Subindex::SUB0, tqe);
        if (!res) return res;
    }
    if (config_.getMotorRatedCurrent()) {
        uint32_t crnt = config_.getMotorRatedCurrent().value();
        res = master_->writeSDO<uint32_t>(id_, CiA402::MotorRatedCurrent, Subindex::SUB0, crnt);
        if (!res) return res;
    }

    // Option codes
    if (config_.getQuickStopOptionCode()) {
        QuickStopOptionCode code = config_.getQuickStopOptionCode().value();
        res = master_->writeSDO<uint32_t>(id_, CiA402::QuickStopOptionCode, Subindex::SUB0, code);
        if (!res) return res;
    }
    if (config_.getShutdownOptionCode()) {
        ShutdownOptionCode code = config_.getShutdownOptionCode().value();
        res = master_->writeSDO<uint32_t>(id_, CiA402::ShutdownOptionCode, Subindex::SUB0, code);
        if (!res) return res;
    }
    if (config_.getDisableOperationOptionCode()) {
        DisableOperationOptionCode code = config_.getDisableOperationOptionCode().value();
        res = master_->writeSDO<uint32_t>(
            id_, CiA402::DisableOperationOptionCode, Subindex::SUB0, code);
        if (!res) return res;
    }
    if (config_.getHaltOptionCode()) {
        HaltOptionCode code = config_.getHaltOptionCode().value();
        res = master_->writeSDO<uint32_t>(id_, CiA402::HaltOptionCode, Subindex::SUB0, code);
        if (!res) return res;
    }
    if (config_.getFaultOptionCode()) {
        FaultOptionCode code = config_.getFaultOptionCode().value();
        res = master_->writeSDO<uint32_t>(
            id_, CiA402::FaultReactionOptionCode, Subindex::SUB0, code);
        if (!res) return res;
    }
    return true;
}

void CiA402CoEDriver::setUpPossibleModesOfOperation() {
    logger_->debug("setUpPossibleModesOfOperation");
    crf::expected<uint32_t> modesExp;
    if (config_.checkSupportedModes()) {
        modesExp = master_->readSDO<uint32_t>(id_, CiA402::SupportedDriveModes, Subindex::SUB0);
        if (!modesExp) {
            logger_->warn("Supported drive modes register could not be read");
            logger_->warn("Activating all modes although they might not work");
            modesExp = 0x000003EF;
        }
    } else {
        modesExp = 0x000003EF;
    }
    uint32_t modes = modesExp.value();

    if ((modes & SupportedModesMask::PPM) == SupportedModes::PPM) {
        ppm_ = std::make_shared<PPM>(config_.getProfilePositionConfig());
        ppmAvailable_ = true;
        logger_->info("Profile position mode is supported");
    }

    if ((modes & SupportedModesMask::VOM) == SupportedModes::VOM) {
        vom_ = std::make_shared<VOM>(config_.getVelocityModeConfig());
        vomAvailable_ =  true;
        logger_->info("Velocity mode is supported");
    }

    if ((modes & SupportedModesMask::PVM) == SupportedModes::PVM) {
        pvm_ = std::make_shared<PVM>(config_.getProfileVelocityConfig());
        pvmAvailable_ = true;
        logger_->info("Profile velocity mode is supported");
    }

    if ((modes & SupportedModesMask::PTM) == SupportedModes::PTM) {
        ptm_ = std::make_shared<PTM>(config_.getProfileTorqueConfig());
        ptmAvailable_ = true;
        logger_->info("Profile torque mode is supported");
    }

    if ((modes & SupportedModesMask::IPM) == SupportedModes::IPM) {
        ipm_ = std::make_shared<IPM>(config_.getInterpolatedPositionConfig());
        ipmAvailable_ = true;
        logger_->info("Interpolated position mode is supported");
    }

    if ((modes & SupportedModesMask::HOM) == SupportedModes::HOM) {
        logger_->info("Homing mode is supported");
        logger_->warn("Homing mode is not yet implemented");
        // TODO(any): Implement homing mode
    }

    if ((modes & SupportedModesMask::CSP) == SupportedModes::CSP) {
        csp_ = std::make_shared<CSP>(config_.getCyclicPositionConfig());
        cspAvailable_ = true;
        logger_->info("Cyclic synchronous position mode is supported");
    }

    if ((modes & SupportedModesMask::CSV) == SupportedModes::CSV) {
        csv_ = std::make_shared<CSV>(config_.getCyclicVelocityConfig());
        csvAvailable_ = true;
        logger_->info("Cyclic synchronous velocity mode is supported");
    }

    if ((modes & SupportedModesMask::CST) == SupportedModes::CST) {
        cst_ = std::make_shared<CST>(config_.getCyclicTorqueConfig());
        cstAvailable_ = true;
        logger_->info("Cyclic synchronous torque mode is supported");
    }
}

bool CiA402CoEDriver::triggerTransition(const StatusWord& swCondition, const ControlWord& cwAction,  // NOLINT
    const std::chrono::milliseconds& timeout) {
    logger_->debug("triggerTransition");
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    while (getStatusWord() != swCondition) {
        *controlWord_ = cwAction;
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout.count()/100));
        end = std::chrono::high_resolution_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (elapsed > timeout) {
            logger_->error("Transition into status 0x{:04X} not successful, timeout", swCondition);
            if (getStatusWord() == StatusWord::Fault) logger_->error("Driver in fault");
            return false;
        }
    }
    return true;
}

bool CiA402CoEDriver::waitTransition(const StatusWord& swCondition,
    const std::chrono::milliseconds& timeout) {
    logger_->debug("waitTransition({})", statusWordToString.at(swCondition));
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    while (getStatusWord() != swCondition) {
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout.count()/100));
        end = std::chrono::high_resolution_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (elapsed > timeout) {
            logger_->error("Transition into status \"{}\" not successful, timeout",
                statusWordToString.at(swCondition));
            if (getStatusWord() == StatusWord::Fault) logger_->error("Driver in fault");
            return false;
        }
    }
    return true;
}

crf::expected<bool> CiA402CoEDriver::changeModeOfOperation(
    const ModeOfOperation& mode, const std::chrono::milliseconds& timeout) {
    logger_->debug("changeModeOfOperation({})", modeOfOperationToString.at(mode));
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    while (getModeOfOperation() != mode) {
        *modeOfOperation_ = mode;
        if (getStatusWord() == StatusWord::Fault) {
            logger_->error("Change of mode not successful, driver in fault");
            return crf::Code::CiA402MotorInFault;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout.count()/100));
        end = std::chrono::high_resolution_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (elapsed > timeout) {
            logger_->error("Transition into mode \"{}\" not successful, timeout",
                modeOfOperationToString.at(mode));
            return crf::Code::CiA402ModeOfOperationTimeout;
        }
    }
    return true;
}

crf::expected<bool> CiA402CoEDriver::writeRegisters(const std::vector<RegisterValues>& values) {
    for (auto& value : values) {
        crf::expected<bool> result;
        switch (value.type) {
            case Type::UINT_8:
                result = master_->writeSDO<uint8_t>(id_, value.index, value.subindex,
                    std::any_cast<uint8_t>(value.value));
                if (!result) return result;
                break;
            case Type::UINT_16:
                result = master_->writeSDO<uint16_t>(id_, value.index, value.subindex,
                    std::any_cast<uint16_t>(value.value));
                if (!result) return result;
                break;
            case Type::UINT_32:
                result = master_->writeSDO<uint32_t>(id_, value.index, value.subindex,
                    std::any_cast<uint32_t>(value.value));
                if (!result) return result;
                break;
            case Type::UINT_64:
                result = master_->writeSDO<uint64_t>(id_, value.index, value.subindex,
                    std::any_cast<uint64_t>(value.value));
                if (!result) return result;
                break;
            case Type::INT_8:
                result = master_->writeSDO<int8_t>(id_, value.index, value.subindex,
                    std::any_cast<int8_t>(value.value));
                if (!result) return result;
                break;
            case Type::INT_16:
                result = master_->writeSDO<int16_t>(id_, value.index, value.subindex,
                    std::any_cast<int16_t>(value.value));
                if (!result) return result;
                break;
            case Type::INT_32:
                result = master_->writeSDO<int32_t>(id_, value.index, value.subindex,
                    std::any_cast<int32_t>(value.value));
                if (!result) return result;
                break;
            case Type::INT_64:
                result = master_->writeSDO<int64_t>(id_, value.index, value.subindex,
                    std::any_cast<int64_t>(value.value));
                if (!result) return result;
                break;
            default:
                throw std::invalid_argument("Type of value to write is not supported");
        }
    }
    return true;
}

StatusWord CiA402CoEDriver::decodeStatusWord(uint16_t statusWord) {
    if ((statusWord & StatusWordMask::NotReady) == StatusWord::NotReady) {
        return StatusWord::NotReady;
    }
    if ((statusWord & StatusWordMask::SwitchONDisabled) == StatusWord::SwitchONDisabled) {
        return StatusWord::SwitchONDisabled;
    }
    if ((statusWord & StatusWordMask::Ready) == StatusWord::Ready) {
        return StatusWord::Ready;
    }
    if ((statusWord & StatusWordMask::SwitchedON) == StatusWord::SwitchedON) {
        return StatusWord::SwitchedON;
    }
    if ((statusWord & StatusWordMask::OperationEnabled) == StatusWord::OperationEnabled) {
        return StatusWord::OperationEnabled;
    }
    if ((statusWord & StatusWordMask::Fault) == StatusWord::Fault) {
        return StatusWord::Fault;
    }
    if ((statusWord & StatusWordMask::FaultReactionActive) ==
        StatusWord::FaultReactionActive) {
        return StatusWord::FaultReactionActive;
    }
    if ((statusWord & StatusWordMask::QuickStopActive) == StatusWord::QuickStopActive) {
        return StatusWord::QuickStopActive;
    }
    return StatusWord::NotReady;
}

}  // namespace crf::devices::canopendrivers
