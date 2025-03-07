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

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/fiber_driver.hpp>

#include "CANopenDrivers/CiA402/CANDrivers/CiA402CANDriver/CiA402CANDriver.hpp"

namespace crf::devices::canopendrivers {

CiA402CANDriver::CiA402CANDriver(
    std::shared_ptr<lely::canopen::AsyncMaster> master,
    const uint64_t& id,
    const nlohmann::json& json) :
    lely::canopen::LoopDriver(*master, id),
    master_(master),
    config_(json),
    logger_("CiA402CANDriver - " + std::to_string(id)),
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

CiA402CANDriver::~CiA402CANDriver() {
    logger_->debug("DTor");
    // Deconfig deinitializes the drivers
}

bool CiA402CANDriver::initialize() {
    logger_->debug("initialize");

    // This might not work with all the motors
    tpdo_mapped[CiA402::ModesOfOperation][Subindex::SUB0] =
        static_cast<uint8_t>(getModeOfOperation());

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

bool CiA402CANDriver::deinitialize() {
    logger_->debug("deinitialize");

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
    return true;
}

bool CiA402CANDriver::inFault() {
    logger_->debug("inFault");
    return getStatusWord() == StatusWord::Fault;
}

bool CiA402CANDriver::resetFault() {
    logger_->debug("resetFault");
    if (getStatusWord() != StatusWord::Fault)  return false;

    // transition 14 on the driver state machine
    tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] =
        static_cast<uint16_t>(ControlWord::FaultResetPrepare);

    waitForPDOExchange();

    // transition 15 on the driver state machine
    tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] =
        static_cast<uint16_t>(ControlWord::FaultResetActive);
    waitForPDOExchange();
    return waitTransition(StatusWord::SwitchONDisabled);
}

bool CiA402CANDriver::quickStop() {
    logger_->debug("quickStop");
    if (getStatusWord() != StatusWord::OperationEnabled) return false;
    // transition 11 on the driver state machine
    if (!triggerTransition(StatusWord::QuickStopActive, ControlWord::QuickStop)) {
        int32_t pos = rpdo_mapped[CiA402::PositionActualValue][Subindex::SUB0];
        tpdo_mapped[CiA402::TargetPosition][Subindex::SUB0] = pos;
        tpdo_mapped[CiA402::TargetVelocity][Subindex::SUB0] = static_cast<int32_t>(0x00000000);
        tpdo_mapped[CiA402::TargetTorque][Subindex::SUB0] = static_cast<int16_t>(0x0000);
        return false;
    }
    int32_t pos = rpdo_mapped[CiA402::PositionActualValue][Subindex::SUB0];
    tpdo_mapped[CiA402::TargetPosition][Subindex::SUB0] = pos;
    tpdo_mapped[CiA402::TargetVelocity][Subindex::SUB0] = static_cast<int32_t>(0x00000000);
    tpdo_mapped[CiA402::TargetTorque][Subindex::SUB0] = static_cast<int16_t>(0x0000);
    return true;
}

void CiA402CANDriver::stop() {
    logger_->debug("stop");
    if (getModeOfOperation() == ModeOfOperation::ProfilePositionMode) {
        tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] =
            static_cast<uint16_t>(ppm_->deactivateWord());
    }
    if (getModeOfOperation() == ModeOfOperation::VelocityMode) {
        tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] =
            static_cast<uint16_t>(vom_->deactivateWord());
        tpdo_mapped[CiA402::TargetVelocity][Subindex::SUB0] = static_cast<int32_t>(0x00000000);
    }
    if (getModeOfOperation() == ModeOfOperation::ProfileVelocityMode) {
        tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] =
            static_cast<uint16_t>(pvm_->deactivateWord());
        tpdo_mapped[CiA402::TargetVelocity][Subindex::SUB0] = static_cast<int32_t>(0x00000000);
    }
    if (getModeOfOperation() == ModeOfOperation::ProfileTorqueMode) {
        tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] =
            static_cast<uint16_t>(ptm_->deactivateWord());
        tpdo_mapped[CiA402::TargetTorque][Subindex::SUB0] = static_cast<int16_t>(0x0000);
    }
    if (getModeOfOperation() == ModeOfOperation::InterpolatedPositionMode) {
        tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] =
            static_cast<uint16_t>(ipm_->deactivateWord());
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

bool CiA402CANDriver::inQuickStop() {
    logger_->debug("inQuickStop");
    return getStatusWord() == StatusWord::QuickStopActive;
}

bool CiA402CANDriver::resetQuickStop() {
    logger_->debug("resetQuickStop");
    if (getStatusWord() != StatusWord::QuickStopActive) return false;
    return triggerTransition(StatusWord::OperationEnabled,
        static_cast<ControlWord>(ControlWord::EnableOperation | ControlWord::Halt));
}

std::vector<crf::ResponseCode> CiA402CANDriver::getMotorStatus() {
    logger_->debug("getMotorStatus");
    std::vector<crf::ResponseCode> codes = emcyError_;
    emcyError_.clear();

    if (inFault()) codes.emplace_back(crf::Code::CiA402MotorInFault);
    if (inQuickStop()) codes.emplace_back(crf::Code::CiA402MotorInQuickStop);

    uint16_t statusWord = rpdo_mapped[CiA402::StatusWord][Subindex::SUB0];

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

StatusWord CiA402CANDriver::getStatusWord() {
    uint16_t statusWord = rpdo_mapped[CiA402::StatusWord][Subindex::SUB0];
    logger_->debug("getStatusWord: 0x{:04X}", statusWord);
    return decodeStatusWord(statusWord);
}

crf::expected<double> CiA402CANDriver::getPosition() {
    int32_t pos = rpdo_mapped[CiA402::PositionActualValue][Subindex::SUB0];
    logger_->debug("getPosition: 0x{:08X}", pos);
    return static_cast<double>(pos / positionUnit_);
}

crf::expected<double> CiA402CANDriver::getVelocity() {
    int16_t vel = rpdo_mapped[(CiA402::VelocityActualValue)][Subindex::SUB0];
    logger_->debug("getVelocity: 0x{:08X}", vel);
    return static_cast<double>(vel / velocityUnit_);
}

crf::expected<double> CiA402CANDriver::getTorque() {
    int16_t tor = rpdo_mapped[CiA402::TorqueActualValue][Subindex::SUB0];
    logger_->debug("getTorque: 0x{:04X}", tor);
    return static_cast<double>(tor / torqueUnit_);
}

ModeOfOperation CiA402CANDriver::getModeOfOperation() {
    int8_t mode = rpdo_mapped[CiA402::ModesOfOperationDisplay][Subindex::SUB0];
    logger_->debug("getModeOfOperation: 0x{:02X}", mode);
    return static_cast<ModeOfOperation>(mode);
}

crf::expected<bool> CiA402CANDriver::setModeOfOperation(const ModeOfOperation& mode) {
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

crf::expected<bool> CiA402CANDriver::setProfilePosition(
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
    tpdo_mapped[CiA402::TargetPosition][Subindex::SUB0] = position;
    waitForPDOExchange();
    tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] =
        static_cast<uint16_t>(ppm_->assumeNewTargetPositonWord());
    waitForPDOExchange();
    return true;
}

crf::expected<bool> CiA402CANDriver::setVelocity(double vel, double deltaSpeedAcc,
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
    tpdo_mapped[CiA402::VLTargetVelocity][Subindex::SUB0] = velocity;
    waitForPDOExchange();
    tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] = static_cast<uint16_t>(vom_->activateWord());
    waitForPDOExchange();
    return true;
}

crf::expected<bool> CiA402CANDriver::setProfileVelocity(double vel, double acc, double dec) {
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
    tpdo_mapped[CiA402::TargetVelocity][Subindex::SUB0] = velocity;
    waitForPDOExchange();
    tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] = static_cast<uint16_t>(pvm_->activateWord());
    waitForPDOExchange();
    return true;
}

crf::expected<bool> CiA402CANDriver::setProfileTorque(double tor) {
    logger_->debug("setProfileTorque({})", tor);
    if (getStatusWord() == StatusWord::Fault) return crf::Code::CiA402MotorInFault;
    if (getStatusWord() != StatusWord::OperationEnabled) return crf::Code::NotInitialized;
    if (!ptmAvailable_) return crf::Code::MethodNotAllowed;
    crf::expected<bool> res = setModeOfOperation(ModeOfOperation::ProfileTorqueMode);
    if (!res) return res;

    int16_t torque = tor * torqueUnit_;

    tpdo_mapped[CiA402::TargetTorque][Subindex::SUB0] = torque;
    waitForPDOExchange();
    tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] = static_cast<uint16_t>(ptm_->activateWord());
    waitForPDOExchange();
    return true;
}

crf::expected<bool> CiA402CANDriver::setInterpolatedPosition(
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
    tpdo_mapped[CiA402::InterpolationDataRecord][Subindex::SUB1] = position;
    waitForPDOExchange();
    tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] = static_cast<uint16_t>(ipm_->setIPM());
    waitForPDOExchange();
    return true;
}

crf::expected<bool> CiA402CANDriver::setCyclicPosition(
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
    tpdo_mapped[CiA402::TargetPosition][Subindex::SUB0] = position;
    return true;
}

crf::expected<bool> CiA402CANDriver::setCyclicVelocity(
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
    tpdo_mapped[CiA402::TargetVelocity][Subindex::SUB0] = velocity;
    return true;
}

crf::expected<bool> CiA402CANDriver::setCyclicTorque(double tor, double torOffset) {
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
    tpdo_mapped[CiA402::TargetTorque][Subindex::SUB0] = torque;
    return true;
}

crf::expected<bool> CiA402CANDriver::setMaximumTorque(double torque) {
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

crf::expected<double> CiA402CANDriver::getMaximumTorque() {
    logger_->debug("getMaximumTorque");
    if (!stopMaxTorqueCheck_) return maxTorque_.load();
    crf::expected<uint16_t> maxTorque = readSDO<uint16_t>(CiA402::MaxTorque, Subindex::SUB0);
    if (!maxTorque) return maxTorque;
    return maxTorque.value() / torqueUnit_;
}

// Private

void CiA402CANDriver::OnBoot(lely::canopen::NmtState st, char es, const std::string& what) noexcept {  // NOLINT
    logger_->debug("OnBoot");
    if (!es || es == 'L') {
        logger_->info("Slave {} booted successfully: {}", static_cast<int>(id()), what);
    } else {
        logger_->error("Slave {} failed booting: {}", static_cast<int>(id()), what);
    }
    uint32_t id = readSDO<uint32_t>(CiA301::IdentityObject, Subindex::SUB1).value();
    logger_->debug("VendorID: 0x{:08X}", id);
}

void CiA402CANDriver::OnDeconfig(std::function<void(std::error_code ec)> res) noexcept {
    logger_->debug("OnDeconfig");
    try {
        if (!deinitialize()) {
            logger_->error("Deinitialization was unsuccessful");
        } else {
            logger_->debug("Deinitialization was successful");
        }
        res({});
    } catch (lely::canopen::SdoError &e) {
        res(e.code());
    }
}

void CiA402CANDriver::waitForPDOExchange() {
    std::mutex mtx;
    std::unique_lock lck(mtx);
    syncCV_.wait_for(lck, syncTimeout_);
}

void CiA402CANDriver::OnSync(uint8_t cnt, const time_point& t) noexcept {
    syncCV_.notify_all();
}

void CiA402CANDriver::OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept {
    logger_->warn("EMCY error code: 0x{:04X}", eec);
    logger_->warn("Error register: 0x{:02X}", er);
    logger_->warn("Manufacturer specific code: 0x{:02X} 0x{:02X} 0x{:02X} 0x{:02X} 0x{:02X}",
        msef[0], msef[1], msef[2], msef[3], msef[4]);
    emcyError_.push_back(crf::ResponseCode(
        crf::Code::CiA301EMCY , static_cast<uint16_t>(msef[0])));
}

crf::expected<bool> CiA402CANDriver::setUpGeneralRegisters() {
    crf::expected<bool> res;
    if (config_.getPolarity()) {
        Polarity pol = config_.getPolarity().value();
        res = writeSDO<uint16_t>(CiA402::Polarity, Subindex::SUB0, pol);
        if (!res) return res;
    }
    if (config_.getMaxMotorSpeed()) {
        uint32_t speed = config_.getMaxMotorSpeed().value();
        res = writeSDO<uint32_t>(CiA402::MaxMotorSpeed, Subindex::SUB0, speed);
        if (!res) return res;
    }
    if (config_.getMaxMotorTorque()) {
        uint16_t tqe = config_.getMaxMotorTorque().value();
        res = writeSDO<uint16_t>(CiA402::MaxTorque, Subindex::SUB0, tqe);
        if (!res) return res;
    }
    if (config_.getMaxMotorCurrent()) {
        uint16_t crnt = config_.getMaxMotorCurrent().value();
        res = writeSDO<uint16_t>(CiA402::MaxCurrent, Subindex::SUB0, crnt);
        if (!res) return res;
    }
    if (config_.getMotorRatedTorque()) {
        uint32_t tqe = config_.getMotorRatedTorque().value();
        res = writeSDO<uint32_t>(CiA402::MotorRatedTorque, Subindex::SUB0, tqe);
        if (!res) return res;
    }
    if (config_.getMotorRatedCurrent()) {
        uint32_t crnt = config_.getMotorRatedCurrent().value();
        res = writeSDO<uint32_t>(CiA402::MotorRatedCurrent, Subindex::SUB0, crnt);
        if (!res) return res;
    }

    // Option codes
    if (config_.getQuickStopOptionCode()) {
        QuickStopOptionCode code = config_.getQuickStopOptionCode().value();
        res = writeSDO<uint32_t>(CiA402::QuickStopOptionCode, Subindex::SUB0, code);
        if (!res) return res;
    }
    if (config_.getShutdownOptionCode()) {
        ShutdownOptionCode code = config_.getShutdownOptionCode().value();
        res = writeSDO<uint32_t>(CiA402::ShutdownOptionCode, Subindex::SUB0, code);
        if (!res) return res;
    }
    if (config_.getDisableOperationOptionCode()) {
        DisableOperationOptionCode code = config_.getDisableOperationOptionCode().value();
        res = writeSDO<uint32_t>(CiA402::DisableOperationOptionCode, Subindex::SUB0, code);
        if (!res) return res;
    }
    if (config_.getHaltOptionCode()) {
        HaltOptionCode code = config_.getHaltOptionCode().value();
        res = writeSDO<uint32_t>(CiA402::HaltOptionCode, Subindex::SUB0, code);
        if (!res) return res;
    }
    if (config_.getFaultOptionCode()) {
        FaultOptionCode code = config_.getFaultOptionCode().value();
        res = writeSDO<uint32_t>(CiA402::FaultReactionOptionCode, Subindex::SUB0, code);
        if (!res) return res;
    }
    return true;
}

void CiA402CANDriver::setUpPossibleModesOfOperation() {
    logger_->debug("setUpPossibleModesOfOperation");
    crf::expected<uint32_t> modesExp;
    if (config_.checkSupportedModes()) {
        modesExp = readSDO<uint32_t>(CiA402::SupportedDriveModes, Subindex::SUB0);
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

bool CiA402CANDriver::triggerTransition(const StatusWord& swCondition, const ControlWord& cwAction,
    const std::chrono::milliseconds& timeout) {
    logger_->debug("triggerTransition");
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    while (getStatusWord() != swCondition) {
        tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] = static_cast<uint16_t>(cwAction);
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

bool CiA402CANDriver::waitTransition(const StatusWord& swCondition,
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

crf::expected<bool> CiA402CANDriver::changeModeOfOperation(
    const ModeOfOperation& mode, const std::chrono::milliseconds& timeout) {
    logger_->debug("changeModeOfOperation({})", modeOfOperationToString.at(mode));
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    while (getModeOfOperation() != mode) {
        tpdo_mapped[CiA402::ModesOfOperation][Subindex::SUB0] = static_cast<int8_t>(mode);
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

crf::expected<bool> CiA402CANDriver::writeRegisters(const std::vector<RegisterValues>& values) {
    for (auto& value : values) {
        crf::expected<bool> result;
        switch (value.type) {
            case Type::UINT_8:
                result = writeSDO<uint8_t>(static_cast<CiA402>(value.index),
                    static_cast<Subindex>(value.subindex), std::any_cast<uint8_t>(value.value));
                if (!result) return result;
                break;
            case Type::UINT_16:
                result = writeSDO<uint16_t>(static_cast<CiA402>(value.index),
                    static_cast<Subindex>(value.subindex), std::any_cast<uint16_t>(value.value));
                if (!result) return result;
                break;
            case Type::UINT_32:
                result = writeSDO<uint32_t>(static_cast<CiA402>(value.index),
                    static_cast<Subindex>(value.subindex), std::any_cast<uint32_t>(value.value));
                if (!result) return result;
                break;
            case Type::UINT_64:
                result = writeSDO<uint64_t>(static_cast<CiA402>(value.index),
                    static_cast<Subindex>(value.subindex), std::any_cast<uint64_t>(value.value));
                if (!result) return result;
                break;
            case Type::INT_8:
                result = writeSDO<int8_t>(static_cast<CiA402>(value.index),
                    static_cast<Subindex>(value.subindex), std::any_cast<int8_t>(value.value));
                if (!result) return result;
                break;
            case Type::INT_16:
                result = writeSDO<int16_t>(static_cast<CiA402>(value.index),
                    static_cast<Subindex>(value.subindex), std::any_cast<int16_t>(value.value));
                if (!result) return result;
                break;
            case Type::INT_32:
                result = writeSDO<int32_t>(static_cast<CiA402>(value.index),
                    static_cast<Subindex>(value.subindex), std::any_cast<int32_t>(value.value));
                if (!result) return result;
                break;
            case Type::INT_64:
                result = writeSDO<int64_t>(static_cast<CiA402>(value.index),
                    static_cast<Subindex>(value.subindex), std::any_cast<int64_t>(value.value));
                if (!result) return result;
                break;
            default:
                throw std::invalid_argument("Type of value to write is not supported");
        }
    }
    return true;
}

StatusWord CiA402CANDriver::decodeStatusWord(uint16_t statusWord) {
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
