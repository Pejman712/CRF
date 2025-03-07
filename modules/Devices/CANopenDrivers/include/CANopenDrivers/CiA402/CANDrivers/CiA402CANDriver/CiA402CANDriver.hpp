/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <future>
#include <cmath>
#include <chrono>
#include <atomic>
#include <utility>
#include <condition_variable>
#include <mutex>
#include <any>

#include <nlohmann/json.hpp>
#include <lely/coapp/loop_driver.hpp>
#include <lely/coapp/master.hpp>

#include "CANopenDrivers/CiA402/ICiA402Driver.hpp"
#include "CANopenDrivers/CiA301Registers.hpp"
#include "CANopenDrivers/CiA402/CiA402Registers.hpp"
#include "CANopenDrivers/CiA402/CiA402DriverConfiguration.hpp"

#include "crf/expected.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_cia_four_zero_two_can_driver
 * @brief Class that inherits from ICiA402Driver and implements a CANopen compliant version
 * using lely.
 *
 */
class CiA402CANDriver : public lely::canopen::LoopDriver, public ICiA402Driver {
 public:
    /**
    * @brief The CiA402CANDriver inherits the constructors and methods from the LoopDriver class
    * which inherits from the BasicDriver class most of the functions. This class is used
    * to define the driver (master).
    *
    */
    CiA402CANDriver(std::shared_ptr<lely::canopen::AsyncMaster> master, const uint64_t &id, const nlohmann::json& j);  // NOLINT
    ~CiA402CANDriver() override;

    /**
     * @brief Function that initializes the motor (puts it in operational state).
     * The initialization process (going from one state to the other) is configured
     * accordingly with the drive state machine.
     *
     * @param: None
     * @return: True if initialization was successful or already initialized
     * @return: False otherwise
     *
     */
    bool initialize() override;

    /**
     * @brief Function that deinitializes the motor (puts it in switch on disabled state).
     * The deinitialization process (going from one state to the other) is configured
     * accordingly with the drive state machine.
     *
     * @param: None
     * @return: True if deinitialization was successful or already deinitialized
     * @return: False otherwise
     *
     */
    bool deinitialize() override;

    bool inFault() override;
    bool resetFault() override;
    void stop() override;
    bool inQuickStop() override;
    bool quickStop() override;
    bool resetQuickStop() override;

    std::vector<crf::ResponseCode> getMotorStatus() override;
    StatusWord getStatusWord() override;

    crf::expected<double> getPosition() override;
    crf::expected<double> getVelocity() override;
    crf::expected<double> getTorque() override;

    ModeOfOperation getModeOfOperation() override;

    crf::expected<bool> setModeOfOperation(const ModeOfOperation& mode) override;

    crf::expected<bool> setProfilePosition(double pos, double vel, double acc, double dec,
        PositionReference reference = PositionReference::Absolute) override;
    crf::expected<bool> setVelocity(double vel, double deltaSpeedAcc,
        double deltaTimeAcc, double deltaSpeedDec, double delatTimeDec) override;
    crf::expected<bool> setProfileVelocity(double vel, double acc, double dec) override;
    crf::expected<bool> setProfileTorque(double tqe) override;
    crf::expected<bool> setInterpolatedPosition(
        double pos, double vel, double acc, double dec) override;
    crf::expected<bool> setCyclicPosition(
        double pos, double posOffset = 0, double velOffset = 0, double torOffset = 0) override;
    crf::expected<bool> setCyclicVelocity(
        double vel, double velOffset = 0, double torOffset = 0) override;
    crf::expected<bool> setCyclicTorque(double tor, double torOffset = 0) override;

    crf::expected<bool> setMaximumTorque(double torque) override;
    crf::expected<double> getMaximumTorque() override;

 protected:
    /**
     * @brief Function from lely that gets called when the boot-up process of the slave completes.
     * The 'st' parameter contains the last known NMT state of the slave
     * (typically pre-operational), 'es' the error code (0 on success), and 'what'
     * a description of the error, if there is any.
     * @param: st- last known NMT state of the slave
     * @param: es- the error code
     * @param: what-description of the error code
     * @return: no return
     *
     */
    void OnBoot(lely::canopen::NmtState st, char es, const std::string &what) noexcept override;

    /**
     * @brief This function is similar to OnConfig(), meaning it just does the
     * deconfiguration of the master. It accepts the same parameter/s as the OnConfig function
     * @param: res - the function to invoke when the deconfiguration process completes.
     * The argument to res is the result: 0 on success, or an error code on failure.
     * @return: mo return
     *
     */
    void OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override;

    /**
     * @brief Method to wait until the sync message has been sent. This also means that the
     * PDOs, heartbeat, and other messages have been sent
     *
     */
    void waitForPDOExchange();

    /**
     * @brief Function invoked when the SYNC message has been received
     *
     * @param cnt the counter (in the range [1..240]), or 0 if the SYNC message is empty.
     * @param t the time at which the SYNC message was received.
     */
    void OnSync(uint8_t cnt, const time_point& t) noexcept override;


    /**
     * @brief The function invoked when an EMCY message is received from the remote node.
     * @param: eec- emergency error code
     * @param: er - error register
     * @param: msef - manufacturer-specific error code
     *
     */
    void OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept override;

    /**
     * @brief Set up the general registers inside the slave
     *
     * @return crf::expected<bool>
     */
    crf::expected<bool> setUpGeneralRegisters();

    /**
     * @brief Function that sets up the possible modes of operation of the driver.
     * Inside the function the register 6502h is read to see which modes are possible for the driver.
     * The value of the register will have certian bits active which will indicate which modes of
     * operation are available for the driver.
     * @param: json configuration file
     *
     */
    void setUpPossibleModesOfOperation();

    /**
     * @brief This function is used to write values for registers inside a vector.
     * The function uses lely's Wait and AsyncWrite functions to write the values
     * in the registers.
     * @param: values - vector of type RegisterValues (registers written in hexadecimal form)
     * @return: crf::expected<bool>
     *
     */
    crf::expected<bool> writeRegisters(const std::vector<RegisterValues>& values);

    /**
     * @brief This function returns the according statusword for the drive
     * state machine. Possible values of the function are:
     * Not ready,
     * Switch on disabled
     * Ready
     * Switched on
     * Operation enabled
     * Fault
     * Fault reaction active
     * Quick stop active
     * @param: statusWord - input statusword
     * @return: according statusword of the drive state machine
     *
     */
    StatusWord decodeStatusWord(uint16_t statusWord);

    /**
     * @brief Method to write to a register through an SDO message in the network
     *
     * @tparam T Type of payload to write: int32, int16, uint32, ....
     * @param index Index of the register to write to
     * @param subindex Subindex of the register
     * @param value Value to write inside the register
     * @return crf::expected<bool> true if write succeded or error code otherwise
     */
    template<typename T>
    crf::expected<bool> writeSDO(const uint16_t& index, const uint8_t& subindex, T value) {
        std::error_code error;
        Wait(AsyncWrite<T>(index, subindex, std::move(value)), error);
        if (error.value() == 0) return true;
        logger_->warn("Write SDO(idx: 0x{:04X}, subidx: 0x{:02X}) failed, error 0x{:04X}: {}",
            index, subindex, error.value(), error.message());
        return crf::ResponseCode(crf::Code::SDOWriteAbort, error.value());
    }

    /**
     * @brief Method to read from a register through an SDO message in the network
     *
     * @tparam T Type of the payload to read: int32, int16, uint32, ....
     * @param index Index of the register to read to
     * @param subindex Subindex of the register
     * @return crf::expected<T> Expected value of the register
     */
    template<typename T>
    crf::expected<T> readSDO(const uint16_t& index, const uint8_t& subindex) {
        std::error_code error;
        T result = Wait(AsyncRead<T>(index, subindex), error);
        if (error.value() == 0) return result;
        logger_->warn("Read SDO(idx: 0x{:04X}, subidx: 0x{:02X}) failed, error 0x{:04X}: {}",
            index, subindex, error.value(), error.message());
        return crf::ResponseCode(crf::Code::SDOReadAbort, error.value());
    }

    std::shared_ptr<lely::canopen::AsyncMaster> master_;
    CiA402DriverConfiguration config_;
    crf::utility::logger::EventLogger logger_;

    double positionUnit_;
    double velocityUnit_;
    double accelerationUnit_;
    double jerkUnit_;
    double torqueUnit_;
    double currentUnit_;

    bool ppmAvailable_;
    bool vomAvailable_;
    bool pvmAvailable_;
    bool ptmAvailable_;
    bool ipmAvailable_;
    bool cspAvailable_;
    bool csvAvailable_;
    bool cstAvailable_;

    std::shared_ptr<PPM> ppm_;
    std::shared_ptr<VOM> vom_;
    std::shared_ptr<PVM> pvm_;
    std::shared_ptr<PTM> ptm_;
    std::shared_ptr<IPM> ipm_;
    std::shared_ptr<CSP> csp_;
    std::shared_ptr<CSV> csv_;
    std::shared_ptr<CST> cst_;

    std::vector<crf::ResponseCode> emcyError_;

    std::condition_variable syncCV_;
    std::atomic<bool> stopMaxTorqueCheck_;
    std::atomic<double> maxTorque_;
    std::thread maxTorqueThread_;

    const std::chrono::milliseconds syncTimeout_ = std::chrono::milliseconds(50);

    // Function to implement for a timeout - to be later put in setModeOfOperation function
    bool triggerTransition(const StatusWord& sw, const ControlWord& cw,
        const std::chrono::milliseconds& timeout = std::chrono::milliseconds(1000));

    bool waitTransition(const StatusWord& sw,
        const std::chrono::milliseconds& timeout = std::chrono::milliseconds(1000));

    crf::expected<bool> changeModeOfOperation(const ModeOfOperation& mode,
        const std::chrono::milliseconds& timeout = std::chrono::milliseconds(1000));

    const std::chrono::milliseconds resetFaultTime_ = std::chrono::milliseconds(100);
};

}  // namespace crf::devices::canopendrivers
