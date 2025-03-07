/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2023
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

#include "EtherCATDrivers/BasicEtherCATDriver/BasicEtherCATDriver.hpp"
#include "CANopenDrivers/CiA402/ICiA402Driver.hpp"
#include "CANopenDrivers/CoEMaster/CoEMaster.hpp"
#include "CANopenDrivers/CiA301Registers.hpp"
#include "CANopenDrivers/CiA402/CiA402Registers.hpp"
#include "CANopenDrivers/CiA402/CiA402DriverConfiguration.hpp"

#include "crf/expected.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_cia_four_zero_two_coe_driver
 * @brief Class that inherits from ICiA402Driver and implements a CANopen over EtherCAT
 * compliant version using SOEM.
 *
 */
class CiA402CoEDriver : public ethercatdrivers::BasicEtherCATDriver, public ICiA402Driver {
 public:
    CiA402CoEDriver(std::shared_ptr<CoEMaster> master, const uint64_t &id, const nlohmann::json& j);  // NOLINT
    ~CiA402CoEDriver() override;

    bool initialize() override;
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
    // Default TxPDO pointers
    uint16_t* controlWord_;
    int8_t* modeOfOperation_;
    int32_t* targetPosition_;
    int32_t* targetVelocity_;
    int16_t* targetTorque_;
    uint32_t* interpolatedPosition_;

    // Default RxPDO pointers
    uint16_t* statusWord_;
    int8_t* modeOfOperationDisplay_;
    int32_t* positionActualValue_;
    int32_t* velocityActualValue_;
    int16_t* torqueActualValue_;
    int16_t* currentActualValue_;

    bool bindIOMap() override;

    /**
     * @brief Checks if the remote bit is active. If it's not active the communication
     * won't work with CANopen.
     *
     */
    void checkRemoteBit();

    /**
     * @brief Set up all the general registers of the slave.
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
     * @return: none
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
     */
    StatusWord decodeStatusWord(uint16_t statusWord);

    std::shared_ptr<CoEMaster> master_;
    uint16_t id_;
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

    std::atomic<bool> stopMaxTorqueCheck_;
    std::atomic<double> maxTorque_;
    std::thread maxTorqueThread_;

    // Function to implement for a timeout - to be later put in setModeOfOperation function
    bool triggerTransition(const StatusWord& sw, const ControlWord& cw,
        const std::chrono::milliseconds& timeout = std::chrono::milliseconds(3000));

    bool waitTransition(const StatusWord& sw,
        const std::chrono::milliseconds& timeout = std::chrono::milliseconds(3000));

    crf::expected<bool> changeModeOfOperation(const ModeOfOperation& mode,
        const std::chrono::milliseconds& timeout = std::chrono::milliseconds(3000));

 private:
#pragma pack(push, 1)
    struct TxPDO {
        uint16_t controlWord;
        int8_t modeOfOperation;
        uint8_t placeholder;
        int32_t targetPosition;
        int32_t targetVelocity;
        int16_t targetTorque;
    };
#pragma pack(pop)

#pragma pack(push, 1)
    struct RxPDO {
        uint16_t statusWord;
        int8_t modeOfOperationDisplay;
        uint8_t placeholder;
        int32_t positionActualValue;
        int32_t velocityActualValue;
        int16_t torqueActualValue;
        int16_t currentActualValue;
    };
#pragma pack(pop)
};

}  // namespace crf::devices::canopendrivers
