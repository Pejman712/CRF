/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <vector>
#include <string>
#include <optional>
#include <cstdint>
#include <map>

#include <nlohmann/json.hpp>

#include "CANopenDrivers/CiA402/CiA402Definitions.hpp"

#include "EventLogger/EventLogger.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_cia_four_zero_two
 * @brief Class to parse and provide the values for the configuration
 * of the driver.
 *
 */
class CiA402DriverConfiguration {
 public:
    CiA402DriverConfiguration() = delete;
    explicit CiA402DriverConfiguration(const nlohmann::json& json);
    explicit CiA402DriverConfiguration(const std::string&) = delete;
    virtual ~CiA402DriverConfiguration() = default;

    /**
     * @brief Get the Slave ID
     *
     * @return uint64_t ID
     */
    uint64_t getSlaveID() const;

    /**
     * @brief See if supported modes should be checked
     *
     * @return true if the supported modes should be checked
     * @return false if the supported modes should not be checked
     */
    bool checkSupportedModes() const;

    /**
     * @brief Get the Position Unit Conversion
     *
     * @return double conversion factor
     */
    double getPositionUnitConversion() const;

    /**
     * @brief Get the Velocity Unit Conversion
     *
     * @return double conversion factor
     */
    double getVelocityUnitConversion() const;

    /**
     * @brief Get the Acceleration Unit Conversion
     *
     * @return double conversion factor
     */
    double getAccelerationUnitConversion() const;

    /**
     * @brief Get the Jerk Unit Conversion
     *
     * @return double conversion factor
     */
    double getJerkUnitConversion() const;

    /**
     * @brief Get the Torque Unit Conversion
     *
     * @return double conversion factor
     */
    double getTorqueUnitConversion() const;

    /**
     * @brief Get the Current Unit Conversion
     *
     * @return double conversion factor
     */
    double getCurrentUnitConversion() const;

    /**
     * @brief Get the Polarity of the motor
     *
     * @return std::optional<Polarity> polarity
     */
    std::optional<Polarity> getPolarity() const;

    /**
     * @brief Get the Max Motor Speed of the motor
     *
     * @return std::optional<uint32_t> max motor speed
     */
    std::optional<uint32_t> getMaxMotorSpeed() const;

    /**
     * @brief Get the Max Motor Torque of the motor
     *
     * @return std::optional<uint16_t> max torque
     */
    std::optional<uint16_t> getMaxMotorTorque() const;

    /**
     * @brief Get the Max Motor Current of the motor
     *
     * @return std::optional<uint16_t> max current
     */
    std::optional<uint16_t> getMaxMotorCurrent() const;

    /**
     * @brief Get the Motor Rated Torque of the motor
     *
     * @return std::optional<uint32_t> rated torque
     */
    std::optional<uint32_t> getMotorRatedTorque() const;

    /**
     * @brief Get the Motor Rated Current of the motor
     *
     * @return std::optional<uint32_t> rated current
     */
    std::optional<uint32_t> getMotorRatedCurrent() const;

    /**
     * @brief Get the Quick Stop Option Code of the motor
     *
     * @return std::optional<QuickStopOptionCode>
     */
    std::optional<QuickStopOptionCode> getQuickStopOptionCode() const;

    /**
     * @brief Get the Shutdown Option Code of the motor
     *
     * @return std::optional<ShutdownOptionCode>
     */
    std::optional<ShutdownOptionCode> getShutdownOptionCode() const;

    /**
     * @brief Get the Disable Operation Option Code of the motor
     *
     * @return std::optional<DisableOperationOptionCode>
     */
    std::optional<DisableOperationOptionCode> getDisableOperationOptionCode() const;

    /**
     * @brief Get the Halt Option Code of the motor
     *
     * @return std::optional<HaltOptionCode>
     */
    std::optional<HaltOptionCode> getHaltOptionCode() const;

    /**
     * @brief Get the Fault Option Code of the motor
     *
     * @return std::optional<FaultOptionCode>
     */
    std::optional<FaultOptionCode> getFaultOptionCode() const;

    /**
     * @brief Get the Profile Position Config
     *
     * @return nlohmann::json
     */
    nlohmann::json getProfilePositionConfig() const;

    /**
     * @brief Get the Profile Velocity Config
     *
     * @return nlohmann::json
     */
    nlohmann::json getProfileVelocityConfig() const;

    /**
     * @brief Get the Profile Torque Config
     *
     * @return nlohmann::json
     */
    nlohmann::json getProfileTorqueConfig() const;

    /**
     * @brief Get the Velocity Mode Config
     *
     * @return nlohmann::json
     */
    nlohmann::json getVelocityModeConfig() const;

    /**
     * @brief Get the Interpolated Position Config
     *
     * @return nlohmann::json
     */
    nlohmann::json getInterpolatedPositionConfig() const;

    /**
     * @brief Get the Homing Config
     *
     * @return nlohmann::json
     */
    nlohmann::json getHomingConfig() const;

    /**
     * @brief Get the Cyclic Position Config
     *
     * @return nlohmann::json
     */
    nlohmann::json getCyclicPositionConfig() const;

    /**
     * @brief Get the Cyclic Velocity Config
     *
     * @return nlohmann::json
     */
    nlohmann::json getCyclicVelocityConfig() const;

    /**
     * @brief Get the Cyclic Torque Config
     *
     * @return nlohmann::json
     */
    nlohmann::json getCyclicTorqueConfig() const;

 private:
    /**
     * @brief Parse the JSON object into the variables
     *
     * @param json config of the robot
     */
    void parse(const nlohmann::json& json);
    void parse(const std::string&) = delete;

    uint64_t slaveID_;
    bool checkSupportedModes_;
    double posUnits_;
    double velUnits_;
    double accUnits_;
    double jrkUnits_;
    double tqeUnits_;
    double crnUnits_;

    std::optional<Polarity> polarity_;
    std::optional<uint32_t> maxMotorSpeed_;
    std::optional<uint16_t> maxMotorTorque_;
    std::optional<uint16_t> maxMotorCurrent_;
    std::optional<uint32_t> motorRatedTorque_;
    std::optional<uint32_t> motorRatedCurrent_;
    std::optional<QuickStopOptionCode> quickStopCode_;
    std::optional<ShutdownOptionCode> shutdownCode_;
    std::optional<DisableOperationOptionCode> disableOperationCode_;
    std::optional<HaltOptionCode> haltCode_;
    std::optional<FaultOptionCode> faultCode_;
    nlohmann::json ppmConfig_;
    nlohmann::json pvmConfig_;
    nlohmann::json ptmConfig_;
    nlohmann::json vomConfig_;
    nlohmann::json ipmConfig_;
    nlohmann::json homConfig_;
    nlohmann::json cspConfig_;
    nlohmann::json csvConfig_;
    nlohmann::json cstConfig_;

    crf::utility::logger::EventLogger logger_;

    const double ampsToMilliAmps = 1000;
    const double newtmToMilliNewtm = 1000;
    const double rpmToRadPerSec_ = 9.5493;

    const std::map<std::string, Polarity> polarityMap_ {
        {"NoInversion", Polarity::NoInversion},
        {"GlobalInversion", Polarity::GlobalInversion},
        {"PositionInversion", Polarity::PositionInversion},
        {"VelocityInversion", Polarity::VelocityInversion}
    };

    const std::map<std::string, QuickStopOptionCode> quickStopCodeMap_ = {
        {"SlowDownRampAndSwitchOnDisable", QuickStopOptionCode::SlowDownRampSOD},
        {"QuickStopRampAndSwitchOnDisable", QuickStopOptionCode::QuickStopRampSOD},
        {"CurrentLimitAndSwitchOnDisable", QuickStopOptionCode::CurrentLimitSOD},
        {"VoltageLimitAndSwitchOnDisable", QuickStopOptionCode::VoltageLimitSOD},
        {"SlowDownRampAndQuickStopActive", QuickStopOptionCode::SlowDownRampQSA},
        {"QuickStopRampAndQuickStopActive", QuickStopOptionCode::QuickStopRampQSA},
        {"CurrentLimitAndQuickStopActive", QuickStopOptionCode::CurrentLimitQSA},
        {"VoltageLimitAndQuickStopActive", QuickStopOptionCode::VoltageLimitQSA}
    };

    const std::map<std::string, ShutdownOptionCode> shutdownCodeMap_ = {
        {"Disabled", ShutdownOptionCode::Disabled},
        {"SlowDownRamp", ShutdownOptionCode::SlowDownRamp}
    };

    const std::map<std::string, DisableOperationOptionCode> disableOperationCodeMap_ = {
        {"Disabled", DisableOperationOptionCode::Disabled},
        {"SlowDownRamp", DisableOperationOptionCode::SlowDownRamp}
    };

    const std::map<std::string, HaltOptionCode> haltCodeMap_ = {
        {"SlowDownRamp", HaltOptionCode::SlowDownRamp},
        {"QuickStopRamp", HaltOptionCode::QuickStopRamp},
        {"CurentLimit", HaltOptionCode::CurrentLimit},
        {"VoltageLimit", HaltOptionCode::VoltageLimit}
    };

    const std::map<std::string, FaultOptionCode> faultCodeMap_ = {
        {"SlowDownRamp", FaultOptionCode::SlowDownRamp},
        {"QuickStopRamp", FaultOptionCode::QuickStopRamp},
        {"CurentLimit", FaultOptionCode::CurrentLimit},
        {"VoltageLimit", FaultOptionCode::VoltageLimit}
    };
};

}  // namespace crf::devices::canopendrivers
