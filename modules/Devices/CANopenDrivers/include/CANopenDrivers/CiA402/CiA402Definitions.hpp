/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_cia_four_zero_two
 * @brief Enum class that explains the possible values that can be set for the
 * control word.
 *
 */
struct ControlWords {
    enum Type : uint16_t {
        Shutdown = 0x0006,
        SwitchON = 0x0007,
        DisableVoltage = 0x0000,
        QuickStop = 0x0002,
        DisableOperation = 0x0007,
        EnableOperation = 0x000F,
        FaultResetPrepare = 0x0000,
        FaultResetActive = 0x0080,
        Halt = 0x0100,
    };
};

using ControlWord = ControlWords::Type;

/**
 * @ingroup group_cia_four_zero_two
 * @brief Enum class with the masks for the control word
 *
 */
struct ControlWordMasks {
    enum Type: uint16_t {
        Shutdown = 0x008F,
        SwitchON = 0x008F,
        DisableVoltage = 0x008F,
        QuickStop = 0x008F,
        DisableOperation = 0x008F,
        EnableOperation = 0x008F,
        FaultResetPrepare = 0x0080,
        FaultResetActive = 0x0080
    };
};

using ControlWordMask = ControlWordMasks::Type;

/**
 * @ingroup group_cia_four_zero_two
 * @brief Enum class that explains the possible values of the status word.
 *
 */
struct StatusWords {
    enum Type : uint16_t {
        NotReady = 0x0000,
        SwitchONDisabled = 0x0040,
        Ready = 0x0021,
        SwitchedON = 0x0023,
        OperationEnabled = 0x0027,
        Fault = 0x0008,
        FaultReactionActive = 0x000F,
        QuickStopActive = 0x0007
    };
};

using StatusWord = StatusWords::Type;

/**
 * @ingroup group_cia_four_zero_two
 * @brief Map to convert the Status Word into a string
 *
 */
const std::map<StatusWord, std::string> statusWordToString = {
    {StatusWord::NotReady, "Not Ready"},
    {StatusWord::SwitchONDisabled, "Switch ON Disabled"},
    {StatusWord::Ready, "Ready"},
    {StatusWord::SwitchedON, "Switched ON"},
    {StatusWord::OperationEnabled, "Operation Enabled"},
    {StatusWord::Fault, "Fault"},
    {StatusWord::FaultReactionActive, "Fault Reaction Active"},
    {StatusWord::QuickStopActive, "Quick Stop Active"}
};

/**
 * @ingroup group_cia_four_zero_two
 * @brief Enum class of masks for the status word.
 *
 */
struct StatusWordMasks {
    enum Type : uint16_t {
        NotReady = 0x004F,
        SwitchONDisabled = 0x004F,
        Ready = 0x006F,
        SwitchedON = 0x006F,
        OperationEnabled = 0x006F,
        Fault = 0x004F,
        FaultReactionActive = 0x004F,
        QuickStopActive = 0x006F
    };
};

using StatusWordMask = StatusWordMasks::Type;

/**
 * @ingroup group_cia_four_zero_two
 * @brief Enum class that explains the possible modes of operation.
 *
 */
struct ModeOfOperationValues {
    enum Type : int8_t {
        NoOperatingMode = 0x00,
        ProfilePositionMode = 0x01,
        VelocityMode = 0x02,
        ProfileVelocityMode = 0x03,
        ProfileTorqueMode = 0x04,
        HomingMode = 0x06,
        InterpolatedPositionMode = 0x07,
        CyclicSyncPositionMode = 0x08,
        CyclicSyncVelocityMode = 0x09,
        CyclicSyncTorqueMode = 0x0A
    };
};

using ModeOfOperation = ModeOfOperationValues::Type;

/**
 * @ingroup group_cia_four_zero_two
 * @brief Map to convert the Mode of Operation into a string
 *
 */
const std::map<ModeOfOperation, std::string> modeOfOperationToString = {
    {ModeOfOperation::NoOperatingMode, "No Operating Mode"},
    {ModeOfOperation::ProfilePositionMode, "Profile Position Mode"},
    {ModeOfOperation::VelocityMode, "Velocity Mode"},
    {ModeOfOperation::ProfileVelocityMode, "Profile Velocity Mode"},
    {ModeOfOperation::ProfileTorqueMode, "Profile Torque Mode"},
    {ModeOfOperation::HomingMode, "Homing Mode"},
    {ModeOfOperation::InterpolatedPositionMode, "Interpolated Position Mode"},
    {ModeOfOperation::CyclicSyncPositionMode, "Cyclic Sync Position Mode"},
    {ModeOfOperation::CyclicSyncVelocityMode, "Cyclic Sync Velocity Mode"},
    {ModeOfOperation::CyclicSyncTorqueMode, "Cyclic Sync Torque Mode"}
};

/**
 * @ingroup group_cia_four_zero_two
 * @brief Enum class with the modes inside the supported modes register
 *
 */
struct SupportedModesOptions {
    enum Type : uint32_t {
        PPM = 0X00000001,
        VOM = 0x00000002,
        PVM = 0x00000004,
        PTM = 0x00000008,
        HOM = 0x00000020,
        IPM = 0x00000040,
        CSP = 0x00000080,
        CSV = 0x00000100,
        CST = 0x00000200
    };
};

using SupportedModes = SupportedModesOptions::Type;

/**
 * @ingroup group_cia_four_zero_two
 * @brief Enum class with the masks for the modes of operation
 *
 */
struct SupportedModesMasks {
    enum Type : uint32_t {
        PPM = 0x00000001,
        VOM = 0x00000002,
        PVM = 0x00000004,
        PTM = 0x00000008,
        HOM = 0x00000020,
        IPM = 0x00000040,
        CSP = 0x00000080,
        CSV = 0x00000100,
        CST = 0x00000200
    };
};

using SupportedModesMask = SupportedModesMasks::Type;

/**
 * @ingroup group_csp
 * @brief Enum values for inverting the direction of the movement of the motor.
 * If the motor is a right tuning rotary motor, the movement is in positive direction,
 * while for a left tuning rotary motor the movement is in negative direction. By setting
 * the polarity register with the according values of the enum PolarityCSP, the movement of
 * each type of motor will be inverted.(positive for left tuning rotary motor, negative for right
 * tuning rotary motor). The first enum value can only be set in the register for PVM or CSV mode,
 * while the second enum value can only be set in the PPM, IPM or CSP mode of the motor.
 * Setting the last value of the enum in the register will invert the movement of the
 * motor for both position and velocity modes.
 *
 */
struct PolarityValues {
    enum Type : uint16_t {
        NoInversion = 0x00,
        GlobalInversion = 0x40,
        VelocityInversion = 0x80,
        PositionInversion = 0x01
    };
};

using Polarity = PolarityValues::Type;

/**
 * @ingroup group_cia_four_zero_two
 * @brief 0x605A
 *
 */
struct QuickStopOptionCodes {
    enum Type : int16_t {
        Disabled = 0x0000,
        SlowDownRampSOD = 0x0001,
        QuickStopRampSOD = 0x0002,
        CurrentLimitSOD = 0x0003,
        VoltageLimitSOD = 0x0004,
        SlowDownRampQSA = 0x0005,
        QuickStopRampQSA = 0x0006,
        CurrentLimitQSA = 0x0007,
        VoltageLimitQSA = 0x0008
    };
};

using QuickStopOptionCode = QuickStopOptionCodes::Type;

/**
 * @ingroup group_cia_four_zero_two
 * @brief 0x605B
 *
 */
struct ShutdownOptionCodes {
    enum Type : int16_t {
        Disabled = 0x0000,
        SlowDownRamp = 0x0001
    };
};

using ShutdownOptionCode = ShutdownOptionCodes::Type;

/**
 * @ingroup group_cia_four_zero_two
 * @brief 0x605C
 *
 */
struct DisableOperationOptionCodes {
    enum Type : int16_t {
        Disabled = 0x0000,
        SlowDownRamp = 0x0001
    };
};

using DisableOperationOptionCode = DisableOperationOptionCodes::Type;

/**
 * @ingroup group_cia_four_zero_two
 * @brief 0x605D
 *
 */
struct HaltOptionCodes {
    enum Type : int16_t {
        SlowDownRamp = 0x0001,
        QuickStopRamp = 0x0002,
        CurrentLimit = 0x0003,
        VoltageLimit = 0x0004
    };
};

using HaltOptionCode = HaltOptionCodes::Type;

/**
 * @ingroup group_cia_four_zero_two
 * @brief 0x605E
 *
 */
struct FaultOptionCodes {
    enum Type : int16_t {
        SlowDownRamp = 0x0001,
        QuickStopRamp = 0x0002,
        CurrentLimit = 0x0003,
        VoltageLimit = 0x0004
    };
};

using FaultOptionCode = FaultOptionCodes::Type;

/**
 * @ingroup group_csp
 * @brief Struct for position range limit, since it has 2 parameters for setting the limits of
 * the target position : minimum and maximum position. This register is used to limit the
 * input value of the position (in the trajectory generator for the CSP mode).
 * If the user wants to prevent the use of current set limits
 * of this register, they can set the limits inside the software
 * position limit register. Disabling the limits of the motor inside
 * the resgister is done by setting the subindicies for min and max to zero.
 *
 */
struct PositionRangeLimit {
    int32_t min;
    int32_t max;
};

/**
 * @ingroup group_csp
 * @brief Struct for software position limit, since it has 2 parameters for setting the limits
 * for the position demand value and the position actual value : minimum and maximum position.
 * Disabling the limits of the motor inside the resgister is done by setting the
 * subindicies for min and max to zero.
 *
 */
struct SoftwarePositionLimit {
    int32_t min;
    int32_t max;
};

/**
 * @ingroup group_csp
 * @brief Enum of profile types for the
 * cyclic synchronous position mode.
 *
 */
struct ProfileTypeValues {
    enum Type : int16_t {
        Trapezoidal = 0x0000,
        Sin2 = 0x0001,
        JerkFree = 0x0002,
        JerkLimited = 0x0003
    };
};

using ProfileType = ProfileTypeValues::Type;

/**
 * @ingroup group_csp
 * @brief Struct for configuring the interpolation time period.
 * It requiers the value of the time between 2 PDOs and the unit
 * of the period (default is milliseconds).
 *
 */
struct InterpolationTimePeriod {
    uint8_t value;
    int8_t index;
};

}  // namespace crf::devices::canopendrivers
