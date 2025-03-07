/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <cstdint>

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_erb_can_driver
 * @brief Enum for error messages that could occur with the ERB motor.
 *
 */
enum ERB415Codes : uint32_t {
    ErrorStatus = 0x8800,
    WarnStatus = 0x8900,
    InfoStatus = 0x8A00,
    ErrorNone = 0x0000,
    InfoNoFreeSpace = 0x0002,
    InfoNoRights = 0x0003,
    InfoUnknownCommand = 0x0004,
    InfoFailed = 0x0005,
    InfoNotReferenced = 0x0006,
    InfoCommunicationError = 0x0009,
    InfoSequenceEnd = 0x000F,
    InfoTimeout = 0x0010,
    InfoUnknownAxisIndex = 0x0011,
    InfoWrongDataType = 0x0012,
    InfoRestart = 0x0013,
    InfoWrongBaudrate = 0x0016,
    InfoChecksum = 0x0019,
    InfoValueLimitMax = 0x001B,
    InfoValueLimitMin = 0x001C,
    InfoMessageLength = 0x001D,
    InfoWrongParameter = 0x001E,
    InfoPositionNotReachable = 0x0022,
    InfoUnknownParameter = 0x0023,
    InfoNMTStopped = 0x0025,
    InfoNMTResetNode = 0x0026,
    InfoNMTPreoperational = 0x0027,
    ErrorBrakeTestFailed = 0x0028,
    ErrorSafeTorqueOff = 0x0029,
    ErrorGearsidePosSystem = 0x002A,
    ErrorPositionBlocked = 0x0058,
    InfoTempHigh = 0x0059,
    ErrorMotorTempLow = 0x006C,
    ErrorMotorTempHigh = 0x006D,
    ErrorTempLowOption = 0x006E,
    ErrorTempHighOption = 0x006F,
    ErrorTempLow = 0x0070,
    ErrorTempHigh = 0x0071,
    ErrorLogicLow = 0x0072,
    ErrorLogicHigh = 0x0073,
    ErrorMotorVoltageLow = 0x0074,
    ErrorMotorVoltageHigh = 0x0075,
    ErrorCableBreak = 0x0076,
    ErrorMotorVoltageShort = 0x0077,
    ErrorPower = 0x0078,
    ErrorLifeSign = 0x007A,
    ErrorCustomDefined = 0x007B,
    ErrorReboot = 0x007C,
    ErrorMotorPhase = 0x007D,
    ErrorPowerTemp = 0x0080,
    ErrorOvershoot = 0x0082,
    ErrorHardwareVersion = 0x0083,
    ErrorSoftwareVersion = 0x0084,
    ErrorWrongDirection = 0x00D1,
    ErrorConfigMemory = 0x00D2,
    ErrorProgramMemory = 0x00D3,
    ErrorInvalidPhase = 0x00D4,
    ErrorSoftLow = 0x00D5,
    ErrorSoftHigh = 0x00D6,
    ErrorService = 0x00D8,
    ErrorFastStop = 0x00D9,
    ErrorTow = 0x00DA,
    ErrorCommunication = 0x00DD,
    ErrorCurrent = 0x00DE,
    ErrorI2T = 0x00DF,
    ErrorInitialize = 0x00E0,
    ErrorInternal = 0x00E1,
    ErrorTooFast = 0x00E4,
    ErrorPosSystem = 0x00E5,
    ErrorFlashMemory = 0x00E7,
    ErrorLimiterActive = 0x00E8,
    ErrorStopCmdTimeout = 0x00E9,
    ErrorUnexpectedStopDuringPhrase = 0x00EA,
    ErrorMath = 0x00EC,
    ErrorLeaveMaster = 0x00ED,
    ErrorCalibCurrent = 0x00EE
};

}  // namespace crf::devices::canopendrivers
