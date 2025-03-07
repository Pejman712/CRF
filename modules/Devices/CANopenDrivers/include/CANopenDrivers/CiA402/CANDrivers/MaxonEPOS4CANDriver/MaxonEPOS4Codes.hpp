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
 * @ingroup group_maxon_epos_four_can_driver
 * @brief Enum class from EMCY error codes specifically for Maxon motor.
 *
 */
enum MaxonEPOS4Codes : uint16_t {
    NoErrorOrReset = 0x0000,
    GenericError = 0x1000,
    GenericInitializationError0 = 0x1080,
    GenericInitializationError1 = 0x1081,
    GenericInitializationError2 = 0x1082,
    GenericInitializationError3 = 0x1083,

    FirmwareIncompatibilityError = 0x1090,
    OvercurrentError = 0X2310,
    PowerStageProtectionError = 0X2320,
    OvervoltageError = 0x3210,
    UndervoltageError = 0X3220,
    ThermalOverloadError = 0x04210,
    ThermalMotorOverloadError = 0X4380,
    LogicSupplyVoltageTooLow = 0X5113,
    HardwareDefectError = 0X5280,
    HardwareIncompatibilityError = 0X5281,

    HardwareError0 = 0X5480,
    HardwareError1 = 0X5481,
    HardwareError2 = 0X5482,
    HardwareError3 = 0X5483,

    SignOfLifeError = 0x6080,
    Extensition1WatchdogError = 0x6081,
    InternalSoftwareError0 = 0x6180,
    InternalSoftwareError1 = 0x6181,
    InternalSoftwareError2 = 0x6182,
    InternalSoftwareError3 = 0x6183,
    InternalSoftwareError4 = 0x6184,
    InternalSoftwareError5 = 0x6185,
    InternalSoftwareError6 = 0x6186,
    InternalSoftwareError7 = 0x6187,
    InternalSoftwareError8 = 0x6188,
    InternalSoftwareError9 = 0x6189,
    InternalSoftwareErrorA = 0x618A,
    InternalSoftwareErrorB = 0x618B,
    InternalSoftwareErrorC = 0x618C,
    InternalSoftwareErrorD = 0x618D,
    InternalSoftwareErrorE = 0x618E,
    InternalSoftwareErrorF = 0x618F,
    InternalSoftwareError16 = 0x6190,

    SoftwareParamteterError = 0x6320,
    PersistentParameterCorruptError = 0x6380,
    PositionSensorError = 0X7320,
    PositionSensorBreachError = 0X7380,
    PositionSensorResolutionError = 0X7381,
    PositionSensorIndexError = 0X7382,
    HalllSensorError = 0X7388,
    HallSensorNotFoundError = 0X7389,
    HallAngleDetectionError = 0X738A,
    SSISensorError = 0X738C,
    MissingMainSensorError = 0X7390,
    MissingCommunicationSensorError = 0X7391,
    MainSensorDirectionError = 0X7392,

    // Communication errors

    CANOverrunObjectLost = 0x8110,
    CANOverrun = 0x8111,
    CANDriverInPassiveMode = 0x8120,
    CANHeartbeatError = 0X8130,
    CANNodeMissedDataFrames = 0x8140,
    CANPDOCOBIDCollision = 0x8150,
    CANBusTurnedOff = 0X81FD,
    CANRPDOQueueOverflow = 0X81FE,
    CANTPDOQueueOverflow = 0X81FF,
    PDOLengthMismatched = 0x8210,
    PDOLengthExceeded = 0x8220,
    UnexpectedSyncMessageLength = 0x8240,
    RPDOTimeout = 0x8250,
    FollowingError = 0X8611,
    NegativeLimitSwitchError = 0X8A80,
    PositiveLimitSwitchError = 0X8A81,
    SoftwarePositionLimitError = 0X8A82,
    STOError = 0X8A88,
    SystemOverloadedError = 0XFF01,
    WatchdogError = 0XFF02,
    SystemPeakOverloadedError = 0XFF0B,
    ControllerGainError = 0XFF10,

    AutoTuningCurrentLimitError = 0XFF12,
    AutoTuningIdentificationCurrentError = 0XFF13,
    AutoTuningDataSamplingError = 0XFF14,
    AutoTuningSampleMismatchError = 0XFF15,
    AutoTuningParameterError = 0XFF16,
    AutoTuningAmplitudeMismatchError = 0XFF17,
    AutoTuningPeriodLengthError = 0XFF18,
    AutoTuningTimeoutError = 0XFF19,
    AutoTuningStandstillError = 0XFF20,
    AutoTuningTorqueInvalidError = 0XFF21,
};

}  // namespace crf::devices::canopendrivers
