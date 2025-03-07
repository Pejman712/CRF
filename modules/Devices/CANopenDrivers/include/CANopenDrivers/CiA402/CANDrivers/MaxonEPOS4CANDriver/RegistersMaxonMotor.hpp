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
 * @brief Enum class that defines all the 2000s and 3000s registers which are
 * related to a Maxon motor.
 *
 */
enum Maxon : uint16_t {
    NodeId = 0x2000,
    CanBitRate = 0x2001,
    Rs232BitRate = 0x2002,
    Rs232FrameTimeout = 0x2005,
    UsbFrameTimeout = 0x2006,
    CanBitRateDisplay = 0x200A,
    ActiveFieldbus = 0x2010,
    AdditionalIdentitiy = 0x2100,
    Extension1Identity = 0x2101,
    CustomPersistentMemory = 0x210C,
    PowerSupply = 0x2200,

    AxisConfiguration = 0x3000,
    MotorData = 0x3001,
    ElectricalSystemParameters = 0x3002,
    GearConfiguration = 0x3003,
    DigitalIncrementalEncoder1 = 0x3010,
    AnalogIncrementalEncoder = 0x3011,
    SsiAbsoluteEncoder = 0x3012,
    DigitalHallSensor = 0x301A,

    DigitalIncrementalEncoder2 = 0x3020,
    CurrentControlParameterSet = 0x30A0,
    PositionControlParameterSet = 0x30A1,
    VelocityControlParameterSet = 0x30A2,
    VelocityObserverParameterSet = 0x30A3,
    DualLoopPositionControlParameterSet = 0x30AE,

    HomePosition = 0x30B0,
    HomeOffsetMoveDistance = 0x30B1,
    CurrentTresholdForHomingMode = 0x30B2,
    CurrentDemandValue = 0x30D0,
    CurrentActualValues = 0x30D1,
    TorqueActualValues = 0x30D2,
    VelocityActualValues = 0x30D3,
    StandstillWindowConfiguration = 0x30E0,

    DigitalInputProperties = 0x3141,
    ConfigurationOfDigitalInputs = 0x3142,
    DigitalOutputProperties = 0x3150,
    ConfigurationOfDigitalOutputs = 0x3151,
    HoldingBrakeParameters = 0x3158,
    AnalogInputProperties = 0x3160,
    AnalogInputAdjustment = 0x3163,
    AnalogOutputProperties = 0x3180,
    ConfigurationOfAnalogOutputs = 0x3181,
    AnalogOutputGeneralPurpose = 0x3182,
    MotorProtection = 0x3200,
    ThermalControlerProtection = 0x3201
};

}  // namespace crf::devices::canopendrivers
