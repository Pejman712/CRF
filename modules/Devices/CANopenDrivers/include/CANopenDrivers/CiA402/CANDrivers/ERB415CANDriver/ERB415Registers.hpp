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
 * @brief Enum that defines all the 2000s registers which are related
 * to an ERB motor.
 *
 */
enum ERB415 : uint16_t {
    BuildInformation = 0x2005,
    Password = 0x2008,
    User = 0x2009,
    DisconnectReset = 0x200A,

    ControllerCurrentKR = 0x2010,
    ControllerCurrentTN = 0x2011,
    ControllerCurrentTD = 0x2012,
    ControllerCurrentKC = 0x2013,

    ControllerSpeedKR = 0x2020,
    ControllerSpeedTN = 0x2021,
    ControllerSpeedTD = 0x2022,
    ControllerSpeedKC = 0x2023,

    ControllerPositionKR = 0x2030,
    ControllerPositionTN = 0x2031,
    ControllerPositionTD = 0x2032,
    ControllerPositionKC = 0x2033,

    ControllerFeedforwardCurrent = 0x2024,
    ControllerFeedforwardSpeed = 0x2034,
    ControllerFeedforwardSpeedDelay = 0x2035,

    ExtendedStatus = 0x2050,
    BrakeTest = 0x2060,
    DriveParameters = 0x2080,
    VoltageRelated = 0x20A0,
    OperatingData = 0x20B0,
    CommunicationParameter = 0x20C0
};

}  // namespace crf::devices::canopendrivers

