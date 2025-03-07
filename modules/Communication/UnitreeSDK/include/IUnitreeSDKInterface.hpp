/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Lucas BRAUD CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include "unitree_legged_sdk/unitree_legged_sdk.h"

namespace crf::communication::unitreesdk {

/**
* @ingroup group_iunitreesdkinterface
*/
class IUnitreeSDKInterface {
 public:
    virtual ~IUnitreeSDKInterface() = default;
     /**
     * @brief Init udp and safety hardware communication layers (based on robot config file)
     * @return True when init successfull, False when failed.
     */
    virtual bool init(uint8_t level, uint16_t localPort, const char* targetIP, uint16_t targetPort,
        UNITREE_LEGGED_SDK::LeggedType type) = 0; // NOLINT
    /**
     * @brief Enable low level command writing
     */
    virtual void UDPInitCmdData(UNITREE_LEGGED_SDK::LowCmd& cmd) = 0; // NOLINT
    /**
     * @brief Enable data (state) reading from sensors through UDP
     */
    virtual void UDPRecv() = 0;
    /**
     * @brief Enable data (cmd) writing to motors through UDP
     */
    virtual void UDPSend() = 0;
    /**
     * @brief Set safety position limit for motors
     */
    virtual void SafetyPositionLimit(UNITREE_LEGGED_SDK::LowCmd& cmd) = 0; // NOLINT
    /**
     * @brief Set safety power limit for motors
     */
    virtual int SafetyPowerProtect(UNITREE_LEGGED_SDK::LowCmd& cmd, // NOLINT
        UNITREE_LEGGED_SDK::LowState& state, int num) = 0; // NOLINT
    /**
     * @brief Send low level commands to the motors
     */
    virtual int UDPSetSend(UNITREE_LEGGED_SDK::LowCmd& cmd) = 0; // NOLINT
    /**
     * @brief Receive robot's state
     */
    virtual void UDPGetRecv(UNITREE_LEGGED_SDK::LowState& state) = 0; // NOLINT
};

}  // namespace crf::communication::unitreesdk
