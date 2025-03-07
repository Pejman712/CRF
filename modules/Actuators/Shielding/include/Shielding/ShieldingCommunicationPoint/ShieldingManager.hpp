/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <thread>
#include <chrono>

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "Shielding/IShielding.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::shielding {

/**
 * @brief The shielding manager class ensures thread safe access to the shielding. It
 *        is also responsible to optimize the resources. If no one is requesting control the
 *        shielding is not deinitialized, since it can be dangerous if other devices are moving.
 *        It also determines which requester has the priority to use it. The shielding must not be
 *        initialized prior to this class.
 */
class ShieldingManager: public crf::utility::devicemanager::DeviceManagerWithPriorityAccess {
 public:
    explicit ShieldingManager(std::shared_ptr<crf::actuators::shielding::IShielding> shielding,
        const std::chrono::milliseconds& initializationTimeout = std::chrono::seconds(60),
        const std::chrono::milliseconds& controlAccessTimeout = std::chrono::seconds(10));
    ShieldingManager(const ShieldingManager& other) = delete;
    ShieldingManager(ShieldingManager&& other) = delete;
    ShieldingManager() = delete;
    ~ShieldingManager() override;
    /**
     * @brief This function is used to open the mechanical shielding.
     * @param priority the priority of the system from 1 to 2^32, being the 1 the number with
     *        highest priority.
     * @return true if the mechanical shielding has been closed correctly.
     * @return false if it failed or if you didn't request the control.
     */
    bool open(const uint32_t &priority);
    /**
     * @brief This function is used to close the mechanical shielding.
     * @param priority the priority of the system from 1 to 2^32, being the 1 the number with
     *        highest priority.
     * @return true if the mechanical shielding has been opened correctly.
     * @return false if it failed or if you didn't request the control.
     */
    bool close(const uint32_t &priority);
    /**
     * @brief Reset the fault state of the shielding motor.
     * @param priority the priority of the system from 1 to 2^32, being the 1 the number with
     *        highest priority.
     * @return True if the reset is succesful.
     * @return False otherwise.
     */
    bool resetFaultState(const uint32_t &priority);
    /**
     * @brief Gets the current status of the shielding, if it is activated or deactivated.
     * @param The priority is used to refresh the timer to be aware on who requested the status for
     *        the last time.
     * @return nlohmann::json with the current status
     */
    nlohmann::json getStatus() override;

 private:
    std::shared_ptr<crf::actuators::shielding::IShielding> shielding_;
    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::shielding
