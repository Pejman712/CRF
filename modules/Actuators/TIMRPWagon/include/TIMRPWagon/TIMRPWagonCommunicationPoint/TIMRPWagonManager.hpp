/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <optional>
#include <map>
#include <tuple>
#include <chrono>
#include <thread>

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "TIMRPWagon/ITIMRPWagon.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::timrpwagon {

class TIMRPWagonManager: public crf::utility::devicemanager::DeviceManagerWithPriorityAccess {
 public:
    explicit TIMRPWagonManager(std::shared_ptr<crf::actuators::timrpwagon::ITIMRPWagon> timRPWagon,
        const std::chrono::milliseconds& inizializationTimeout = std::chrono::seconds(60),
        const std::chrono::milliseconds& controlAccessTimeout = std::chrono::seconds(15));
    TIMRPWagonManager(const TIMRPWagonManager&) = delete;
    TIMRPWagonManager(TIMRPWagonManager&&) = delete;
    TIMRPWagonManager() = delete;
    ~TIMRPWagonManager() override;

    /**
     * @brief It triggers the automatic sequence to bring the arm to its retracted position. It has
     *        to be set to true once.
     * @param The control priority level.
     * @return True if it manage to retract the arm.
     * @return False if the retract sequence failes failed.
     * @return std::nullopt if it failed to send the command.
     */
    std::optional<bool> retractRPArm(const uint32_t &priority);
    /**
     * @brief It triggers the automatic sequence to bring the arm to its deployed position. It has
     *        to be set to true once.
     * @return True if it manage to deploy the arm.
     * @return False if the deploy sequence failes failed.
     * @return std::nullopt if it failed to send the command.
     */
    std::optional<bool> deployRPArm(const uint32_t &priority);
    /**
     * @brief Moves up the arm by velocity, It has to be set to true every 500 ms for the arm to
     *        move.
     * @param The control priority level.
     * @return True if the arm started moving.
     * @reutrn False if the arm failed to move.
     * @return std::nullopt if it failed to send the command.
     */
    std::optional<bool> moveRPArmUp(const uint32_t &priority);
    /**
     * @brief Moves down the arm by velocity, It has to be set to true every 500 ms for the arm to
     *        move.
     * @param The control priority level.
     * @return True if the arm started moving.
     * @reutrn False if the arm failed to move.
     * @return std::nullopt if it failed to send the command.
     */
    std::optional<bool> moveRPArmDown(const uint32_t &priority);
    /**
     * @brief Stops the movement of teh RP arm.
     * @param The control priority level.
     * @return True if it stop.
     * @return False if it failed to stop.
     * @return std::nullopt if it failed to send the command.
     */
    std::optional<bool> stopRPArm(const uint32_t &priority);
    /**
     * @brief Locks the arm in deployed or retracted position.
     * @param The control priority level.
     * @return True if manage to lock.
     * @return False if it failed to lock.
     * @return std::nullopt if it failed to send the command.
     */
    std::optional<bool> lockRPArm(const uint32_t &priority);
    /**
     * @brief Releases the lock of the arm.
     * @param The control priority level.
     * @return True if manage to release the lock.
     * @return False if it failed to release the lock.
     * @return std::nullopt if it failed to send the command.
     */
    std::optional<bool> unlockRPArm(const uint32_t &priority);
    /**
     * @brief Resets the RP arm motor driver.
     * @param The control priority level.
     * @return True if the driver reset
     * @return False if it failed to reset the driver.
     * @return std::nullopt if it failed to send the command.
     */
    std::optional<bool> resetRPArmDriver(const uint32_t &priority);
    /**
     * @brief Cleans all the error flags in the TIM RP Wagon, turning off all the alarms.
     * @param The control priority level.
     * @return True if it turned them off.
     * @return False if it did not turned them off.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    std::optional<bool> acknowledgeErrors(const uint32_t &priority);
    /**
     * @brief Gets the current status of the TIM RP Wagon.
     * @param The priority is used to refresh the timer to be aware on who requested the status for
     *        the last time.
     * @return nlohmann::json with the current status
     */
    nlohmann::json getStatus() override;

 private:
    std::shared_ptr<crf::actuators::timrpwagon::ITIMRPWagon> timRPWagon_;
    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::timrpwagon
