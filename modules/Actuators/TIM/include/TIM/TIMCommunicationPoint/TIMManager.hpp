/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
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
#include "CommunicationUtility/STLJSONConverter.hpp"

#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "TIM/ITIM.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::tim {

class TIMManager: public crf::utility::devicemanager::DeviceManagerWithPriorityAccess {
 public:
    explicit TIMManager(std::shared_ptr<crf::actuators::tim::ITIM> tim,
        const std::chrono::milliseconds& inizializationTimeout = std::chrono::seconds(60),
        const std::chrono::milliseconds& controlAccessTimeout = std::chrono::seconds(15));
    TIMManager(const TIMManager&) = delete;
    TIMManager(TIMManager&&) = delete;
    TIMManager() = delete;
    ~TIMManager() override;

    /**
     * @brief Allows to set manually the current position of the TIM. This methods is useful
     *        whenever the TIM is moved manually and it didn't had time to read a barcode to know
     *        its new position.
     * @param The DCUM position to which the TIM will go.
     * @param priotiy the control priority level.
     * @return True if the curent position was correctly setup.
     * @return False if it failed
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> setCurrentPosition(const float &position, const uint32_t &priority);
    /**
     * @brief Sets the position in DCUM that the TIM will go to in its next movement.
     * @param The DCUM position to which the TIM will go.
     * @param priotiy the control priority level.
     * @return True if the target position was correctly setup.
     * @return False if it failed.
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> setTargetPosition(const float &position, const uint32_t &priority);
    /**
     * @brief Sets the velocity in m/s that the TIM will have in its next movement to the target
     *        position.
     * @param The velocity (m/s) that the TIM will have. A positive value means that the TIM will
     *        move forward and a negative one backward.
     * @param priotiy the control priority level.
     * @return True if the target position was correctly setup.
     * @return False if it failed.
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> setTargetVelocity(const float &velocity, const uint32_t &priority);
    /**
     * @brief Executes the movement towards the target position with the target velocity, it waits
     *        until the TIM starts to move, with a defined timeout.
     * @param The DCUM position to which the TIM will go.
     * @param The velocity (m/s) that the TIM will try to have. A positive value means that the TIM
     *        will move forward and a negative one backward.
     * @param priotiy the control priority level.
     * @return True if it manage to start the movement.
     * @return False if it failed to move.
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> moveToTarget(const float &position, const float &velocity,
        const uint32_t &priority);
    /**
     * @brief Starts the movement of the TIM with a constant velocity. This command most be called
     *        constantly (0.5 Hz)for the TIM to keep moving 
     * @param The velocity (m/s) of TIM during the jogging. A positive value means that the TIM
     *        will move forward and a negative one backward. It can not be bigger than the maximum
     *        allowed by the TIM at that time.
     * @param priotiy the control priority level.
     * @return True if it manage to start the movement.
     * @return False if it failed.
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> jog(const float &velocity, const uint32_t &priority);
    /**
     * @brief It stops the TIM with a controlled movement reducing the velocity, following the
     *        internal parameters already predefined on its own controller.
     * @param priotiy the control priority level.
     * @return True if it stopped the movement.
     * @return False if it failed.
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> stop(const uint32_t &priority);
    /**
     * @brief It stops the TIM with the mechanical breaks.
     * @param priotiy the control priority level.
     * @return True if it stopped the movement.
     * @return False if it failed.
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> emergencyStop(const uint32_t &priority);
    /**
     * @brief It extends the chargning arm until it arrives to its limit in the PLC.
     * @param priotiy the control priority level.
     * @return True if it was extend.
     * @return False if it failed to extend.
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> extendChargingArm(const uint32_t &priority);
    /**
     * @brief It retracts the chargning arm until it arrives to its limit in the PLC.
     * @param priotiy the control priority level.
     * @return True if it was retracted.
     * @return False if it failed to retract.
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> retractChargingArm(const uint32_t &priority);
    /**
     * @brief It starts the charging process.
     * @param priotiy the control priority level.
     * @return True if it is charging.
     * @return False if it failed to charge.
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> startCharging(const uint32_t &priority);
    /**
     * @brief It stops the charging process.
     * @param priotiy the control priority level.
     * @return True if it is not charging.
     * @return False if it is charging.
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> stopCharging(const uint32_t &priority);
    /**
     * @brief It sets the economy mode, which turns off every component of the TIM except the
     *        battery, router and main PLC.
     * @param priotiy the control priority level.
     * @return True if it is in economy mode.
     * @return False if it failed to go into economy mode.
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> enableEconomyMode(const uint32_t &priority);
    /**
     * @brief It unsets the economy mode. I takes a few seconds since the different devices have to
     *        be initialized again, like for example the driver of the motor.
     * @param priotiy the control priority level.
     * @return True if it is not in economy mode.
     * @return False if it is in economy mode.
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> disableEconomyMode(const uint32_t &priority);
    /**
     * @brief It sends the signal to reboot the Robot Arm Wagon, it takes around 5 seconds.
     *        Warning, if the control PC is in that wagon it might reboot it as well.
     * @param priotiy the control priority level.
     * @return True if it reboot the wagon.
     * @return False if it failed.
     * @return std::nullopt if it failed to send the command to the PLC or not valid priority.
     */
    std::optional<bool> rebootRobotArmWagon(const uint32_t &priority);
    /**
     * @brief It allows you to change a specific field (maximum velocity) of a permanent obstacle
     *        area. This change will be reverted after 2 minutes. It also allows you to store new
     *        temporal obstacle areas that will dissapear eveytime this object is detroyed. These
     *        temporal obstacles will only be added if the type is defined as temporal. They are
     *        also not sent to the PLC so they don't affect the low level movement of the train.
     *        The identifier of every obstacle is unique.
     * @param obstacle parameters of the obstacle area
     * @param priotiy the control priority level.
     * @return True if it manage to change or add the obstacle
     * @return False if it failed, due to invalid request.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    std::optional<bool> setObstacleArea(const LHCObstacle &obstacle, const uint32_t &priority);
    /**
     * @brief Tells the train if all the devices are retracted in a safe position for it to pass
     *        through an obstacle area that requires it. Like for example the doors.
     * @param True if the devices are retracted, false otherwise.
     * @param priotiy the control priority level.
     * @return True if it manage to set the variable.
     * @return False if it failed.
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> devicesRetracted(bool deviceStatus, const uint32_t &priority);
    /**
     * @brief Tells the train if it is save to move from the perspetive of all the devices it
     *        doesnt control. For example, if the meachanical stabilizer is enables the train cant
     *        move but the PLC doesnt know that.
     * @param True if the movement is allowed, false otherwise.
     * @param priotiy the control priority level.
     * @return True if it manage to set the variable.
     * @return False if it failed.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    std::optional<bool> allowMovement(bool allow, const uint32_t &priority);
    /**
     * @brief Cleans all the error flags in the TIM, turning off all the alarms.
     * @param priotiy the control priority level.
     * @return True if it turned them off.
     * @return False if it did not turned them off.
     * @return std::nullopt if it failed to retrieve the information from the PLC or not valid
     *         priority.
     */
    std::optional<bool> acknowledgeAlarms(const uint32_t &priority);
    /**
     * @brief Gets the current status of the tim.
     * @param The priority is used to refresh the timer to be aware on who requested the status for
     *        the last time.
     * @return nlohmann::json with the current status
     */
    nlohmann::json getStatus() override;

 private:
    std::shared_ptr<crf::actuators::tim::ITIM> tim_;
    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::tim
