/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <optional>
#include <map>
#include <tuple>
#include <memory>

#include "CommonInterfaces/IInitializable.hpp"
#include "TIM/TIMConfiguration.hpp"
#include "TIM/LHCObstacle.hpp"
#include "TIM/TIMAlarms.hpp"

namespace crf::actuators::tim {

class ITIM: public utility::commoninterfaces::IInitializable {
 public:
    ~ITIM() override = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief Checks if the client has connection with the TIM and if there is a heartbeat.
     * 
     * @return true if there is connection.
     * @return false if there is no connection.
     */
    virtual bool isConnected() = 0;
    /**
     * @brief Allows to set manually the current position of the TIM. This methods is useful
     *        whenever the TIM is moved manually and it didn't had time to read a barcode to know
     *        its new position.
     * 
     * @param position The DCUM position to which the TIM will go.
     * @return true if the curent position was correctly setup.
     * @return false if it failed
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<bool> setCurrentPosition(const float &position) = 0;
    /**
     * @brief Sets the position in DCUM that the TIM will go to in its next movement.
     * 
     * @param position The DCUM position to which the TIM will go.
     * @return true if the target position was correctly setup.
     * @return false if it failed.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> setTargetPosition(const float &position) = 0;
    /**
     * @brief Sets the velocity in m/s that the TIM will have in its next movement to the target
     *        position.
     * 
     * @param velocity The velocity (m/s) that the TIM will have. A positive value means that the TIM will
     *        move forward and a negative one backward.
     * @return true if the target position was correctly setup.
     * @return false if it failed.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> setTargetVelocity(const float &velocity) = 0;
    /**
     * @brief Executes the movement towards the target position with the target velocity, it waits
     *        until the TIM starts to move, with a defined timeout.
     * 
     * @param position The DCUM position to which the TIM will go.
     * @param velocity The velocity (m/s) that the TIM will try to have. A positive value means that the TIM
     *        will move forward and a negative one backward.
     * @return true if it manage to start the movement.
     * @return false if it failed to move.
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<bool> moveToTarget(const float &position, const float &velocity) = 0;
    /**
     * @brief Starts the movement of the TIM with a constant velocity. This command most be called
     *        constantly (0.5 Hz)for the TIM to keep moving.
     * 
     * @param velocity The velocity (m/s) of TIM during the jogging. A positive value means that the TIM
     *        will move forward and a negative one backward. It can not be bigger than the maximum
     *        allowed by the TIM at that time
     * @return true if it manage to start the movement.
     * @return false if it failed.
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<bool> jog(const float &velocity) = 0;
    /**
     * @brief Gives the current position of the TIM, measured in DCUM.
     * 
     * @return The value if it was able to get the position.
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<float> getCurrentPosition() = 0;
    /**
     * @brief Gives the position in DCUM that the TIM has as an objective.
     * 
     * @return The value if it was able to get the position.
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<float> getTargetPosition() = 0;
    /**
     * @brief Gives the current velocity of the TIM, measured in m/s.
     * 
     * @return The value if it was able to get the velocity. The direction is defined by the sign,
     *         positive is forward and negative is backward.
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<float> getCurrentVelocity() = 0;
    /**
     * @brief Gives the velocity in m/s that the TIM has as an objective.
     * 
     * @return The value if it was able to get the velocity. The direction is defined by the sign,
     *         positive is forward and negative is backward.
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<float> getTargetVelocity() = 0;
    /**
     * @brief Gives the maximum velocity in m/s that the TIM can currently have.
     * 
     * @return The value if it was able to get the velocity. The value has no sign, is an absolute
     *         value.
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<float> getCurrentMaximumVelocity() = 0;
    /**
     * @brief Checks if the TIM is moving.
     * 
     * @return true if it is moving.
     * @return false if stopped.
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<bool> isMoving() = 0;
    /**
     * @brief Checks if the TIM has arrived to the target position.
     * 
     * @return true if it arrived.
     * @return false if it hasn't arrived.
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<bool> isTargetReached() = 0;
    /**
     * @brief It stops the TIM with a controlled movement reducing the velocity, following the
     *        internal parameters already predefined on its own controller.
     * 
     * @return true if it stopped the movement.
     * @return false if it failed.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> stop() = 0;
    /**
     * @brief It stops the TIM with the mechanical breaks.
     * 
     * @return true if it stopped the movement.
     * @return false if it failed.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> emergencyStop() = 0;
    /**
     * @brief It extends the chargning arm until it arrives to its limit in the PLC.
     * 
     * @return true if it was extend.
     * @return false if it failed to extend.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> extendChargingArm() = 0;
    /**
     * @brief It retracts the chargning arm until it arrives to its limit in the PLC.
     * 
     * @return true if it was retracted.
     * @return false if it failed to retract.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> retractChargingArm() = 0;
    /**
     * @brief It starts the charging process.
     * 
     * @return true if it is charging.
     * @return false if it failed to charge.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> startCharging() = 0;
    /**
     * @brief It stops the charging process.
     * 
     * @return true if it is not charging.
     * @return false if it is charging.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> stopCharging() = 0;
    /**
     * @brief Checks if the TIM is charging.
     * 
     * @return true if it is charging.
     * @return false if it is not charging.
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<bool> isCharging() = 0;
    /**
     * @brief Gives the charging current of the TIM.
     * 
     * @return The current value in Amperes.
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<float> getChargingCurrent() = 0;
    /**
     * @brief Gives the voltage of the batteries of the TIM.
     * 
     * @return The voltage value (V).
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<float> getBatteryVoltage() = 0;
    /**
     * @brief Gives the current consumption in Amperes of the TIM.
     * 
     * @return The value if it was able to get the current.
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<float> getBatteryCurrent() = 0;
    /**
     * @brief It sets the economy mode, which turns off every component of the TIM except the
     *        battery, router and main PLC.
     * 
     * @return true if it is in economy mode.
     * @return false if it failed to go into economy mode.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> enableEconomyMode() = 0;
    /**
     * @brief It unsets the economy mode. I takes a few seconds since the different devices have to
     *        be initialized again, like for example the driver of the motor.
     * 
     * @return true if it is not in economy mode.
     * @return false if it is in economy mode.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> disableEconomyMode() = 0;
    /**
     * @brief Checks if the TIM is economy mode.
     * 
     * @return true if it is in economy mode.
     * @return false if it is not in economy mode and all the devices were initialized.
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<bool> isInEconomy() = 0;
    /**
     * @brief It sends the signal to reboot the Robot Arm Wagon, it takes around 5 seconds.
     *        Warning, if the control PC is in that wagon it might reboot it as well.
     * 
     * @return true if it reboot the wagon.
     * @return false if it failed.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> rebootRobotArmWagon() = 0;
    /**
     * @brief Gives the beginning and end in DCUM of the closest obstacle areas in the back and the
     *        front of the TIM.
     * 
     * @return A pair in which the first element indicates the previous obstacle area and the
     *         second one the next obstacle.
     * @return Empty array if it failed to to retrieve the information from the PLC.
     */
    virtual std::array<LHCObstacle, 2> getClosestObstacleAreas() = 0;
    /**
     * @brief Checks if the TIM is in an obstacle area
     * 
     * @return LHCObstcale object if the robot, empty if is not in a obstacle area.
     */
    virtual LHCObstacle getCurrentObstacleArea() = 0;
    /**
     * @brief It allows you to change a specific field (maximum velocity) of a permanent obstacle
     *        area. This change will be reverted after 2 minutes. It also allows you to store new
     *        temporal obstacle areas that will dissapear eveytime this object is detroyed. These
     *        temporal obstacles will only be added if the type is defined as temporal. They are
     *        also not sent to the PLC so they don't affect the low level movement of the train.
     *        The identifier of every obstacle is unique.
     * 
     * @param obstacle parameters of the obstacle area
     * @return true if it manage to change or add the obstacle
     * @return false if it failed, due to invalid request.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> setObstacleArea(const LHCObstacle &obstacle) = 0;
    /**
     * @brief Tells the train if all the devices are retracted in a safe position for it to pass
     *        through an obstacle area that requires it. Like for example the doors.
     * 
     * @param deviceStatus true if the devices are retracted, false otherwise.
     * @return true if it manage to set the variable.
     * @return false if it failed.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> devicesRetracted(bool deviceStatus) = 0;
    /**
     * @brief Tells if the train thinks that all the devices are retracted and is safe to go into
     *        an obstacle area.
     * 
     * @return true if it thinks that the devices are retracted, false otherwise.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> devicesRetracted() = 0;
    /**
     * @brief Checks if warning of the front field is triggered. This happens whenever we are close
     *        to an obstacle either if it is known or unknown.
     * 
     * @return true if it is active.
     * @return false if it is not active
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<bool> isFrontWarningFieldActive() = 0;
    /**
     * @brief Checks if warning of the front field is triggered. This happens whenever we are close
     *        to an obstacle either if it is known or unknown.
     * 
     * @return true if it is active.
     * @return false if it is not active
     * @return std::nullopt if it failed to retrieve the information from the PLC.
     */
    virtual std::optional<bool> isBackWarningFieldActive() = 0;
    /**
     * @brief Tells the train if it is save to move from the perspetive of all the devices it
     *        doesnt control. For example, if the meachanical stabilizer is enables the train cant
     *        move but the PLC doesnt know that.
     * 
     * @param allow true if the movement is allowed, false otherwise.
     * @return true if it manage to set the variable.
     * @return false if it failed.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> allowMovement(bool allow) = 0;
    /**
     * @brief Checks if the TIM can move, is the result of checking if the train is not moving,
     *        there is no emergency stop and if is not charging.
     * 
     * @return true if it is active.
     * @return false if it is not active
     * @return std::nullopt if it failed to retrive the information from the PLC.
     */
    virtual std::optional<bool> isSafeToMove() = 0;
    /**
     * @brief Gives the status of all the alarms of the TIM. Check the definition of
     *        crf::actuators::tim::TIMAlarms to get more detailed information of each flag.
     * 
     * @return The TIMAlamrs structure.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual crf::actuators::tim::TIMAlarms getAlarms() = 0;
    /**
     * @brief Cleans all the error flags in the TIM, turning off all the alarms.
     * 
     * @return true if it turned them off.
     * @return false if it did not turned them off.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> acknowledgeAlarms() = 0;
    /**
     * @brief Gives the information about the current TIM configuration
     * 
     * @return the pointer to the crf::actuators::tim::TIMConfiguration object.
     * @return std::nullptr if it failed
     */
    virtual std::shared_ptr<TIMConfiguration> getConfiguration() = 0;
};

}  // namespace crf::actuators::tim
