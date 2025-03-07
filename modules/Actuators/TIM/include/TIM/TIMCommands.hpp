/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <map>
#include <utility>
#include <shared_mutex>

#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::tim {

/**
 * @brief Class that works as container to save and access the current values that were given to
 *        the PLC to control the Train Inspection Monorail (TIM).
*/
class TIMCommands {
 public:
    TIMCommands();
    TIMCommands(const TIMCommands&);
    TIMCommands(TIMCommands&&) = delete;
    ~TIMCommands() = default;

    /**
     * @brief Interprets the information received from the Siemens PLC and saves it in all the
     *        different variables.
     * 
     * @param buffer 
     * @param variablesDBLocation 
     * @return true if manage to store all the values that were input.
     * @return false if it failed.
     */
    bool parseSiemensPLCBuffer(const std::string& buffer,
        std::map<std::string, std::array<unsigned int, 2>> variablesDBLocation);
    /**
     * @brief resets the values of all the variables.
     */
    void clear();
    /**
     * @return true if it is empty
     * @return false if it is not empty
     */
    bool isEmpty() const;
    /**
     * @brief Tells the heartbeat of the local system that the PLC received.
     * 
     * @return int16_t 
     */
    int16_t localHeartbeat() const;
    void localHeartbeat(int16_t input);
    /**
     * @brief Tells if the PLC received the signal to set the manual position.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool setPositionManually() const;
    void setPositionManually(bool input);
    /**
     * @brief Tells if the PLC received the signal to move to the target.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool moveToTarget() const;
    void moveToTarget(bool input);
    /**
     * @brief Tells if the PLC received the signal to start jogging forward.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool jogForward() const;
    void jogForward(bool input);
    /**
     * @brief Tells if the PLC received the signal to start jogging backward.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool jogBackward() const;
    void jogBackward(bool input);
    /**
     * @brief Tells if the PLC received the signal to stop.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool stop() const;
    void stop(bool input);
    /**
     * @brief Tells if the PLC received the signal to make an emergency stop.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool emergencyStop() const;
    void emergencyStop(bool input);
    /**
     * @brief Tells if the PLC received the signal to put the charging arm in manual control.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool chargingArmManualControl() const;
    void chargingArmManualControl(bool input);
    /**
     * @brief Tells if the PLC received the signal to start charging.
     * @return True if it received it, false otherwise.
     */
    bool startCharging() const;
    void startCharging(bool input);
    /**
     * @brief Tells if the PLC received the signal to stop charging.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool stopCharging() const;
    void stopCharging(bool input);
    /**
     * @brief Tells if the PLC received the signal to extend the charging arm.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool extendChargingArm() const;
    void extendChargingArm(bool input);
    /**
     * @brief Tells if the PLC received the signal to retract the charging arm.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool retractChargingArm() const;
    void retractChargingArm(bool input);
    /**
     * @brief Tells if the PLC received the signal to enable economy mode.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool enableEconomyMode() const;
    void enableEconomyMode(bool input);
    /**
     * @brief Tells if the PLC received the signal to disable economy mode.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool disableEconomyMode() const;
    void disableEconomyMode(bool input);
    /**
     * @brief Tells if the PLC received the signal to restart the robot arm wagon.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool rebootRobotArmWagon() const;
    void rebootRobotArmWagon(bool input);
    /**
     * @brief Tells if the PLC received the signal to change the maximum velocity of the given
     *        obstacle.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool setObstacleMaximumVelocity() const;
    void setObstacleMaximumVelocity(bool input);
    /**
     * @brief Tells if the PLC received the signal that tells if all devices are retracted.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool devicesRetracted() const;
    void devicesRetracted(bool input);
    /**
     * @brief Tells if the PLC received the signal that allows the movement.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool allowMovement() const;
    void allowMovement(bool input);
    /**
     * @brief Tells if the PLC received the signal to turn off all the alarms.
     * 
     * @return true if it received it.
     * @return false otherwise..
     */
    bool acknowledgeAlarms() const;
    void acknowledgeAlarms(bool input);

 private:
    utility::logger::EventLogger logger_;
    std::atomic<bool> isEmpty_;

    std::atomic<int16_t> localHeartbeat_;
    std::atomic<bool> setPositionManually_;
    std::atomic<bool> moveToTarget_;
    std::atomic<bool> jogForward_;
    std::atomic<bool> jogBackward_;
    std::atomic<bool> stop_;
    std::atomic<bool> emergencyStop_;
    std::atomic<bool> chargingArmManualControl_;
    std::atomic<bool> startCharging_;
    std::atomic<bool> stopCharging_;
    std::atomic<bool> extendChargingArm_;
    std::atomic<bool> retractChargingArm_;
    std::atomic<bool> enableEconomyMode_;
    std::atomic<bool> disableEconomyMode_;
    std::atomic<bool> rebootRobotArmWagon_;
    std::atomic<bool> setObstacleMaximumVelocity_;
    std::atomic<bool> devicesRetracted_;
    std::atomic<bool> allowMovement_;
    std::atomic<bool> acknowledgeAlarms_;
};

}  // namespace crf::actuators::tim
