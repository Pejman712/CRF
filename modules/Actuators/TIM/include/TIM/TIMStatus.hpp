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
 * @brief Class that works as container to save and access the status variables of the Train
 *        Inspection Monorail (TIM).
*/
class TIMStatus {
 public:
    TIMStatus();
    TIMStatus(const TIMStatus&);
    TIMStatus(TIMStatus&&) = delete;
    ~TIMStatus() = default;

    /**
     * @brief Interprets the information received from the Siemens PLC and saves it in all the
     *        different variables.
     * 
     * @return true if manage to store all the status values.
     * @return false if it failed.
     */
    bool parseSiemensPLCBuffer(const std::string& buffer,
        std::map<std::string, std::array<unsigned int, 2>> variablesDBLocation);
    /**
     * @brief resets the values of all the variables.
     */
    void clear();
    /**
     * @brief Function to check if the TIM status is empty.
     * 
     * @return true if is empty
     * @return false if is not.
     */
    bool isEmpty() const;
    /**
     * @brief Tells the current DCUM position of the TIM.
     * 
     * @return Value in meters [m].
     */
    float timPosition() const;
    void timPosition(float input);
    /**
     * @brief Tells the current velocity of the TIM.
     * 
     * @return Value in meters per second [m/s].
     */
    float timVelocity() const;
    void timVelocity(float input);
    /**
     * @brief Tells if the target position was reached.
     * 
     * @return true if it did.
     * @return false if it did not.
     */
    bool targetReached() const;
    void targetReached(bool input);
    /**
     * @brief Tells if the TIM is stopped and is not moving.
     * 
     * @return true if it did stop.
     * @return false if it did not.
     */
    bool timStopped() const;
    void timStopped(bool input);
    /**
     * @brief Tells if the TIM is charging.
     * 
     * @return true if it is charging.
     * @return false if it is not charging.
     */
    bool charging() const;
    void charging(bool input);
    /**
     * @brief Tells the charging current of the TIM.
     * 
     * @return Value in amperes [A].
     */
    float chargingCurrent() const;
    void chargingCurrent(float input);
    /**
     * @brief Tells the battery voltage of the TIM.
     * 
     * @return Value in volts [V].
     */
    float batteryVoltage() const;
    void batteryVoltage(float input);
    /**
     * @brief Tells the current consumption of the TIM.
     * 
     * @return Value in volts [V].
     */
    float batteryCurrent() const;
    void batteryCurrent(float input);
    /**
     * @brief Tells if the economy mode of the TIM is activated.
     * 
     * @return true if it is active.
     * @return false if it is not active.
     */
    bool economyMode() const;
    void economyMode(bool input);
    /**
     * @brief Tells if the charging arm connected and charging.
     * 
     * @return true if it is connected.
     * @return false if it is not connected.
     */
    bool chargingArmConnected() const;
    void chargingArmConnected(bool input);
    /**
     * @brief Tells if the charging arm of the TIM is retracted.
     * 
     * @return true if it is retracted.
     * @return false if it is not retracted.
     */
    bool chargingArmRetracted() const;
    void chargingArmRetracted(bool input);
    /**
     * @brief Tells if the warning signal of the front field is triggered.
     * 
     * @return true if it is active.
     * @return false if it is not active.
     */
    bool frontWarningField() const;
    void frontWarningField(bool input);
    /**
     * @brief Tells if the warning signal of the back field is triggered.
     * 
     * @return true if it is active.
     * @return false if it is not active.
     */
    bool backWarningField() const;
    void backWarningField(bool input);
    /**
     * @brief Function to check the current value of the TIM heartbeat with the PLC.
     * 
     * @return the value of the heartbeat.
     */
    int16_t timHeartbeat() const;
    void timHeartbeat(int16_t input);
    /**
     * @brief Function to check if the main motor is on.
     * 
     * @return true if it's on.
     * @return false if it's off.
     */
    bool mainMotorOn() const;
    void mainMotorOn(bool input);
    /**
     * @brief Function to check if is safe to move.
     * 
     * @return true if it is ok.
     * @return false if it is not ok.
     */
    bool safeToMove() const;
    void safeToMove(bool input);

 private:
    utility::logger::EventLogger logger_;
    std::atomic<bool> isEmpty_;

    std::atomic<float> timPosition_;
    std::atomic<float> timVelocity_;
    std::atomic<bool> targetReached_;
    std::atomic<bool> timStopped_;
    std::atomic<bool> charging_;
    std::atomic<float> chargingCurrent_;
    std::atomic<float> batteryVoltage_;
    std::atomic<float> batteryCurrent_;
    std::atomic<bool> economyMode_;
    std::atomic<bool> chargingArmConnected_;
    std::atomic<bool> chargingArmRetracted_;
    std::atomic<bool> frontWarningField_;
    std::atomic<bool> backWarningField_;
    std::atomic<int16_t> timHeartbeat_;
    std::atomic<bool> mainMotorOn_;
    std::atomic<bool> safeToMove_;
};

}  // namespace crf::actuators::tim
