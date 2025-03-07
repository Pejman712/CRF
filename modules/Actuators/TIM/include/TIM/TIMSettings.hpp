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
 * @brief Class that works as container to save and access the current setting values that were
 *        given to the PLC to control the Train Inspection Monorail (TIM).
 */
class TIMSettings {
 public:
    TIMSettings();
    TIMSettings(const TIMSettings&);
    TIMSettings(TIMSettings&&) = delete;
    ~TIMSettings() = default;

    /**
     * @brief Interprets the information received from the Siemens PLC and saves it in all the
     *        different variables.
     * 
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
     * @brief Function to check if the TIM setting are empty.
     * 
     * @return true if is empty.
     * @return false if is not.
     */
    bool isEmpty() const;
    /**
     * @brief Tells which is the latest target position received by the PLC.
     * 
     * @return DCUM value in meters [m].
     */
    float targetPosition() const;
    void targetPosition(float input);
    /**
     * @brief Tells which is the latest desired velocity received by the PLC. The robot will try to
     *        have this velocity whenever it is moving.
     * 
     * @return Value in meters per second [m/s].
     */
    float targetVelocity() const;
    void targetVelocity(float input);
    /**
     * @brief Tells if the PLC received the signal to move to the target.
     * 
     * @return true if it received it, false otherwise.
     */
    float positionSetManually() const;
    void positionSetManually(float input);
    /**
     * @brief Tells which is the latest targeted obstacle to be modified.
     * 
     * @return int8_t numeric identifier
     */
    int8_t obstacleID() const;
    void obstacleID(int8_t input);
    /**
     * @brief Tells which is the latest maximum velocity received by the PLC that is use to modify
     *        the targeted obstacles.
     * 
     * @return float value in meters per second [m/s].
     */
    float obstacleMaximumVelocity() const;
    void obstacleMaximumVelocity(float input);

 private:
    utility::logger::EventLogger logger_;
    std::atomic<bool> isEmpty_;

    std::atomic<float> targetPosition_;
    std::atomic<float> targetVelocity_;
    std::atomic<float> positionSetManually_;
    std::atomic<int8_t> obstacleID_;
    std::atomic<float> obstacleMaximumVelocity_;
};

}  // namespace crf::actuators::tim
