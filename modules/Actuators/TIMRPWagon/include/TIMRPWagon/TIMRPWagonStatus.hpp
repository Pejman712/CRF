/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <map>
#include <utility>
#include <shared_mutex>

#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::timrpwagon {

/**
 * @brief Class that works as container to save and access the status variables of the TIM
 *        RP Wagon.
*/
class TIMRPWagonStatus {
 public:
    TIMRPWagonStatus();
    TIMRPWagonStatus(const TIMRPWagonStatus&);
    TIMRPWagonStatus(TIMRPWagonStatus&&) = delete;
    ~TIMRPWagonStatus() = default;

    /**
     * @brief Interprets the information received from the Siemens PLC and saves it in all the
     *        different variables.
     * @return True if manage to store all the status values.
     * @return False if it failed.
     */
    bool parseSiemensPLCBuffer(const std::string& buffer,
        std::map<std::string, std::pair<unsigned int, unsigned int>> variablesDBLocation);
    /**
     * @brief resets the values of all the variables.
     */
    void clear();
    /**
     * @brief Function to check if the TIM status is empty.
     * @return True if is empty
     * @return False if is not.
     */
    bool isEmpty() const;
    /**
     * @brief Tells if the RP arm is in retracted position.
     * @return True if the arm is retracted.
     * @return False if the arm is not retracted.
     */
    bool rpArmRetracted() const;
    /**
     * @brief Tells if the RP arm is in between the retracted and deployed position.
     * @return True if the arm is in the middle.
     * @return False if the arm is not in the middle.
     */
    bool rpArmInTheMiddle() const;
    /**
     * @brief Tells if the RP arm is in deployed position.
     * @return True if the arm is deployed.
     * @return False if the arm is not deployed.
     */
    bool rpArmDeployed() const;
    /**
     * @brief Tells if there is a problem with the arm. For example if the arm doesn't for a given
     *        time after sending a move command.
     * @return True if there is an error.
     * @return False if there are no errors.
     */
    bool rpArmInError() const;

 private:
    utility::logger::EventLogger logger_;
    std::atomic<bool> isEmpty_;

    std::atomic<bool> rpArmRetracted_;
    std::atomic<bool> rpArmInTheMiddle_;
    std::atomic<bool> rpArmDeployed_;
    std::atomic<bool> rpArmInError_;
};

}  // namespace crf::actuators::timrpwagon
