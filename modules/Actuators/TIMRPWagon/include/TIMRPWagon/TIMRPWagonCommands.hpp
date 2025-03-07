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
 * @brief Class that works as container to save and access the current values that were given to
 *        the PLC to control the Train Inspection Monorail (TIM).
*/
class TIMRPWagonCommands {
 public:
    TIMRPWagonCommands();
    TIMRPWagonCommands(const TIMRPWagonCommands&);
    TIMRPWagonCommands(TIMRPWagonCommands&&) = delete;
    ~TIMRPWagonCommands() = default;

    /**
     * @brief Interprets the information received from the Siemens PLC and saves it in all the
     *        different variables.
     * @return True if manage to store all the values that were input.
     * @return False if it failed.
     */
    bool parseSiemensPLCBuffer(const std::string& buffer,
        std::map<std::string, std::pair<unsigned int, unsigned int>> variablesDBLocation);
    /**
     * @brief resets the values of all the variables.
     */
    void clear();
    /**
     * @return True if is empty
     * @return False if is not.
     */
    bool isEmpty() const;
    /**
     * @brief It is used to move in velocity the arm up and down.
     * @return True to enable the motion.
     * @return False to disable the motion.
     */
    bool enableRPArmManualControl() const;
    /**
     * @brief It triggers the automatic sequence to bring the arm to its retracted position. It has
     *        to be set to true once.
     * @return True to move.
     * @return False to do nothing.
     */
    bool retractRPArm() const;
    /**
     * @brief It triggers the automatic sequence to bring the arm to its deployed position. It has
     *        to be set to true once.
     * @return True to move.
     * @return False to do nothing.
     */
    bool deployRPArm() const;
    /**
     * @brief It moves up the arm by velocity, this field has to be set to true every 500 ms for
     *        the arm to move. It only works if the manual control is enabled.
     * @return True to move.
     * @return False to stop the movement.
     */
    bool moveRPArmUp() const;
    /**
     * @brief It moves down the arm by velocity, this field has to be set to true every 500 ms for
     *        the arm to move. It only works if the manual control is enabled.
     * @return True to move.
     * @return False to stop the movement.
     */
    bool moveRPArmDown() const;
    /**
     * @brief It releases the lock of the arm in either the deployed or retracted position. It only
     *        works if the manual control is enabled.
     * @return True to release the lock.
     * @return False to engage the lock.
     */
    bool releaseLockRPArm() const;
    /**
     * @brief It stops the arm during the automatic sequences.
     * @return True to stop.
     * @return False to do nothing.
     */
    bool stopRPArm() const;
    /**
     * @brief Used to go from an error to and IDLE state.
     * @return True to acknowledge the errors.
     * @return False does nothing.
     */
    bool acknowledgeError() const;
    /**
     * @brief Resets the driver of the motor.
     * @return True to send the reset signal.
     * @return False does nothing.
     */
    bool resetRPArmDriver() const;


 private:
    utility::logger::EventLogger logger_;
    std::atomic<bool> isEmpty_;

    std::atomic<bool> enableRPArmManualControl_;
    std::atomic<bool> retractRPArm_;
    std::atomic<bool> deployRPArm_;
    std::atomic<bool> moveRPArmUp_;
    std::atomic<bool> moveRPArmDown_;
    std::atomic<bool> releaseLockRPArm_;
    std::atomic<bool> stopRPArm_;
    std::atomic<bool> acknowledgeError_;
    std::atomic<bool> resetRPArmDriver_;
};

}  // namespace crf::actuators::timrpwagon
