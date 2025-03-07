/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include <nlohmann/json.hpp>

#include "DeviceManager/IDeviceManager.hpp"

namespace crf::applications::missionmanager {

class IMissionManager: public crf::utility::devicemanager::IDeviceManager {
 public:
    virtual ~IMissionManager() = default;
    /**
     * @brief This function is used to start the mission.
     * @return true if the mission was started correctly.
     * @return false otherwise.
     */
    virtual bool start() = 0;
    /**
     * @brief This function is used to go to the next step in the mission.
     * @return true if the next process was successful.
     * @return false otherwise.
     */
    virtual bool next() = 0;
    /**
     * @brief This function is used to stop the mission.
     * @return true if the mission was stopped correctly.
     * @return false otherwise.
     */
    virtual bool stop() = 0;
    /**
     * @brief This function is used to pause the mission.
     * @return true if the mission was paused correctly.
     * @return false otherwise.
     */
    virtual bool pause() = 0;
    /**
     * @brief This function is used to resume the mission from a pause state.
     * @return true if the mission was resumed correctly.
     * @return false otherwise.
     */
    virtual bool resume() = 0;
    /**
     * @brief This function is used to move the robot to a home position.
     * @return true if the home position was reached correctly.
     * @return false otherwise.
     */
    virtual bool goHome() = 0;
    /**
     * @brief This function is used to recharge the robot,it is optional since some robots cannot recharge.
     * @return true if the home position was reached correctly.
     * @return false otherwise.
     */
    virtual bool recharge() {return true;}
    /**
     * @brief This function is used to get a status update of the robot.
     * @return a JSON with all the information needed for the mission.
     */
    virtual nlohmann::json getStatus() = 0;
    /**
     * @brief This function is used to set a specific status on the robot.
     * @return true if the status was set correctly.
     * @return false otherwise.
     */
    virtual bool setStatus(const nlohmann::json& json) = 0;
    /**
     * @brief This function is used to launch an emergency on the system.
     * @return true if the emergency was launched correctly.
     * @return false otherwise.
     */
    virtual bool emergency() = 0;
    /**
     * @brief This function is used to recover the robot after an emergency was launched.
     * @return true if the system was able to recover from the emergency.
     * @return false otherwise.
     */
    virtual bool rearm() = 0;
};

}  // namespace crf::applications::missionmanager
