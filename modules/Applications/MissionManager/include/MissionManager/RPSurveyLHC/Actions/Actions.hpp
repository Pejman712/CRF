/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/

#pragma once

#include <string>
#include <atomic>
#include <memory>
#include <deque>
#include <mutex>
#include <vector>

#include <nlohmann/json.hpp>
#include "Types/JsonConverters.hpp"

#include "CommonInterfaces/IInitializable.hpp"
#include "TIM/ITIM.hpp"
#include "RPSensor/IRPSensor.hpp"
#include "EventLogger/EventLogger.hpp"
#include "MissionManager/RPSurveyLHC/Actions/BackgroundRPMeasurement.hpp"
#include "MissionUtility/TIMMovement/TIMMovement.hpp"
#include "MissionUtility/IDeployableDevice.hpp"

namespace crf::applications::missionmanager::rpsurveylhc {

class Actions: public utility::commoninterfaces::IInitializable {
 public:
    Actions() = delete;
    explicit Actions(
        std::shared_ptr<crf::actuators::tim::ITIM> tim,
        std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable,
        std::shared_ptr<crf::sensors::rpsensor::IRPSensor> rpSensor,
        const nlohmann::json& configFile);
    ~Actions();

    bool deinitialize() override;
    bool initialize() override;

    /**
     * @brief Deploys the deployable device with the neccessary checks.
     *
     * @return true if succeded
     * @return false otherwise
     */
    bool deploy();
    /**
     * @brief Retracts the deployable device with the neccessary checks.
     *
     * @return true if succeded
     * @return false otherwise
     */
    bool retract();
    /**
     * @brief Performs the starting sequence of the mission. Retracting the arm, moving to the
     * starting point and setting up the next stage.
     *
     * @return true if succeded
     * @return false otherwise
     */
    bool goToStart();
    /**
     * @brief Performs the main mission phase. Moves the train to the ending DCUM specified
     * and retracts and deploys the arm depending on the obstacles. Measurement of radiation
     * is done on the background.
     *
     * @return true if succeded
     * @return false otherwise
     */
    bool goToEnd();
    /**
     * @brief Preapres the train for parking. Retracts the arm, moves the train, charge and
     * can activate economy or other fucntions.
     *
     * @return true if succeded
     * @return false otherwise
     */
    bool goHome();
    /**
     * @brief Stops the train and performs the actions to pause the mission.
     *
     * @return true if succeded
     * @return false otherwise
     */
    bool pause();
    /**
     * @brief Stops the train and the arm movement and performs the actions to stop the mission.
     *
     * @return true if succeded
     * @return false otherwise
     */
    bool stop();
    /**
     * @brief Emergency stop on the train and the arm, stops all movement.
     *
     * @return true if succeded
     * @return false otherwise
     */
    bool emergencyStop();
    /**
     * @brief Get the Status of the running RP survey
     *
     * @return nlohmann::json with several fields to send to the operator monitoring the survey
     */
    nlohmann::json getStatus();
    /**
     * @brief Set the Survey Parameters. These include the parameters the user can set suchs as
     * starting, stopping, and home DCUMs among others.
     *
     * @param json with the fields and necessary data.
     */
    void setSurveyParameters(const nlohmann::json& json);

 private:
    std::shared_ptr<crf::actuators::tim::ITIM> tim_;
    std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable_;
    std::shared_ptr<crf::sensors::rpsensor::IRPSensor> rpSensor_;

    crf::utility::logger::EventLogger logger_;

    crf::utility::missionutility::TIMMovement timMovement_;
    BackgroundRPMeasurement measurements_;

    std::thread startMovementThread_;
    std::thread deviceMovementThread_;

    std::atomic<bool> stopThreads_;
    std::atomic<bool> goingToStart_;
    std::atomic<bool> goingToEnd_;
    std::atomic<bool> finished_;

    bool initialized_;
    bool surveyParametersSet_;

    std::deque<std::string> buffer_;
    std::mutex bufferMtx_;

    float startingDCUM_;
    float endingDCUM_;
    float parkingDCUM_;

    float timMovementThreshold_;
    uint32_t maxBufferSize_;

    bool initializeClients();
    bool deinitializeClients();
    void clearMission();
    void deviceMovement();
    bool isTIMParked();
    void errorLog(const std::string& logg);
};

}  // namespace crf::applications::missionmanager::rpsurveylhc
