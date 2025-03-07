/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/
#pragma once

#include <condition_variable>
#include <string>
#include <atomic>
#include <thread>
#include <memory>

#include <nlohmann/json.hpp>

#include "TIM/ITIM.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::utility::missionutility {

/**
 * @brief This class runs on the premise that the TIM is already initialized.
 *
 */
class TIMMovement {
 public:
    TIMMovement() = delete;
    TIMMovement(std::shared_ptr<crf::actuators::tim::ITIM> tim, const nlohmann::json& configFile);
    ~TIMMovement();

    /**
     * @brief Blocking method to move the train to an specific DCUM.
     *
     * @param dcum Goal of the movement.
     * @return true Sucessfully arrived.
     * @return false otherwise.
     */
    bool moveToDCUM(float dcum);
    /**
     * @brief Checks if the train is being moved.
     *
     * @return true if the train is moving.
     * @return false otherwise.
     */
    bool isTIMMoving();
    /**
     * @brief Starts the obstacle detection thread to activate variables such as entering obstacle
     *        or exiting.
     *
     */
    void startObstacleDetection();
    /**
     * @brief Stops the obstacle detection thread.
     *
     */
    void stopObstacleDetection();
    /**
     * @brief Checks if the train is entering an obstacle area (inside the margin provided).
     *
     * @return true if it's enetering.
     * @return false otherwise.
     */
    bool isEnteringObstacle();
    /**
     * @brief Checks if the train is exiting an obstacle area (within the margin provided).
     *
     * @return true if it's exiting.
     * @return false otherwise.
     */
    bool isExitingObstacle();
    /**
     * @brief Checks if the train is inside an obstacle area.
     *
     * @return true if it's inside.
     * @return false otherwise.
     */
    bool isInObstacle();
    /**
     * @brief Checks if the next obstacle the train will encounter needs the devices to be
     *        retracted.
     *
     * @return true if they need to be retracted.
     * @return false otherwise.
     */
    bool needToRetractForNextObstacle();

 private:
    std::shared_ptr<crf::actuators::tim::ITIM> tim_;
    utility::logger::EventLogger logger_;

    std::atomic<bool> enteringObstacle_;
    std::atomic<bool> exitingObstacle_;
    std::atomic<bool> inObstacle_;
    std::atomic<bool> detectingObstacle_;
    std::atomic<bool> movingTIM_;
    std::atomic<bool> needToRetractInNext_;

    std::thread DCUMTrackerThread_;
    std::condition_variable cv_;

    float timVelocity_;
    float obstacleMargin_;
    float timMovementThreshold_;
    float obstacleAreaExtension_;
    float headOffset_;
    float tailOffset_;

    const std::chrono::milliseconds loopFrequency_ = std::chrono::milliseconds(250);

    void DCUMTracker();
};

}  // namespace crf::utility::missionutility
