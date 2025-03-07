/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#include <condition_variable>
#include <string>
#include <atomic>
#include <thread>

#include <nlohmann/json.hpp>

#include "MissionUtility/TIMMovement/TIMMovement.hpp"

namespace crf::utility::missionutility {

TIMMovement::TIMMovement(std::shared_ptr<crf::actuators::tim::ITIM> tim,
    const nlohmann::json& configFile):
    tim_(tim),
    logger_("TIMMovement"),
    enteringObstacle_(false),
    exitingObstacle_(false),
    inObstacle_(false),
    detectingObstacle_(false),
    movingTIM_(false),
    needToRetractInNext_(true) {
    logger_->debug("CTor");
    timVelocity_ = configFile["Movement"]["DefaultTIMVelocity"].get<float>();
    obstacleMargin_ = configFile["Movement"]["Obstacles"]["Margin"].get<float>();
    timMovementThreshold_ = configFile["Movement"]["TIMPositionThreshold"].get<float>();
    obstacleAreaExtension_ = configFile["Movement"]["Obstacles"]["AreaExtension"].get<float>();
    headOffset_ = configFile["TIMHeadOffset"].get<float>();
    tailOffset_ = configFile["TIMTailOffset"].get<float>();
}

TIMMovement::~TIMMovement() {
    logger_->debug("DTor");
    if (!tim_->stop()) {
        logger_->error("Cannot stop the TIM");
    }
    detectingObstacle_ = false;
    if (DCUMTrackerThread_.joinable()) {
        DCUMTrackerThread_.join();
    }
}

bool TIMMovement::moveToDCUM(float dcum) {
    logger_->debug("moveToDCUM");

    std::optional<float> posOpt = tim_->getCurrentPosition();
    if (!posOpt) {
        logger_->warn("Cannot get current position of TIM");
    } else if (std::fabs(posOpt.value() - dcum) <= timMovementThreshold_) {
        logger_->info("Already close to the desired position {}≈{}", posOpt.value(), dcum);
        return true;
    }

    // Commented because TIM12's charger does not work for now
    // std::optional<bool> stopchargOpt = tim_->stopCharging();
    // if (!stopchargOpt) {
    //     logger_->error("Cannot check if the TIM stopped charging");
    //     return false;
    // }
    // if (!stopchargOpt.value()) {
    //     logger_->error("Error stopping the charging");
    //     return false;
    // }

    logger_->info("Moving train to DCUM: {}", dcum);
    std::optional<bool> moveOpt = tim_->moveToTarget(dcum, timVelocity_);
    movingTIM_ = true;
    if (!moveOpt) {
        logger_->error("Error checking if movement of TIM was succesfull");
        return false;
    }
    if (!moveOpt.value()) {
        logger_->error("Movement of TIM not succesfull");
        return false;
    }

    // Wait until it arrives to the target
    std::optional<bool> movOpt = true;
    while (movOpt) {
        std::this_thread::sleep_for(loopFrequency_);
        movOpt = tim_->isMoving();
        if (!movOpt) {
            continue;
        }
        if (!movOpt.value()) {
            break;
        }
    }
    movingTIM_ = false;

    posOpt = tim_->getCurrentPosition();
    if (!posOpt) {
        logger_->warn("Cannot get current position of TIM");
        return false;
    }
    if (std::fabs(posOpt.value() - dcum) > timMovementThreshold_) {
        logger_->warn("It didn't arrive. CurrentPos: {}, DesiredPos: {}", posOpt.value(), dcum);
        return false;
    }

    // Commented because TIM12's charger does not work for now
    // std::optional<bool> startchargOpt = tim_->startCharging();
    // if (!startchargOpt) {
    //     logger_->error("Cannot check if the TIM started charging");
    // } else if (!startchargOpt.value()) {
    //     logger_->error("Error starting the charging");
    // }

    logger_->info("Arrived to position {}", posOpt.value());
    return true;
}

bool TIMMovement::isTIMMoving() {
    logger_->debug("isTIMMoving");
    return movingTIM_.load();
}

void TIMMovement::startObstacleDetection() {
    logger_->debug("startObstacleDetection");
    detectingObstacle_ = true;
    if (!DCUMTrackerThread_.joinable()) {
        DCUMTrackerThread_ = std::thread(&TIMMovement::DCUMTracker, this);
        std::mutex mtx;
        std::unique_lock<std::mutex> lck(mtx);
        cv_.wait_for(lck, loopFrequency_);
    }
}

void TIMMovement::stopObstacleDetection() {
    logger_->debug("stopObstacleDetection");
    detectingObstacle_ = false;
    if (DCUMTrackerThread_.joinable()) {
        DCUMTrackerThread_.join();
    }
}

bool TIMMovement::isEnteringObstacle() {
    logger_->debug("isEnteringObstacle");
    return enteringObstacle_.load();
}

bool TIMMovement::isExitingObstacle() {
    logger_->debug("isExitingObstacle");
    return exitingObstacle_.load();
}

bool TIMMovement::isInObstacle() {
    logger_->debug("isInObstacle");
    return inObstacle_.load();
}

bool TIMMovement::needToRetractForNextObstacle() {
    logger_->debug("needToRetractForNextObstacle");
    return needToRetractInNext_.load();
}

// Private

void TIMMovement::DCUMTracker() {
    logger_->debug("DCUMTracker");
    bool firstIteration = true;
    while (detectingObstacle_) {
        if (!firstIteration) {
            std::this_thread::sleep_for(loopFrequency_);
            cv_.notify_one();
        }
        firstIteration = false;

        int direction = 0;
        std::optional<float> optPosition = tim_->getCurrentPosition();
        std::optional<float> optVelocity = tim_->getCurrentVelocity();
        if (!optPosition || !optVelocity) {
            continue;
        }
        float position = optPosition.value();
        float velocity = optVelocity.value();
        if (velocity > 0) {
            direction = 1;
        } else if (velocity < 0) {
            direction = -1;
        } else {
            enteringObstacle_ = false;
            exitingObstacle_ = false;
            logger_->info("The train is not moving (Velocity 0 m/s)");
            continue;
        }

        std::array<actuators::tim::LHCObstacle, 2> obstacles = tim_->getClosestObstacleAreas();
        crf::actuators::tim::LHCObstacle nextObstacle = obstacles.at(1);
        crf::actuators::tim::LHCObstacle prevObstacle = obstacles.at(0);
        if (nextObstacle.isEmpty() || prevObstacle.isEmpty()) {
            logger_->warn("Obstacles Empty!");
            continue;
        }
        needToRetractInNext_ = nextObstacle.mustRetractDevices();

        // Add offset and check if the two obstacles are too close to each other
        float startNext = 0;
        float endPrev = 0;
        float tailMargin = obstacleMargin_ + obstacleAreaExtension_ + tailOffset_;
        float headMargin = obstacleMargin_ + obstacleAreaExtension_ + headOffset_;
        if (direction == -1) {
            startNext = nextObstacle.endPosition() + tailMargin;
            endPrev = prevObstacle.startPosition() - headMargin;
            if (startNext - endPrev > 0) {
                logger_->warn("Obstacles too close");
                if (nextObstacle.mustRetractDevices()) {
                    logger_->warn("Arm stays retracted");
                    enteringObstacle_ = true;
                    exitingObstacle_ = false;
                    inObstacle_ = false;
                    continue;
                }
                logger_->info("No need to retract arm");
            }
        }
        if (direction == 1) {
            startNext = nextObstacle.startPosition() - headMargin;
            endPrev = prevObstacle.endPosition() + tailMargin;
            if (startNext - endPrev < 0) {
                logger_->warn("Obstacles too close");
                if (nextObstacle.mustRetractDevices()) {
                    logger_->warn("Arm stays retracted");
                    enteringObstacle_ = true;
                    exitingObstacle_ = false;
                    inObstacle_ = false;
                    continue;
                }
                logger_->info("No need to retract arm");
            }
        }

        crf::actuators::tim::LHCObstacle inObs = tim_->getCurrentObstacleArea();
        if (!inObs.isEmpty()) {
            inObstacle_ = true;
            enteringObstacle_ = false;
            exitingObstacle_ = false;
            logger_->warn("In obstacle [{}, {}]", inObs.startPosition(), inObs.endPosition());
            continue;
        }

        if (((position > startNext && direction == 1) || (position < startNext && direction == -1))
            && !enteringObstacle_) {
            enteringObstacle_ = true;
            exitingObstacle_ = false;
            inObstacle_ = false;
            logger_->info("Approaching an obstacle in position {}", startNext);
            tim_->setTargetVelocity(nextObstacle.maximumVelocity());
        } else if (
            ((position + obstacleMargin_ > endPrev && position < endPrev && direction == 1) ||
            (position - obstacleMargin_ < endPrev && position > endPrev && direction == -1)) &&
            !exitingObstacle_) {
            exitingObstacle_ = true;
            enteringObstacle_ = false;
            inObstacle_ = false;
            logger_->info("Exiting from obstacle in position {}", endPrev);
            tim_->setTargetVelocity(timVelocity_);
        }
    }
}

}  // namespace crf::utility::missionutility
