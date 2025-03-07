/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
*/

#include <memory>
#include <vector>
#include <random>
#include <chrono>
#include <thread>

#include <nlohmann/json.hpp>

#include "MissionManager/ScienceGateway/Actions/FigureCube.hpp"

namespace crf::applications::missionmanager::sciencegateway {

FigureCube::FigureCube(
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> motor):
    motor_(motor),
    logger_("FigureCube") {
    logger_->debug("CTor");
}

FigureCube::~FigureCube() {
    logger_->debug("DTor");
}

bool FigureCube::deinitialize() {
    return motor_->deinitialize();
}

bool FigureCube::initialize() {
    return motor_->initialize();
}

bool FigureCube::randomizeFigure() {
    std::random_device devive;
    std::mt19937 rng(devive());
    std::uniform_int_distribution<std::mt19937::result_type> distribution(1, 4);
    auto rand = distribution(rng);
    if (!motor_->setPosition(
        positionsMap.at(static_cast<Figure>(rand)),
        defaultVelocity_,
        deafultAcceleration_,
        defaultDeceleration_,
        false)) {
        logger_->error("Cube did not reach final position {}", rand);
        return false;
    }
    auto start = std::chrono::high_resolution_clock::now();
    while (getCurrentFigure() != static_cast<Figure>(rand)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto end = std::chrono::high_resolution_clock::now();
        if (start + timeout_ < end) continue;
        logger_->error("Timeout in setting position! The motor did not arrive!");
        return false;
    }
    logger_->info("Arrived to position {}", rand);
    return true;
}

Figure FigureCube::getCurrentFigure() {
    if (isAlmostEqual(motor_->getPosition().value(), positionsMap.at(Figure::First)))
        return Figure::First;
    if (isAlmostEqual(motor_->getPosition().value(), positionsMap.at(Figure::Second)))
        return Figure::Second;
    if (isAlmostEqual(motor_->getPosition().value(), positionsMap.at(Figure::Third)))
        return Figure::Third;
    if (isAlmostEqual(motor_->getPosition().value(), positionsMap.at(Figure::Fourth)))
        return Figure::Fourth;
    return Figure::NotDefined;
}

// Private

bool FigureCube::isAlmostEqual(const double& value1, const double& value2, const double& epsilon) {
    return std::abs(value1 - value2) < epsilon;
}

}  // namespace crf::applications::missionmanager::sciencegateway
