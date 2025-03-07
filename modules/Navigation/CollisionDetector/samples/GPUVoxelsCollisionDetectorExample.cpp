/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <cmath>
#include <memory>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "CollisionDetector/SpaceType.hpp"
#include "CollisionDetector/GPUVoxelsCollisionDetector/GPUVoxelsCollisionDetector.hpp"

int main(int argc, char** argv) {
    crf::utility::logger::EventLogger logger("GPUVoxelsCollisionDetectorExample");

    if (argc != 4) {
        logger->error("The number of arguments is not correct");
        return -1;
    }
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig;
    try {
        robotArmConfig.reset(new crf::robots::robotarm::RobotArmConfiguration());
    } catch (const std::exception& e) {
        logger->error("Failed to create the robot arm configuration");
        return -1;
    }

    std::ifstream robotData(argv[1]);
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    if (!robotArmConfig->parse(robotJSON)) {
        logger->error("Failed to parse the robot arm configuration {}", argv[1]);
        return -1;
    }

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (std::size_t i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }
    std::unique_ptr<crf::navigation::collisiondetector::GPUVoxelsCollisionDetector> colliDetector;
    try {
        colliDetector.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
            robotArmConfig,
            stateTypes,
            argv[2],
            argv[3]));
    } catch (const std::exception& e) {
        logger->error("Failed to create the collision detector - {}", e.what());
        return -1;
    }

    logger->info("NEXT - Check state");
    std::cin.ignore();
    for (float i = 0; i < 3.14f; i = i+0.1f) {
        std::vector<float> state = {i, i, i, i, i, i};
        logger->info("checkState {} - {}", colliDetector->checkState(state), i);
        std::cin.ignore();
    }

    logger->info("NEXT - Check motion");
    std::cin.ignore();
    for (float i = 0; i < 3.14f; i = i+0.05f) {
        std::vector<float> start = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        std::vector<float> goal = {i, i, i, i, i, i};
        logger->info("checkMotion", colliDetector->checkMotion(start, goal));
    }

    colliDetector->displayPerformanceResults();
    return 0;
}
