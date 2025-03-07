/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <vector>
#include <memory>
#include <string>

#include <octomap/OcTree.h>

#include "EventLogger/EventLogger.hpp"
#include "CollisionDetector/SpaceType.hpp"
#include "CollisionDetector/FCLCollisionDetector/FCLCollisionDetector.hpp"

int main(int argc, char** argv) {
    crf::utility::logger::EventLogger logger("FCLCollisionDetectorExample");

    if (argc < 2) {
        printf("Too few arguments\n");
        printf("[1] Directory to file of robot configuration (e.g. charmbotconfig.json)\n");
        return 0;
    }

    std::shared_ptr<crf::actuators::robotbase::RobotBaseConfiguration> robotBaseConfig;
    try {
        robotBaseConfig.reset(new crf::actuators::robotbase::RobotBaseConfiguration());
    } catch (const std::exception& e) {
        logger->error("Failed to create the robot base configuration");
        return -1;
    }

    if (!robotBaseConfig->parse(std::string(argv[1]))) {
        logger->error("Failed to parse the robot base configuration {}", argv[1]);
        return -1;
    }

    // Create standard SE2 State Space (Planning in X, Y, YAW) for RobotBase
    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    stateTypes.push_back(crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    stateTypes.push_back(crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    stateTypes.push_back(crf::navigation::collisiondetector::SpaceType::SO2_STATE_SPACE);

    std::unique_ptr<crf::navigation::collisiondetector::FCLCollisionDetector> fclCollisionChecker;
    try {
        fclCollisionChecker.reset(new crf::navigation::collisiondetector::FCLCollisionDetector(
            robotBaseConfig,
            stateTypes));
    } catch (const std::exception& e) {
        logger->error("Failed to create the collision detector - {}", e.what());
        return -1;
    }

    octomap::OcTree tree(0.05);
    // Creating some occupied cells (x, y, z) in 3D space
    tree.updateNode(octomap::point3d(2.0, -2.0, 0.1), true);
    tree.updateNode(octomap::point3d(2.0, 2.0, 0.1), true);
    tree.updateNode(octomap::point3d(5.0, .0, .0), true);
    tree.updateNode(octomap::point3d(.0, .0, 1.5), true);
    tree.updateNode(octomap::point3d(.0, -0.38, .0), true);

    if (!fclCollisionChecker->updateMap(tree)) {
        logger->error("Failed to update Map");
        return -1;
    }

    std::vector<float> stateRot = {.0, .0 , M_PI/2};
    std::cout << "Result for Check Initial State with Rotation = " <<
            fclCollisionChecker->checkState(stateRot) << std::endl;

    std::vector<float> state1 = {.0, .0 , .0};
    for (int xCoord = 1; xCoord < 20; xCoord++) {
        float step = 0.1;
        std::vector<float> state2 = {xCoord*step, xCoord*step , .0};
        // Boolean output (0 equals invalid --> collision, 1 equals valid --> no collision)
        std::cout << "Result for Check Initial State = " <<
            fclCollisionChecker->checkState(state1) << std::endl;
        std::cout << "Result for Check Final State = " <<
            fclCollisionChecker->checkState(state2) << std::endl;
        bool valid = fclCollisionChecker->checkMotion(state1, state2);
        std::cout << "Result for Check Motion = " <<
            valid << std::endl;
        if (fclCollisionChecker->clearance(state1)) {
            std::cout << "Result for Initial Clearance = " <<
                fclCollisionChecker->clearance(state1).get() << std::endl;
        }
        if (fclCollisionChecker->clearance(state2)) {
            std::cout << "Result for Final Clearance = " <<
                fclCollisionChecker->clearance(state2).get() << std::endl;
        }
        state1 = state2;
        if (!valid) {
            std::cout << "Collision!!" << std::endl;
        }
    }

    return 0;
}
