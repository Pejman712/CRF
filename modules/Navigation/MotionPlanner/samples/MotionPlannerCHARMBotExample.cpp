/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <memory>
#include <vector>

#include <nlohmann/json.hpp>

#include "CANSocket/CANSocket.hpp"
#include "CHARMBot/CHARMBot.hpp"
#include "TrajectoryGenerator/TaskTimeOptimalTrajectory.hpp"
#include "CollisionDetector/SpaceType.hpp"
#include "CollisionDetector/FCLCollisionDetector.hpp"
#include "CollisionDetector/ICollisionDetector.hpp"
#include "PathPlanner/OptimizerMethod.hpp"
#include "PathPlanner/PathPlannerMethod.hpp"
#include "PathPlanner/StateSpaceDefinition.hpp"
#include "PathPlanner/CompoundStatesSpacePlanner.hpp"
#include "PathPlanner/CompoundStatesValidatorForCollisionAvoidance.hpp"
#include "RobotBaseControllers/RobotBaseControllerFactory.hpp"
#include "MotionPlanner/MotionPlannerLaser.hpp"
#include "Laser/Hokuyo/HokuyoLaser.hpp"

int main(int argc, char* argv[]) {
    if (argc < 5) {
        printf("Too few arguments\n");
        printf("[1] CAN interface (e.g. can1)\n");
        printf("[2] Directory to robot configuration file\n");
        printf("[3] Directory to planner configuration file\n");
        printf("[4] Hokuyo IP Adress\n");
        return 0;
    }
    printf("Starting CHARMbot\n");
    std::shared_ptr<CANSocket> socket = std::make_shared<CANSocket>(argv[1]);

    std::ifstream configFile(argv[2]);
    if ((configFile.rdstate() & std::ifstream::failbit) != 0) {
        std::cout << "Provided configuration file does not exists" << std::endl;
        return -1;
    }

    auto bot = std::make_shared<crf::robots::robotbase::CHARMBot>(
        socket, nlohmann::json::parse(configFile));
    if (!bot->initialize()) {
        std::cout << "Could not initialize the CHARMBot" << std::endl;
        return -1;
    }

    crf::applications::robotbasecontroller::RobotBaseControllerFactory factory(bot);
    auto controller = factory.create(
        crf::applications::robotbasecontroller::ControlMode::Position);
    auto robotConfig = bot->getConfiguration();
    nlohmann::json jsonPlannerConfig_;
     std::vector<crf::algorithms::pathplanner::StateSpaceDefinition> definitions;
    try {
        std::ifstream config(argv[3]);
        config >> jsonPlannerConfig_;
    } catch (std::exception& e) {
        throw std::runtime_error("Planner configuration file not valid");
    }

    int plannedDimensions_ = jsonPlannerConfig_.at("Dimensions").get<int>();
    try {
        for (int i = 0; i < plannedDimensions_; i++) {
            std::string state = "State" + std::to_string(i);
            crf::algorithms::pathplanner::StateSpaceDefinition stateSpace;
            int Type = jsonPlannerConfig_.at("StateSpaces").at(state).at("Type").get<int>();
            stateSpace.type = (crf::algorithms::collisiondetector::SpaceType)Type;
            if (jsonPlannerConfig_.at("StateSpaces").at(state).count("lowerBound") > 0) {
                stateSpace.lowerBound = jsonPlannerConfig_.at("StateSpaces").
                    at(state).at("lowerBound").get<float>();
            }
            if (jsonPlannerConfig_.at("StateSpaces").at(state).count("upperBound") > 0) {
                stateSpace.upperBound = jsonPlannerConfig_.at("StateSpaces").
                    at(state).at("upperBound").get<float>();
            }
            if (jsonPlannerConfig_.at("StateSpaces").at(state).count("weight") > 0) {
                stateSpace.weight = jsonPlannerConfig_.at("StateSpaces").
                    at(state).at("weight").get<float>();
            }
            definitions.push_back(stateSpace);
        }
    } catch (const std::exception& e) {
        std::cout << e.what();
    }

    std::vector<crf::algorithms::collisiondetector::SpaceType> typeDef;
    for (int typeID = 0; typeID < plannedDimensions_; typeID++) {
        typeDef.push_back(definitions[typeID].type);
  }

    std::shared_ptr<crf::algorithms::collisiondetector::ICollisionDetector> collisiondetector;
    try {
        collisiondetector = std::make_shared<
        crf::algorithms::collisiondetector::FCLCollisionDetector>(robotConfig, typeDef);
    } catch (const std::exception& e) {
        std::cout << e.what();
    }
    std::shared_ptr<
    crf::algorithms::pathplanner::CompoundStatesValidatorForCollisionAvoidance> statesValidator;
    try {
        statesValidator =
        std::make_shared<crf::algorithms::pathplanner::
            CompoundStatesValidatorForCollisionAvoidance>(collisiondetector, typeDef, 0.1);
    } catch (const std::exception& e) {
        std::cout << e.what();
    }
    std::shared_ptr<crf::algorithms::pathplanner::IPathPlanner> planner;
    try {
        int optimizerMethod = jsonPlannerConfig_.at("Optimizer").get<int>();
        crf::algorithms::pathplanner::OptimizerMethod optiMethod =
            (crf::algorithms::pathplanner::OptimizerMethod)optimizerMethod;
        int planMethod = jsonPlannerConfig_.at("Planner").get<int>();
        crf::algorithms::pathplanner::PathPlannerMethod plannerMethod =
            (crf::algorithms::pathplanner::PathPlannerMethod)planMethod;
        float maxTime = jsonPlannerConfig_.at("MaxTime").get<float>();
        float maxSimplificationTime  = jsonPlannerConfig_.at("MaxTime").get<float>();
        planner =
        std::make_shared<crf::algorithms::pathplanner::CompoundStatesSpacePlanner>(plannerMethod,
            optiMethod,
            maxTime, maxSimplificationTime,
            definitions,
            statesValidator);
    } catch (const std::exception& e) {
        std::cout << e.what();
    }
    auto laser = std::make_shared< crf::sensors::laser::HokuyoLaser>(crf::sensors::laser::HokuyoConnectionType::ETHERNET, argv[4], 10940); // NOLINT
    float timeStep = 1;
    float maxDeviation = 0.1;

    auto timeOptTrajec =
    std::make_shared<crf::algorithms::trajectorygenerator::TaskTimeOptimalTrajectory>(
        robotConfig->getTaskLimits().maximumVelocity,
        robotConfig->getTaskLimits().maximumAcceleration,
        timeStep,
        maxDeviation);

    crf::utility::types::TaskPose laserPose{-0.335, .0 , .0, .0, .0, .0};

    crf::applications::motionplanner::MotionPlannerLaser motionplanner(
        controller, planner, collisiondetector, timeOptTrajec, robotConfig, laser, laserPose, 0.05);
    motionplanner.initialize();
    crf::utility::types::TaskPose pos({-2.0, .0, .0, .0, .0, .0});
    motionplanner.plan(pos);
    motionplanner.deinitialize();
    bot->deinitialize();

    return 0;
}
