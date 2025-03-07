/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <iostream>
#include <fstream>

#include "EventLogger/EventLogger.hpp"
#include "CANSocket/CANSocket.hpp"
#include "SchunkPowerCube/SchunkPowerCube.hpp"
#include "Types/JointPositions.hpp"
#include "Types/JointVelocities.hpp"
#include "Types/JointTypes/JointForceTorques.hpp"

/*
 * Note: The baud rate of the CAN port must be set to 250000 Bd
 */

int main(int argc, char* argv[]) {
    crf::utility::logger::EventLogger logger("SchunkPowerCubeExample");
    if (argc != 3) {
        logger->error("Wrong number of arguments");
        logger->error("[1] Configuration File ");
        logger->error("[2] CAN Socket ");
        return -1;
    }

    auto socket = std::make_shared<CANSocket>(argv[2]);

    std::ifstream robotData(argv[1]);
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);

    crf::robots::schunkpowercube::SchunkPowerCube cube(socket, robotJSON);
    logger->info("cube.initialize(): {}", cube.initialize());
    sleep(1);

    auto jointPositions = cube.getJointPositions();
    if (jointPositions) {
        logger->info("Position: {}", jointPositions.get());
    }
    auto jointVelocities = cube.getJointVelocities();
    if (jointVelocities) {
        logger->info("Velocity: {}", jointVelocities.get());
    }
    auto jointForceTorques = cube.getJointForceTorques();
    if (jointForceTorques) {
        logger->info("Torque: {}", jointForceTorques.get());
    }

    crf::utility::types::JointVelocities goalVelocity({0.05f, 0.05f, 0.05f, 0.05f});
    logger->info("cube.setJointVelocities(): {}", cube.setJointVelocities(goalVelocity));
    std::cin.ignore();

    jointPositions = cube.getJointPositions();
    if (jointPositions) {
        logger->info("Position: {}", jointPositions.get());
    }
    jointVelocities = cube.getJointVelocities();
    if (jointVelocities) {
        logger->info("Velocity: {}", jointVelocities.get());
    }
    JointForceTorques = cube.getJointForceTorques();
    if (jointForceTorques) {
        logger->info("Torque: {}", jointForceTorques.get());
    }

    logger->info("cube.deinitialize(): {}", cube.deinitialize());
    return 0;
}
