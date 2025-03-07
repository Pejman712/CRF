/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <set>
#include <thread>
#include <math.h>

#include "EventLogger/EventLogger.hpp"
#include "Haption/HaptionRaptorAPI/HaptionRaptorAPI.hpp"
#include "Robot/Virtuose6DTAO/Virtuose6DTAOConfiguration.hpp"
#include "Robot/Virtuose6DTAO/Virtuose6DTAO.hpp"
#include "crf/ResponseDefinitions.hpp"
#include "Types/Types.hpp"

int main(int argc, char** argv) {
    // Sanity Checks
    if (argc > 3) {
        std::cout << "Too many arguments:" << std::endl;
        std::cout << "  [1] Robot Configuration File." << std::endl;
        std::cout << "  [2] Haption Parameters File." << std::endl;
        return -1;
    } else if (argc <= 2) {
        std::cout << "Not Enough input arguments:" << std::endl;
        std::cout << "  [1] Configuration File." << std::endl;
        std::cout << "  [2] Haption Parameters File." << std::endl;
        return -1;
    }

    // Parse the configuration file
    std::ifstream filePath(argv[1]);
    crf::actuators::robot::Virtuose6DTAOConfiguration config(nlohmann::json::parse(filePath));

    // Create RaptorAPI
    unsigned int localPort = 12120;
    unsigned int remotePort = 5000;
    auto raptor = std::make_shared<crf::devices::haption::HaptionRaptorAPI>(
        localPort, config.getIPAddress(), remotePort, argv[2]);

    crf::actuators::robot::Virtuose6DTAO robot(raptor, config);

    if (!robot.initialize()) {
        std::cout << "Failed to initialize." << std::endl;
        return -1;
    }

    while (true) {
        std::set<crf::Code> status = robot.robotStatus();

        crf::expected<crf::utility::types::TaskPose> position = robot.getTaskPose();
        if (!position) {
            std::cout << "Failed to get cartesian position." << std::endl;
            continue;
        }

        std::cout << "Cartesian Position: " << position.value() << std::endl;
        std::cout << "Cartesian Velocity: " << robot.getTaskVelocity().value() << std::endl;
        std::cout << "Cartesian Torque: " << robot.getTaskForceTorque().value() << std::endl;

        std::cout << "Joint Position: " << robot.getJointPositions().value() << std::endl;
        std::cout << "Joint Velocity: " << robot.getJointVelocities().value() << std::endl;
        std::cout << "Joint Torque: " << robot.getJointForceTorques().value() << std::endl;

        // Simulate a virtual plane at Z = -0.1 m
        crf::utility::types::TaskForceTorque torque;
        double z = position.value().getPosition()[2];
        if (status.count(crf::Code::PoweredOn) >= 1 && z < -0.1) {
            // Apply pure stiffness of 1000 N/m
            torque[2] = -(z + 0.1)* 1000;
        }

        if (robot.setTaskForceTorque(false, torque)) {
            std::cout << "Failed to set cartesian torque." << std::endl;
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    robot.deinitialize();
    return 0;
}
