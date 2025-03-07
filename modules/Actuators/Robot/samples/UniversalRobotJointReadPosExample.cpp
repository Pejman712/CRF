/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <cmath>
#include <chrono>
#include <memory>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>

#include "Robot/UniversalRobot/UniversalRobot.hpp"
#include "UniversalRobotRTDE/UniversalRobotRTDEInterface.hpp"

int main(int argc, char** argv) {
    // Sanity Checks
    if (argc > 2) {
        std::cout << "Too many arguments! Expected Input:" << std::endl;
        std::cout << "[1] Configuration File" << std::endl;
        return -1;
    } else if (argc <= 1) {
        std::cout << "Not Enough input arguments! Expected Input:" << std::endl;
        std::cout << "[1] Configuration File" << std::endl;
        return -1;
    }

    // Get the Universal Robot configuration file path from user input
    std::ifstream robotConfigFilePath(argv[1]);

    // Construct an UniversalRobotRTDE object
    crf::actuators::robot::UniversalRobot ur{
        std::make_shared<crf::communication::universalrobotrtde::UniversalRobotRTDEInterface>(),
        crf::actuators::robot::UniversalRobotConfiguration(
            nlohmann::json::parse(robotConfigFilePath))};

    // Initialize the UR - Establish RTDE connection
    ur.initialize();

    // Read the actual joint positions
    crf::expected<crf::utility::types::JointPositions> qAct = ur.getJointPositions();

    // Check if the call was successful
    if (!qAct) {
        std::cout << "Error Response Code: " << qAct.get_response() << std::endl;
    } else {
        // Print the actual joint positions
        std::cout << "qAct = " << qAct.value() << std::endl;
    }

    // Deinitialize the UR - Close RTDE connection and
    // stop the script running on the UR control box
    ur.deinitialize();

    return 0;
}
