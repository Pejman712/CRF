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

#include "EventLogger/EventLogger.hpp"
#include "UniversalRobot/UniversalRobotRTDE.hpp"
#include "UniversalRobot/UniversalRobotRTDEInterface.hpp"

void printVec(std::vector<double> vec, std::string name) {
    std::cout << name << " = ( ";
    for (size_t i = 0; i < vec.size()-1; i++) std::cout << vec[i] << ", ";
    std::cout << vec[vec.size()-1] << " )" << std::endl;
}

int main(int argc, char** argv) {
    // Sanity Checks
    if (argc > 2) {
        std::cout << "Too many arguments" << std::endl;
        std::cout << "[1] Configuration File" << std::endl;
        return -1;
    } else if (argc <= 1) {
        std::cout << "Not Enough input arguments" << std::endl;
        std::cout << "[1] Configuration File" << std::endl;
        return -1;
    }

    // Parse the configuration for the UniversalRobot
    std::ifstream robotData(argv[1]);
    crf::actuators::universalrobot::UniversalRobotRTDE ur{
        std::make_shared<crf::actuators::universalrobot::UniversalRobotRTDEInterface>(),
        nlohmann::json::parse(robotData)};

    // Initialize the UR - Establish RTDE connection
    ur.initialize();

    // Read the actual joint positions
    crf::utility::types::JointPositions qAct = ur.getJointPositions().get();
    std::vector<double> qActVec{qAct[0]*180/M_PI, qAct[1]*180/M_PI, qAct[2]*180/M_PI,
        qAct[3]*180/M_PI, qAct[4]*180/M_PI, qAct[5]*180/M_PI};
    printVec(qActVec, "qAct");

    // Deinitialize the UR
    // - Close RTDE connection and stop the UR script running on the UR control box
    ur.deinitialize();

    return 0;
}
