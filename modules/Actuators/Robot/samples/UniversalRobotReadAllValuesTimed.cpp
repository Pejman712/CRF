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

    auto start = std::chrono::high_resolution_clock::now();
    crf::expected<crf::utility::types::JointPositions> qAct = ur.getJointPositions();
    auto end1 = std::chrono::high_resolution_clock::now();

    crf::expected<crf::utility::types::JointVelocities> qdAct = ur.getJointVelocities();
    auto end2 = std::chrono::high_resolution_clock::now();

    crf::expected<crf::utility::types::JointAccelerations> qddAct = ur.getJointAccelerations();
    auto end3 = std::chrono::high_resolution_clock::now();

    crf::expected<crf::utility::types::JointForceTorques> t = ur.getJointForceTorques();
    auto end4 = std::chrono::high_resolution_clock::now();

    int64_t timePos = std::chrono::duration_cast<std::chrono::microseconds>(end1 - start).count();
    int64_t timeVel = std::chrono::duration_cast<std::chrono::microseconds>(end2 - end1).count();
    int64_t timeAcc = std::chrono::duration_cast<std::chrono::microseconds>(end3 - end2).count();
    int64_t timeTor = std::chrono::duration_cast<std::chrono::microseconds>(end4 - end3).count();

    std::cout << "Summary of times: " << std::endl;
    std::cout << " - getJointPositions() = " << timePos << " us" << std::endl;
    std::cout << " - getJointVelocities() = " << timeVel << " us" << std::endl;
    std::cout << " - getJointAccelerations() = " << timeAcc << " us" << std::endl;
    std::cout << " - getJointForceTorques() = " << timeTor << " us" << std::endl;

    return 0;
}
