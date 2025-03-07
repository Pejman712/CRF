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
#include "Robot/UniversalRobot/UniversalRobot.hpp"
#include "UniversalRobotRTDE/UniversalRobotRTDEInterface.hpp"

using namespace std::chrono; // NOLINT

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
    std::ifstream robotConfigFilePath(argv[1]);
    crf::actuators::robot::UniversalRobot ur{
        std::make_shared<crf::communication::universalrobotrtde::UniversalRobotRTDEInterface>(),
        crf::actuators::robot::UniversalRobotConfiguration(
            nlohmann::json::parse(robotConfigFilePath))};

    ur.initialize();  // Establish the RTDE connection, send the UR script and start the script

    // Read the actual task position
    crf::utility::types::TaskPose z = ur.getTaskPose().value();
    std::cout << "Initial Task Angles: z = "
              << crf::utility::types::TaskPose(z.getPosition(), z.getAngleAxis()) << std::endl;

    // Move the robot to a desired task position independet from the current position by setting
    // isSmoothTrajectory = false. Hence, the robot control box will plan a trajectory using either
    // the default profileJointVelocities = 1.05 rad/s and profileJointAccelerations = 1.4 rad/s^2
    // or the user can set these parameters with setProfileJointVelocities() and
    // setProfileJointAccelerations().
    bool isSmoothTrajectory = false;
    // z = crf::utility::types::TaskPose({z(0), z(1), z(2)+0.1, z(3), z(4), z(5)});
    z.setPosition(z.getPosition() + Eigen::Vector3d({0, 0, 0.1}));
    std::cout << "Desired Task Angles: z = "
              << crf::utility::types::TaskPose(z.getPosition(), z.getAngleAxis()) << std::endl;

    std::cout << "Task Position Control of Non Smooth Trajectory ..." << std::endl;
    ur.setTaskPose(isSmoothTrajectory, z);

    // Read the actual joint positions
    z = ur.getTaskPose().value();
    std::cout << "Actual Task Position: z = "
              << crf::utility::types::TaskPose(z.getPosition(), z.getAngleAxis()) << std::endl;

    ur.deinitialize();  // stop the connection and the script running on the UR control box

    return 0;
}
