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

    // Read the actual joint positions
    crf::utility::types::JointPositions q = ur.getJointPositions().value();
    std::cout << "Initial Joint Angles: q = " << q << std::endl;

    // Directly move the servo motor of a joint to a position with a small incremental step by
    // setting isSmoothTrajectory = true. Hence, there will be no internal trajectory planner used
    // and positions need to be smooth.
    const int moveJoint = 3;  // Joint to be moved
    bool isSmoothTrajectory = true;  // parameter to tell robot it will receive a smooth trajectory
    q[moveJoint-1] += 0.001;  // Increase joint moveJoint by 0.001 rad

    std::cout << "Joint Position Control of Smooth Trajectory ..." << std::endl;
    ur.setJointPositions(isSmoothTrajectory, q);

    // Read the actual joint positions
    q = ur.getJointPositions().value();
    std::cout << "Joint Angles: q = " << q << std::endl;

    // Move the servo motor to a desired position independet from the current position by setting
    // isSmoothTrajectory = false. Hence, the robot control box will plan a trajectory using either
    // the default profileJointVelocities = 1.05 rad/s and profileJointAccelerations = 1.4 rad/s^2
    // or the user can set these parameters with setProfileJointVelocities() and
    // setProfileJointAccelerations().
    isSmoothTrajectory = false;
    q[moveJoint-1] = 3.1415/2;

    std::cout << "Joint Position Control of Non Smooth Trajectory ..." << std::endl;
    ur.setJointPositions(isSmoothTrajectory, q);

    // Read the actual joint positions
    q = ur.getJointPositions().value();
    std::cout << "Joint Angles: q = " << q << std::endl;

    ur.deinitialize();  // stop the connection and the script running on the UR control box

    return 0;
}
