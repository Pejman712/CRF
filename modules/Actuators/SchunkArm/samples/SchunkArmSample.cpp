/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "CANSocket/CANSocket.hpp"
#include "Types/Types.hpp"
#include "SchunkArm/SchunkArm.hpp"
#include "SchunkArm/SchunkGripper.hpp"

using crf::actuators::schunkarm::SchunkArm;
using crf::actuators::schunkarm::SchunkGripper;
using crf::actuators::gripper::IGripper;

void moveTarget(std::shared_ptr<SchunkGripper> gripper,
                float target) {
    gripper->setVelocity(target);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
}

void moveTargetEnd(std::shared_ptr<SchunkGripper> gripper,
    IGripper::GripperState state) {
    std::cout << gripper->setPosition(state) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    while (std::fabs(gripper->getPosition().get() - state) > 0.3) {
        if (gripper->isGrasping()) {
            return;
        }
        std::cout << "pos error : " << std::fabs(gripper->getPosition().get() - state)
                    << " grasping : " << gripper->isGrasping() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cout << "Too few arguments" << std::endl;
        std::cout << "[1] CAN interface " << std::endl;
        std::cout << "[2] CONFIG_FILENAME " << std::endl;
        return -1;
    }

    auto socket = std::make_shared<crf::communication::cansocket::CANSocket>(argv[1]);
    auto gripper = std::make_shared<SchunkGripper>(socket);

    std::ifstream robotData(argv[2]);

    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    auto arm = std::make_shared<SchunkArm>(socket, robotJSON, gripper);

    arm->initialize();
    int numberOfJoints = 6;

    // Move the arm to position
    crf::utility::types::JointPositions goal({1, 1, 1, 1, 1, 1});
    arm->setJointPositions(goal);
    // Check when it reaches
    bool poseReached = false;
    while (!poseReached) {
        auto jp = arm->getJointPositions().get();
        int jointsReached = 0;
        for (int i = 0; i < numberOfJoints; ++i) {
            float error = fabs(jp[i] - goal[i]);
            if (error < 0.01) {
                jointsReached++;
            }
        }
        if (jointsReached == numberOfJoints) {
            poseReached = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "We reached the goal position" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(4));

    // Check Velocity Movement
    crf::utility::types::JointVelocities jv({-0.1, -0.1, -0.1, -0.1, -0.1, -0.1});
    arm->setJointVelocities(jv);
    // Stop when it reaches 0
    crf::utility::types::JointPositions zeroPos({0, 0, 0, 0, 0, 0});
    poseReached = false;
    while (!poseReached) {
        auto jp = arm->getJointPositions().get();
        int jointsReached = 0;
        for (int i = 0; i < numberOfJoints; ++i) {
            float error = fabs(jp[i] - zeroPos[i]);
            if (error < 0.01) {
                jointsReached++;
            }
        }
        if (jointsReached == numberOfJoints) {
            poseReached = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "We reached the ZERO position" << std::endl;
    crf::utility::types::JointVelocities zeroVelocity({0, 0, 0, 0, 0, 0});
    arm->setJointVelocities(zeroVelocity);
    std::this_thread::sleep_for(std::chrono::seconds(4));

    std::cout << "Seting gripper to middle" << std::endl;
    gripper->setPosition(50);
    std::this_thread::sleep_for(std::chrono::seconds(4));

    std::cout << "Opening" << std::endl;
    gripper->setPosition(IGripper::Gripper_Open);
    std::this_thread::sleep_for(std::chrono::seconds(4));

    std::cout << "Closing" << std::endl;
    gripper->setPosition(IGripper::Gripper_Closed);
    std::this_thread::sleep_for(std::chrono::seconds(4));

    std::cout << "Opening fast" << std::endl;
    gripper->setVelocity(-100);
    std::this_thread::sleep_for(std::chrono::seconds(4));

    std::cout << "Closing very slow" << std::endl;
    gripper->setVelocity(5);
    std::this_thread::sleep_for(std::chrono::seconds(4));

    return 0;
}
