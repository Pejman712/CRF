/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

// Standard libraries
#include <vector>
#include <memory>
#include <iostream>

#include "InverseKinematics/JointLimits/JointLimits.hpp"
#include "InverseKinematics/DesiredJointPositions/DesiredJointPositions.hpp"

#include "Robot/UniversalRobot/UniversalRobot.hpp"
#include "UniversalRobotRTDE/UniversalRobotRTDEInterface.hpp"
#include "Controller/DirectOpenLoopVelocity/DirectOpenLoopVelocity.hpp"
#include "ForwardKinematics/MathExprForwardKinematics/MathExprForwardKinematics.hpp"
#include "InverseKinematics/OptOLIK/OptOLIK.hpp"
#include "MotionController/Teleoperation/Teleoperation.hpp"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::puts("Please input the necessary parameters:");
        std::puts("   1 - Config file path for the UR robot");
        return -1;
    }

    // Create UR Robot
    std::ifstream robotConfigFilePath(argv[1]);

    auto robot = std::make_shared<crf::actuators::robot::UniversalRobot>(
        std::make_shared<crf::communication::universalrobotrtde::UniversalRobotRTDEInterface>(),
        crf::actuators::robot::UniversalRobotConfiguration(
            nlohmann::json::parse(robotConfigFilePath)));

    // Create Forward Kinematics
    auto forwardKinematics = robot->getConfiguration()->getForwardKinematics();

    // Create Inverse Kinematics and Objective Functions
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>>
        vecObjFun;

    // Joint limits
    vecObjFun.push_back(std::make_shared<crf::control::inversekinematics::JointLimits>(
        10.0,  // 10s for the range of the sinusoid
        static_cast<double>(robot->getConfiguration()->getRobotControllerLoopTime().count())/1000,
        1.0,  // Unitary exponential function
        0.1,  // Low proportional gain
        robot->getConfiguration()->getJointLimits().minPosition,
        robot->getConfiguration()->getJointLimits().maxPosition));

    // Desired Joint Position
    vecObjFun.push_back(std::make_shared<crf::control::inversekinematics::DesiredJointPositions>(
        10,  // 10s for the range of the sinusoid
        static_cast<double>(robot->getConfiguration()->getRobotControllerLoopTime().count())/1000,
        1));  // Unitary exponential function

    // Inverse Kinematics
    auto inverseKinematics = std::make_shared<crf::control::inversekinematics::OptOLIK>(
        robot->getConfiguration(),
        std::vector<double>(robot->getConfiguration()->getJointSpaceDoF(), 1));

    auto controller = std::make_shared<crf::control::controller::DirectOpenLoopVelocity>(
        robot->getConfiguration()->getJointSpaceDoF(), inverseKinematics);

    // Motion controller
    auto motion = std::make_shared<crf::control::motioncontroller::Teleoperation>(
        robot, controller);

    if (!motion->initialize()) {
        std::puts("Failed to initialize motion controller");
        return -1;
    }
}
