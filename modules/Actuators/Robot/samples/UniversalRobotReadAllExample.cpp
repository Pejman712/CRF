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

    crf::expected<crf::utility::types::JointPositions> qAct = ur.getJointPositions();
    if (!qAct) {
        std::cout << "Error Response Code: " << qAct.get_response() << std::endl;
    } else {
        std::cout << "qAct = " << qAct.value() << std::endl;
    }

    crf::expected<crf::utility::types::JointVelocities> qdAct = ur.getJointVelocities();
    if (!qdAct) {
        std::cout << "Error Response Code: " << qdAct.get_response() << std::endl;
    } else {
        std::cout << "qdAct = " << qdAct.value() << std::endl;
    }

    crf::expected<crf::utility::types::JointAccelerations> qddAct = ur.getJointAccelerations();
    if (!qddAct) {
        std::cout << "Error Response Code: " << qddAct.get_response() << std::endl;
    } else {
        std::cout << "qddAct = " << qddAct.value() << std::endl;
    }

    crf::expected<crf::utility::types::JointForceTorques> t = ur.getJointForceTorques();
    if (!t) {
        std::cout << "Error Response Code: " << t.get_response() << std::endl;
    } else {
        std::cout << "joint torque = " << t.value() << std::endl;
    }

    crf::expected<crf::utility::types::TaskPose> zAct = ur.getTaskPose();
    if (!zAct) {
        std::cout << "Error Response Code: " << zAct.get_response() << std::endl;
    } else {
        Eigen::Vector3d zActPosition = zAct.value().getPosition();
        Eigen::AngleAxisd zActAngleAxis = zAct.value().getAngleAxis();
        std::cout << "zActPosition = " << zActPosition(0) << ", " <<  zActPosition(1)
                  << ", " << zActPosition(2)
                  << std::endl
                  << "zActAngleAxis = " << zActAngleAxis.angle() << ", "
                  <<  zActAngleAxis.axis()(0) << ", "
                  <<  zActAngleAxis.axis()(1)
                  << ", "      << zActAngleAxis.axis()(2) << std::endl;

        /* Or request single entries with
         * zAct.value().getPosition()([0-2])
         * zAct.value().getAngleAxis().angle()
         * zAct.value().getAngleAxis().axis()([0-2])
         */
    }

    crf::expected<crf::utility::types::TaskVelocity> zdAct = ur.getTaskVelocity();
    if (!zdAct) {
        std::cout << "Error Response Code: " << zdAct.get_response() << std::endl;
    } else {
        std::cout << "zdAct = " << zdAct.value() << std::endl;
    }

    crf::expected<crf::utility::types::TaskAcceleration> zddAct = ur.getTaskAcceleration();
    if (!zddAct) {
        std::cout << "Error Response Code: " << zddAct.get_response() << std::endl;
    } else {
        std::cout << "zddAct = " << zddAct.value() << std::endl;
    }

    crf::expected<crf::utility::types::TaskForceTorque> wrench = ur.getTaskForceTorque();
    if (!wrench) {
        std::cout << "Error Response Code: " << wrench.get_response() << std::endl;
    } else {
        std::cout << "wrench = " << wrench.value() << std::endl;
    }

    crf::expected<bool> success = ur.setGravity({9.81, 0, 0});
    if (!success) {
        std::cout << "Error Response Code: " << success.get_response() << std::endl;
    } else {
        std::cout << "setGravity() response = " << success.value() << std::endl;
    }

    std::set<crf::Code> status = ur.robotStatus();

    // Deinitialize the UR. Close RTDE connection and stop the script running on the UR control box
    ur.deinitialize();

    return 0;
}
