/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: David Forkel CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 *
 */
#include <string>
#include <initializer_list>
#include <cmath>
#include <chrono>
#include <memory>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>
#include <bitset>

#include "Robot/EtherCATRobot/EtherCATRobot.hpp"
#include "EtherCATDevices/EtherCATManager.hpp"
#include "EtherCATDevices/EtherCATMotor.hpp"

#include "crf/expected.hpp"

int main(int argc, char** argv) {
    // Sanity Checks
    if (argc != 3) {
        std::cout << "Input arguments not correct! Expected Input:" << std::endl;
        std::cout << "[1] Configuration File" << std::endl;
        std::cout << "[2] EtherCAT port (eno1, eno2, enxa.. , ...)" << std::endl;
        return -1;
    }

    // Get the EtherCAT Robot configuration file path from user input
    std::string portName(argv[2]);
    std::ifstream robotConfigFilePath(argv[1]);
    int IOMapSize = 4096;
    int nSlaves = 4;
    std::shared_ptr<crf::devices::ethercatdevices::EtherCATManager> manager =
        std::make_shared<crf::devices::ethercatdevices::EtherCATManager>(
            portName, nSlaves, IOMapSize, 1000);

    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> motor1 =
        std::make_shared<crf::devices::ethercatdevices::EtherCATMotor>(1, manager);
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> motor2 =
        std::make_shared<crf::devices::ethercatdevices::EtherCATMotor>(2, manager);
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> motor3 =
        std::make_shared<crf::devices::ethercatdevices::EtherCATMotor>(3, manager);
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> motor4 =
        std::make_shared<crf::devices::ethercatdevices::EtherCATMotor>(4, manager);

    std::shared_ptr<crf::actuators::robot::EtherCATRobotConfiguration> config =
        std::make_shared<crf::actuators::robot::EtherCATRobotConfiguration>(
            nlohmann::json::parse(robotConfigFilePath));

    std::vector<std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor>> vectorMotors;
    vectorMotors.push_back(motor1);
    vectorMotors.push_back(motor2);
    vectorMotors.push_back(motor3);
    vectorMotors.push_back(motor4);

    crf::actuators::robot::EtherCATRobot robot(manager, vectorMotors, config);

    // Initialize the EtherCATRobot
    if (!robot.initialize()) {
        std::puts("Error initializing");
        return -1;
    }

    std::puts("Starting Test!!");

    robot.setJointVelocities(false, crf::utility::types::JointVelocities({2, 0, 0, 0}));
    sleep(3);

    robot.setJointVelocities(false, crf::utility::types::JointVelocities({0, 2, 0, 0}));
    sleep(3);

    robot.setJointVelocities(false, crf::utility::types::JointVelocities({0, 0, 2, 0}));
    sleep(3);

    robot.setJointVelocities(false, crf::utility::types::JointVelocities({0, 0, 0, 2}));
    sleep(3);

    robot.setJointVelocities(false, crf::utility::types::JointVelocities({2, 2, 2, 2}));
    sleep(3);

    robot.setJointVelocities(false, crf::utility::types::JointVelocities({-2, -2, -2, -2}));
    sleep(3);

    robot.setJointVelocities(false, crf::utility::types::JointVelocities({0, 0, 0, 0}));
    sleep(3);

    // Read the actual joint positions
    crf::expected<crf::utility::types::JointPositions> qAct = robot.getJointPositions();
    if (!qAct) {
        std::cout << "Error Response Code: " << qAct.get_response() << std::endl;
    } else {
        std::cout << "qAct = " << qAct.value() << std::endl;
    }
    return 0;
}
