/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <cmath>
#include <chrono>
#include <memory>
#include <vector>
#include <iostream>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "KinovaArm/KinovaJaco.hpp"
#include "KinovaArm/KinovaApiInterface.hpp"
#include "RobotArmControllersDeprecated/RobotArmSlaveController.hpp"
#include "RobotArmKinematics/RobotArmKDLKinematics/RobotArmKDLKinematics.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "ClosedLoopController/PIDController.hpp"
#include "Types/Types.hpp"
#include "SchunkArm/SchunkArm.hpp"
#include "CANSocket/CANSocket.hpp"

using crf::robots::robotarmkinematics::RobotArmKDLKinematics;
using crf::robots::robotarm::RobotArmConfiguration;
using crf::algorithms::closedloopcontroller::PIDController;
using crf::applications::robotarmcontroller::RobotArmSlaveController;
using crf::utility::types::JointPositions;
using crf::utility::types::areAlmostEqual;

int main(int argc, char** argv) {
    crf::utility::logger::EventLogger logger("SlaveControllerExample");


    std::string file = "/home/robotronics/CERNRoboticFramework/"
                               "modules/Robots/SchunkArm/configuration/SchunkLWA4P.json";
    std::ifstream robotData(file);
    nlohmann::json configSchunk = nlohmann::json::parse(robotData);
    auto can_socket = std::make_shared<CANSocket>("can0");
    auto arm = std::make_shared<crf::robots::schunkarm::SchunkArm>(
            can_socket, configSchunk);

//    std::string configKinova = "/home/robotronics/CERNRoboticFramework/"
//                               "modules/Robots/KinovaArm/config/KinovaJaco2CW.json";
//    auto arm = std::make_shared<crf::robots::kinovaarm::KinovaJaco>(
//            std::make_shared<crf::robots::kinovaarm::KinovaApiInterface>(), configKinova);

    logger->info("Going to initialize arm");
    logger->info("arm.initialize(): {}", arm->initialize());

    auto robotArmConfiguration = std::make_shared<RobotArmConfiguration>();
    robotArmConfiguration->parse(configSchunk);
    auto kinematics = std::make_shared<RobotArmKDLKinematics>(robotArmConfiguration);
    auto closed_loop_controller = std::make_shared<PIDController>(
            std::vector<float>(6, .9),
            std::vector<float>(6, 0),
            std::vector<float>(6, 0));

    auto ctrl = std::make_shared<RobotArmSlaveController>(arm, kinematics, closed_loop_controller);

    ctrl->initialize();

    for (int i=0; i < 3; i++) {
        std::cout << "Moving to PI" << std::endl;
        JointPositions jp({M_PI, M_PI, M_PI, M_PI, M_PI, M_PI});
        jp *= 0.25;
        std::cout << "Setting pi/2 " << ctrl->setJointPositions(jp) << std::endl;
        while (!areAlmostEqual(ctrl->getJointPositions(), jp)) {
            std::this_thread::sleep_for(
                    std::chrono::milliseconds(100));
        }

        std::cout << "Moving to -PI" << std::endl;
        ctrl->setJointPositions(jp*-1);
        while (!areAlmostEqual(ctrl->getJointPositions(), jp*-1)) {
            std::this_thread::sleep_for(
                    std::chrono::milliseconds(100));
        }

//        std::cout << "Moving to taskPos" << std::endl;
//        crf::utility::types::TaskPose cp({0, 0.4, 0.6, 0, 0, 0});
//        ctrl->setTaskPose(cp);
//        bool moving = true;
//        while(moving){
//            auto taskPos = ctrl->getTaskPose().rotationAndTranslationMatrix();
//            if (fabs(taskPos[11] - 0.5) < 0.01){moving = false;};
//            std::this_thread::sleep_for(
//                    std::chrono::milliseconds(100));
//        }
    }

    logger->info("arm.deinitialize(): {}", arm->deinitialize());
    return 0;
}
