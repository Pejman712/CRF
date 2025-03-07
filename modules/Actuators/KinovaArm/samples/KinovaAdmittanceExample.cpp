/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "EventLogger/EventLogger.hpp"
#include "KinovaArm/KinovaJaco.hpp"
#include "KinovaArm/KinovaApiInterface.hpp"
#include "Types/Types.hpp"
#include "RobotArmKinematics/IRobotArmKinematics.hpp"
#include "KinovaArm/KinovaAdmittanceController.hpp"
#include "RobotArmKinematics/RobotArmKDLKinematics/RobotArmKDLKinematics.hpp"

#include <fstream>
#include <memory>

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::control::robotarmkinematics::IRobotArmKinematics;
using crf::control::robotarmkinematics::RobotArmKDLKinematics;
using crf::actuators::kinovaarm::KinovaAdmittanceController;

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cout << "Too few arguments" << std::endl;
        std::cout << "[1] Kinova CONFIG_FILENAME " << std::endl;
        std::cout << "[2] Admittance CTRL CONFIG_FILENAME " << std::endl;
        return -1;
    }

    crf::utility::logger::EventLogger logger("KinovaAdmittanceExample");
    std::ifstream robotData(argv[1]);
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    auto arm = std::make_shared<crf::actuators::kinovaarm::KinovaJaco>(
            std::make_shared<crf::actuators::kinovaarm::KinovaApiInterface>(), robotJSON);
    logger->info("Going to initialize arm");
    logger->info("arm->initialize(): {}", arm->initialize());
    std::shared_ptr<IRobotArmKinematics> kinematics =
            std::make_shared<RobotArmKDLKinematics>(arm->getConfiguration());

    auto ctrl = std::make_shared<KinovaAdmittanceController>(arm, kinematics, argv[2]);
    ctrl->initialize();

    std::cout << "Click Enter to reset arm to vertical position!";
    std::cin.ignore();
    ctrl->setJointPositions(JointPositions(6));
    std::cin.ignore();

    logger->info("arm->deinitialize(): {}", arm->deinitialize());
    return 0;
}
