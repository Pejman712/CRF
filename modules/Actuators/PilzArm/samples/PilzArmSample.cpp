/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author:       Thomas Breant CERN EN/SMM/MRO 2020
 * Contributor:  Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
*/

#include <fstream>
#include <iostream>
#include <memory>

#include "CANSocket/CANSocket.hpp"
#include "Types/Types.hpp"
#include "PilzArm/PilzArm.hpp"
#include "Gripper/SchunkGripperCANOpen/SchunkGripperCANOpen.hpp"

using crf::communication::cansocket::CANSocket;

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cout << "Too few arguments" << std::endl;
        std::cout << "[1] CAN interface " << std::endl;
        std::cout << "[2] CONFIG_FILENAME " << std::endl;
        return -1;
    }

    std::shared_ptr<CANSocket> socket =
        std::make_shared<CANSocket>(argv[1]);

    auto ctx = std::make_shared<crf::devices::canopendevices::CANOpenContext>(socket);

    if (!ctx->initialize()) {
        std::puts("Could not initialize context");
        return -1;
    }

    std::shared_ptr<crf::actuators::gripper::SchunkGripperCANOpen> gripper = nullptr;

    std::ifstream robotData(argv[2]);
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    std::shared_ptr<crf::actuators::pilzarm::PilzArm> arm =
        std::make_shared<crf::actuators::pilzarm::PilzArm>(
            socket,
            robotJSON,
            gripper,
            ctx);

    if (!arm->initialize()) {
        std::puts("Could not initialize arm");
        return -1;
    }

    boost::optional<crf::utility::types::JointPositions> joinOpt = arm->getJointPositions();
    if (!joinOpt) {
        std::cout << "Error reading from arm" << std::endl;\
        return -1;
    }
    std::cout << "Joint Position: " << joinOpt.get() << std::endl;
    return 0;
}
