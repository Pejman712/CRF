/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alessandro Vascelli CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <thread>
#include <chrono>

#include "SerialCommunication/SerialCommunication.hpp"
#include "Gripper/ECBPMi/ECBPMiSerial.hpp"

int main(int argc, char* argv[]) {
    auto serial = std::make_shared<crf::communication::serialcommunication::SerialCommunication>(
        "/dev/ttyUSB0", 115200, false, true, 8, std::chrono::seconds(0));
    std::string urdfPath = __FILE__;
    urdfPath = urdfPath.substr(0, urdfPath.find("cpproboticframework"));
    urdfPath += "cpproboticframework/modules/Actuators/Gripper/config/ECBPMi/ECBPMi.urdf";
    crf::actuators::gripper::ECBPMiSerial vGripper(serial, urdfPath);
    if (!vGripper.initialize()) {
        return -1;
    }

    bool runSwitch = true;
    while (runSwitch) {
        int choice = -1;
        std::cout << "Choose an option:" << std::endl;
        std::cout << "1. Activate" << std::endl;
        std::cout << "2. Deactivate" << std::endl;
        std::cout << "0. Quit" << std::endl;
        std::cout << "Choice: ";
        std::cin >> choice;
        std::cout << '\n';

        if (choice == 1) {
            vGripper.activate();
        } else if (choice == 2) {
            vGripper.deactivate();
        } else if (choice == 0) {
            runSwitch = false;
        } else if (choice < 0 || choice > 2) {
            std::cout << "Unknown option: try again..." << std::endl;
            std::cout << '\n';
        }
    }
    return 0;
}
