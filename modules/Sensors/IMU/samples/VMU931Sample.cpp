/*
 * © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <iostream>
#include <thread>
#include <fstream>
#include <filesystem>

#include "IMU/VMU931/VMU931.hpp"

std::vector<std::string> findVMU931Devices() {
    std::vector<std::string> list;
    for (const auto& entry : std::filesystem::directory_iterator("/dev/serial/by-id/")) {
        const std::string deviceName = entry.path().filename();
        if (deviceName.find("usb-Variense_VMU931") != std::string::npos) {
            list.push_back("/dev/serial/by-id/" + deviceName);
        }
    }
    std::cout << list.size() << " VMU931 devices found" << std::endl;
    return list;
}

int main(int argc, char* argv[]) {
    std::vector< std::string> list = findVMU931Devices();
    if (list.size() != 1) {
        std::cout << "More than one VMU931 device found" << std::endl;
        return -1;
    }
    std::string command = "sudo chmod a+rw " + list[0];
    int result = std::system(command.c_str());

    // Sets serial communication for IMU sensor
    auto serial = std::make_shared<crf::communication::serialcommunication::SerialCommunication>(
        list[0], 57600, true);

    crf::sensors::imu::VMU931 imu(serial);
    imu.initialize();

    if (!imu.calibrate()) {
        std::cout << "Failed to calibrate" << std::endl;
        return -1;
    }
    // Sensor must be hold in an horizontal position during calibration
    std::cout << "Waiting for 5 seconds ..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        crf::sensors::imu::IMUSignals signal = imu.getSignal();
        std::cout << "New values" << std::endl;
        if (signal.position) {
            std::cout << "position = " <<
                signal.position.value()[0] << ", " <<
                signal.position.value()[1] << ", " <<
                signal.position.value()[2] <<std::endl;
        }
        if (signal.quaternion) {
            std::cout << "quaternion = " <<
                signal.quaternion.value()[0] << ", " <<
                signal.quaternion.value()[1] << ", " <<
                signal.quaternion.value()[2] << ", " <<
                signal.quaternion.value()[3] <<std::endl;
        }
        if (signal.linearVelocity) {
            std::cout << "linearVelocity = " <<
                signal.linearVelocity.value()[0] << ", " <<
                signal.linearVelocity.value()[1] << ", " <<
                signal.linearVelocity.value()[2] <<std::endl;
        }
        if (signal.angularVelocity) {
            std::cout << "angularVelocity = " <<
                signal.angularVelocity.value()[0] << ", " <<
                signal.angularVelocity.value()[1] << ", " <<
                signal.angularVelocity.value()[2] <<std::endl;
        }
        if (signal.linearAcceleration) {
            std::cout << "linearAcceleration = " <<
                signal.linearAcceleration.value()[0] << ", " <<
                signal.linearAcceleration.value()[1] << ", " <<
                signal.linearAcceleration.value()[2] <<std::endl;
        }
        if (signal.angularAcceleration) {
            std::cout << "angularAcceleration = " <<
                signal.angularAcceleration.value()[0] << ", " <<
                signal.angularAcceleration.value()[1] << ", " <<
                signal.angularAcceleration.value()[2] <<std::endl;
        }
        if (signal.magneticField) {
            std::cout << "magneticField = " <<
                signal.magneticField.value()[0] << ", " <<
                signal.magneticField.value()[1] << ", " <<
                signal.magneticField.value()[2] <<std::endl;
        }
    }
    return 0;
}
