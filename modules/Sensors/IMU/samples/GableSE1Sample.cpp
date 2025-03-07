/*
 * © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Henry Paul Espinosa Peralta CERN BE/CEM/MRO 2023
 *         Giancarlo D'Ago CERN BE/CEM/MRO 2023
 * 
 *  ==================================================================================================
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <crf/expected.hpp>
#include <boost/program_options.hpp>
#include "IMU/Gable/SE1/GableSE1.hpp"
#include "IMU/Gable/GableInfo.hpp"
#include "EtherCATDrivers/EtherCATMaster/EtherCATMaster.hpp"

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()("help", "Show help message")
    ("ethernet_port", po::value<std::string>(), "Ethernet Port");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    } catch ( std::exception& ) {
        std::cout << "Bad command line values" << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    po::notify(vm);

    if ( vm.count("help") ) {
        std::cout << desc << std::endl;
        return 0;
    }
    if ( !vm.count("ethernet_port") ) {
        std::cout << "Missing ethernet_port" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::string ethernetPort((vm["ethernet_port"].as<std::string>()));
    uint16_t deviceId = 1;

    int IOMapSize = 116;
    int nSlaves = 1;
    std::shared_ptr<crf::devices::ethercatdrivers::EtherCATMaster> masterEthercatImu
        = std::make_shared<crf::devices::ethercatdrivers::EtherCATMaster>(
            ethernetPort, nSlaves, std::chrono::milliseconds(1), IOMapSize);
    masterEthercatImu->initialize();

    crf::sensors::imu::GableSE1 gableIMU(masterEthercatImu, deviceId);
    gableIMU.initialize();

    crf::expected<bool> result = gableIMU.calibrate();
    std::cout << "Calibration result: ";
    if (result) {
        std::cout << result.value() << "\n\n";
    } else {
        crf::ResponseCode error = result.get_response();
        std::cout << error << "\n";
    }

    crf::expected<crf::sensors::imu::GableInfo> info = gableIMU.getInfo();
    std::cout << "Get Info result: ";
    if (info) {
        std::cout << "Baudrate: " << info.value().Baudrate << "\n";
        std::cout << "Firmware Major Version: " << info.value().Firmware << "\n";
        std::cout << "Hardware Major Version: " << info.value().Hardware << "\n";
        std::cout << "DeviceID: " << info.value().DeviceID << "\n\n";
    } else {
        crf::ResponseCode error = info.get_response();
        std::cout << error << "\n";
    }

    while (true) {
        crf::sensors::imu::IMUSignals signal = gableIMU.getSignal();
        crf::expected<std::array<double, 3>> linearAcceleration = signal.linearAcceleration;
        crf::expected<std::array<double, 3>> angularVelocity = signal.angularVelocity;
        crf::expected<std::array<double, 3>> magneticField = signal.magneticField;

        std::cout << " ]"<< std::endl << "Linear Acceleration: [ ";
        if (linearAcceleration) {
            for (int i=0; i < 3; i++) {
                std::cout << signal.linearAcceleration.value()[i] << " ";
            }
        } else {
            crf::ResponseCode errorLinAcc = linearAcceleration.get_response();
            std::cout << errorLinAcc;
        }

        std::cout << " ]"<< std::endl << "Angular Velocity: [ ";
        if (angularVelocity) {
            for (int i=0; i < 3; i++) {
                std::cout << signal.angularVelocity.value()[i] << " ";
            }
        } else {
            crf::ResponseCode errorAngVel = angularVelocity.get_response();
            std::cout << errorAngVel;
        }

        std::cout << " ]"<< std::endl << "Magnetic Field: [ ";
        if (magneticField) {
            for (int i=0; i < 3; i++) {
                std::cout << signal.magneticField.value()[i] << " ";
            }
        } else {
            crf::ResponseCode errorMagFie = magneticField.get_response();
            std::cout << errorMagFie;
        }

        std::cout << " ]"<< std::endl << "-----------------------" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 1;
}
