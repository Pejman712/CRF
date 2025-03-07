/* © Copyright CERN 2023.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
*/

#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include <signal.h>
#include <csignal>
#include <fstream>
#include <thread>

#include <boost/program_options.hpp>

#include "EtherCATDevices/EtherCATManager.hpp"
#include "EtherCATDevices/EtherCATMotor.hpp"

#include "MissionManager/ScienceGateway/Actions/FigureCube.hpp"

using crf::devices::ethercatdevices::EtherCATManager;
using crf::devices::ethercatdevices::EtherCATMotor;

namespace po = boost::program_options;

int main(int argc, char *argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("port", po::value<std::string>(), "Port for the EtherCAT (e.g. eno1).")
        ("id", po::value<uint32_t>(), "ID of the EtherCAT slave (e.g. 1).")
        ("slave_number", po::value<uint32_t>(), "Number of slaves in the network");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    } catch (std::exception&) {
        std::cout << "Bad command line values." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }
    if (!vm.count("port") || !vm.count("slave_number")) {
        std::cout << "Missing parameters." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::string ifname = vm["port"].as<std::string>();
    int IOMapSize = 4096;
    int nSlaves = vm["slave_number"].as<uint32_t>();
    std::shared_ptr<EtherCATManager> manager = std::make_shared<EtherCATManager>(
        ifname, nSlaves, IOMapSize, 1000);
    std::shared_ptr<EtherCATMotor> motor = std::make_shared<EtherCATMotor>(
        vm["id"].as<uint32_t>(), manager);

    crf::applications::missionmanager::sciencegateway::FigureCube cube(motor);

    if (manager->initialize()) {
        printf("Manager initialized\n");
    } else {
        printf("ERROR: Cannot initiliaze Manager\n");
        return -1;
    }

    if (motor->initialize()) {
        printf("Motor initialized\n");
    } else {
        printf("ERROR: Cannot initiliaze Motor\n");
        return -1;
    }

    if (manager->configureIOMap()) {
        printf("IOMap configured\n");
    } else {
        printf("ERROR: Cannot configure IOMap\n");
        return -1;
    }

    if (motor->bindPDOs()) {
        printf("PDO registers linked to the motor\n");
    } else {
        printf("ERROR: Cannot link PDO registers to the motor\n");
        return -1;
    }

    if (manager->enterOp()) {
        printf("Manager in Operation Mode \n");
    } else {
        printf("Manager can't enter in Operation Mode \n");
        return -1;
    }

    if (motor->setMaxCurrent(50000)) {
        printf("Set motor max current\n");
    } else {
        printf("ERROR: Max current can't be set \n");
        return -1;
    }

    if (motor->setMaxTorque(50000)) {
        printf("Set motor max torque\n");
    } else {
        printf("ERROR: Max current can't be set \n");
        return -1;
    }

    if (motor->setModeOfOperation(
        crf::devices::ethercatdevices::modesofoperation::ProfilePositionMode)) {
        printf("Motor in Profile Position Mode \n");
    } else {
        printf("Motor can't enter in Profile Position Mode \n");
        return -1;
    }

    std::optional<bool> retval = motor->inFault();
    if (!retval) {
        printf("ERROR: Communication problem with the motor \n");
        return -1;
    }
    if (retval.value()) {
        printf("Motor in fault. Trying to reset the fault... \n");
        if (motor->faultReset()) {
            printf("Fault reset performed \n");
        } else {
            printf("ERROR: Fault reset can't be performed \n");
            return -1;
        }
    }

    if (motor->shutdown()) {
        printf("Motor shutdown performed \n");
    } else {
        printf("ERROR: Shutdown can't be performed \n");
        return -1;
    }

    if (motor->enableOperation()) {
        printf("Motor enabled for operations \n");
    } else {
        printf("ERROR: Motor can't be enabled for operations \n");
        return -1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    if (!cube.randomizeFigure()) {
        printf("ERROR: random figure failed \n");
        return -1;
    }

    printf("Random position set \n");

    std::cout << "Current figure is: " <<
        static_cast<uint32_t>(cube.getCurrentFigure()) << std::endl;

    return 0;
}
