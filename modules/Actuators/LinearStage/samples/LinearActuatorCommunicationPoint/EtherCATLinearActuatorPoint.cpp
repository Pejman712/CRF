 /* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>

#include <boost/program_options.hpp>

#include "EtherCATDevices/EtherCATManager.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "EtherCATDevices/EtherCATMotor.hpp"
#include "EtherCATDevices/EtherCATDef.hpp"

#include "LinearStage/EtherCATLinearActuator/EtherCATLinearActuator.hpp"

#include "Sockets/TCP/TCPServer.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "LinearStage/LinearActuatorCommunicationPoint/LinearActuatorCommunicationPoint.hpp"
#include "LinearStage/LinearActuatorCommunicationPoint/LinearActuatorCommunicationPointFactory.hpp"
#include "LinearStage/LinearActuatorCommunicationPoint/LinearActuatorManager.hpp"

using crf::devices::ethercatdevices::EtherCATManager;
using crf::devices::ethercatdevices::IEtherCATMotor;
using crf::devices::ethercatdevices::EtherCATMotor;
using crf::devices::ethercatdevices::modesofoperation::ProfilePositionMode;

using crf::actuators::linearactuator::EtherCATLinearActuator;

namespace po = boost::program_options;

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    std::cout << "Caught signal Ctrl-Z: " << signal << std::endl;
    gSignalStatus = signal;
}
}  // unnamed namespace

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("ethercat_port", po::value<std::string>(), "EtherCAT port of the linear stage (eno1, enp5s0, ...)")  // NOLINT
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp) [Required if port is set]")  // NOLINT
        ("port", po::value<unsigned int>(), "Network port [1-65535] [Required if protocol is set]");  // NOLINT

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    } catch (std::exception&) {
        std::cout << "Bad command line values" << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }

    if (!vm.count("ethercat_port")) {
        std::cout << "Missing ethercat port file" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("protocol") || !vm.count("port")) {
        std::cout << "Missing communication params" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::string ifname = vm["ethercat_port"].as<std::string>();
    int IOMapSize = 4096;
    int nSlaves = 1;
    std::shared_ptr<EtherCATManager> manager = std::make_shared<EtherCATManager>(ifname,
    nSlaves, IOMapSize, 1000);
    std::shared_ptr<IEtherCATMotor> motor = std::make_shared<EtherCATMotor>(1, manager);

    std::shared_ptr<EtherCATLinearActuator> linearAct =
        std::make_shared<EtherCATLinearActuator>(motor);

    // ----- MANAGER AND MOTOR INITIALIZATION -------
    if (manager->initialize()) {
        printf("Manager initialized\n");
    } else {
        printf("ERROR: Cannot initiliaze Manager\n");
        return -1;
    }

    if (motor->initialize()) {
        printf("Linear Actuator initialized\n");
    } else {
        printf("ERROR: Cannot initiliaze Linear Actuator\n");
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
    // ----------------------------------------------


    std::this_thread::sleep_for(std::chrono::seconds(1));


    // ---- MANAGER IN OPERATION (NMT - START PDO COMMUNICATION -----
    if (manager->enterOp()) {
        printf("Manager in Operation Mode \n");
    } else {
        printf("Manager can't enter in Operation Mode \n");
        return -1;
    }
    // --------------------------------------------------------------


    std::this_thread::sleep_for(std::chrono::seconds(1));


    // ---- CONFIGURATION (MANDATORY) ----
    if (motor->setMaxCurrent(50000)) {
        printf("Set motor max current\n");  // ALWAYS TO SET
    } else {
        printf("ERROR: Max current can't be set \n");
        return -1;
    }

    if (motor->setMaxTorque(50000)) {
        printf("Set motor max current\n");  // ALWAYS TO SET
    } else {
        printf("ERROR: Max current can't be set \n");
        return -1;
    }
    // -------------------------------------


    std::this_thread::sleep_for(std::chrono::seconds(1));


    // ---- MOTOR IN OPERATION ----
    if (motor->setModeOfOperation(ProfilePositionMode)) {
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
    // ------------------------------

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Motion Communication Point
    std::shared_ptr<crf::actuators::linearactuator::LinearActuatorManager> linearManager(
        new crf::actuators::linearactuator::LinearActuatorManager(linearAct));

    std::shared_ptr<crf::actuators::linearactuator::LinearActuatorCommunicationPointFactory> communicationPointFactory( // NOLINT
        new crf::actuators::linearactuator::LinearActuatorCommunicationPointFactory(
            linearManager));

    std::unique_ptr<crf::communication::communicationpointserver::CommunicationPointServer>
        networkServer;

    std::string net_protocol = vm["protocol"].as<std::string>();
    int net_port = vm["port"].as<unsigned int>();

    if ((net_protocol != "tcp") || (net_port < 1) || (net_port > 65535)) {
        std::cout << "Wrong network parameters" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::shared_ptr<crf::communication::sockets::ISocketServer> server;
    if (net_protocol == "tcp") {
        server = std::make_shared<crf::communication::sockets::TCPServer>(net_port);
    }
    networkServer.reset(
        new crf::communication::communicationpointserver::CommunicationPointServer(server,
        communicationPointFactory));
    if (!networkServer->initialize()) {
        std::cout << "Failed to initialize network server, check logger for details" << std::endl;  // NOLINT
        return -1;
    }

    std::cout << "Started network server on port " << net_port << std::endl;

    std::signal(SIGTSTP, signal_handler);
    std::cout << "Communication point started correctly\n";
    std::cout << "Use Ctrl-Z for a smooth ending, or Ctrl-C to abort the execution\n";
    while (gSignalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
