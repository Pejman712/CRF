/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Contributors: Francesco Riccardi CERN EN/SMM/MRO 2020
 *               Alejandro Diaz Rosales CERN BE/SEM/MRO 2021
 *  ==================================================================================================
 */

#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include <vector>
#include <csignal>

#include "EtherCATDevices/TIMRobotArmWagonMotors/TIMRobotArmWagonMotors.hpp"
#include "TIMArm/TIMArm.hpp"
#include "KinovaArm/KinovaApiInterface.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerCommunicationPointFactory.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerManager.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "Sockets/IPC/UnixSocketServer.hpp"
#include "Sockets/TCP/TCPServer.hpp"

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
        ("ethercat_port", po::value<std::string>(), "EtherCAT port name (e.g. enp5s0)")
        ("configuration", po::value<std::string>(), "Configuration file path for the arm")
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp) [Required if port is set]")  // NOLINT
        ("port", po::value<unsigned int>(), "Network port [1-65535] [Required if protocol is set]");

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

    if (!vm.count("configuration")) {
        std::cout << "Missing configuration file" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::shared_ptr<crf::devices::ethercatdevices::TIMRobotArmWagonMotors> motors =
        std::make_shared<crf::devices::ethercatdevices::TIMRobotArmWagonMotors>(
            vm["ethercat_port"].as<std::string>());
    if (!motors->initialize()) {
        std::cout << "Could not initialize the BLM Wagon motors" << std::endl;
        return -1;
    }
    std::cout << "Manager and Motors initialized" << std::endl;

    std::ifstream robotData(vm["configuration"].as<std::string>());
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    std::shared_ptr<crf::actuators::timarm::TIMArm> arm =
        std::make_shared<crf::actuators::timarm::TIMArm>(robotJSON, motors,
            std::make_shared<crf::actuators::kinovaarm::KinovaApiInterface>());
    if (!arm->initialize()) {
        std::cout << "Could not initialize the arm" << std::endl;
        return -1;
    }
    std::cout << "TIM arm initialized" << std::endl;

    std::shared_ptr<crf::control::robotarmcontroller::RobotArmControllerManager> manager(
        new crf::control::robotarmcontroller::RobotArmControllerManager(arm));

    std::shared_ptr<crf::control::robotarmcontroller::RobotArmControllerCommunicationPointFactory> communicationPointFactory( // NOLINT
        new crf::control::robotarmcontroller::RobotArmControllerCommunicationPointFactory(
            manager));

    std::unique_ptr<crf::communication::communicationpointserver::CommunicationPointServer>
        networkServer;
    if (vm.count("protocol") && vm.count("port")) {
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
    }

    std::unique_ptr<crf::communication::communicationpointserver::CommunicationPointServer>
        ipcServer;
    if (vm.count("ipc_name")) {
        std::string ipc_name = vm["ipc_name"].as<std::string>();
        std::shared_ptr<crf::communication::sockets::ISocketServer> server;
        server = std::make_shared<crf::communication::sockets::UnixSocketServer>(ipc_name);
        ipcServer.reset(new crf::communication::communicationpointserver::CommunicationPointServer(
            server, communicationPointFactory));

        if (!ipcServer->initialize()) {
            std::cout << "Failed to initialize ips server, check logger for details" << std::endl;  // NOLINT
            return -1;
        }

        std::cout << "Started ipc server on resource " << ipc_name << std::endl;
    }

    std::signal(SIGTSTP, signal_handler);
    std::cout << "Communication point started correctly\n";
    std::cout << "Use Ctrl-Z for a smooth ending, or Ctrl-C to abort the execution\n";
    while (gSignalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
