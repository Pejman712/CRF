/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 * Contributor: Jorge Playán Garai CERN BE/CEM/MRO
 *  ==================================================================================================
 */

#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <csignal>

#include "EventLogger/EventLogger.hpp"
#include "CANSocket/CANSocket.hpp"
#include "SchunkPowerCube/SchunkPowerCube.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerCommunicationPointFactory.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerManager.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "Sockets/Ipc/UnixSocketServer.hpp"
#include "Sockets/Tcp/TcpServer.hpp"

namespace po = boost::program_options;

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    std::cout << "Caught signal Ctrl-Z: " << signal << std::endl;
    gSignalStatus = signal;
}
}  // unnamed namespace

int main(int argc, char* argv[]) {
    crf::utility::logger::EventLogger logger("SchunkPowerCubePoint");
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("configuration", po::value<std::string>(), "Configuration file of the Schunk Power Cube")
        ("can_port", po::value<std::string>(), "Can port name (e.g. can0)")
        ("protocol", po::value<std::string>(), "Protocol (available: tcp) [Required if port set]")
        ("port", po::value<unsigned int>(), "Network port [1-65535] [Required if protocol set]")
        ("input_fifo", po::value<std::string>(), "Input FIFO [Required if output mmap is set]")
        ("output_mmap", po::value<std::string>(), "Output MMAP [Required if input fifo is set]");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    } catch (std::exception&) {
        logger->error("Bad command line values");
        std::cout << desc << std::endl;
        return -1;
    }
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }

    if (!vm.count("configuration")) {
        logger->error("Missing configuration file");
        std::cout << desc << std::endl;
        return -1;
    }

    auto socket = std::make_shared<CANSocket>((char*)(vm["can_port"].as<std::string>().c_str())); // NOLINT

    std::ifstream robotData(vm["configuration"].as<std::string>());
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);

    auto arm = std::make_shared<crf::robots::schunkpowercube::SchunkPowerCube>(socket, robotJSON);

    if (!arm->initialize()) {
        logger->error("Could not initialize the arm");
        return -1;
    }

    std::shared_ptr<crf::control::robotarmcontroller::RobotArmControllerManager> manager(
        new crf::control::robotarmcontroller::RobotArmControllerManager(arm));
    std::shared_ptr<
      crf::control::robotarmcontroller::RobotArmControllerCommunicationPointFactory>
        communicationPointFactory(
          new crf::control::robotarmcontroller::RobotArmControllerCommunicationPointFactory(
            manager));

    std::unique_ptr<crf::communication::sockets::CommunicationPointServer> networkServer;
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
            server = std::make_shared<crf::communication::sockets::TcpServer>(net_port);
        }
        networkServer.reset(new crf::communication::sockets::CommunicationPointServer(
            server, communicationPointFactory));
        if (!networkServer->initialize()) {
            std::cout << "Failed to initialize network server, check logger for details" << std::endl;  // NOLINT
            return -1;
        }

        std::cout << "Started network server on port " << net_port << std::endl;
    }

    std::unique_ptr<crf::communication::sockets::CommunicationPointServer> ipcServer;
    if (vm.count("ipc_name")) {
        std::string ipc_name = vm["ipc_name"].as<std::string>();
        std::shared_ptr<crf::communication::sockets::ISocketServer> server;
        server = std::make_shared<crf::communication::sockets::UnixSocketServer>(ipc_name);
        ipcServer.reset(new crf::communication::sockets::CommunicationPointServer(
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
