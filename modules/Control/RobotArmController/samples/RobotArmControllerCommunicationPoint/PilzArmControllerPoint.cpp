/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *  ==================================================================================================
*/

#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include <vector>
#include <csignal>

#include "CANSocket/CANSocket.hpp"
#include "PilzArm/PilzArm.hpp"
#include "Gripper/SchunkGripperCANOpen/SchunkGripperCANOpen.hpp"
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
        ("can_port", po::value<std::string>(), "Can port name (e.g. can0)")  // NOLINT
        ("configuration", po::value<std::string>(), "Configuration file path for the Schunk Arm")
        ("gripper", po::value<std::string>(), "Gripper [optional] (available: schunk, robotiq)")
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

    if (!vm.count("can_port")) {
        std::cout << "Missing can port" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("configuration")) {
        std::cout << "Missing configuration file" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    auto can_socket = std::make_shared<crf::communication::cansocket::CANSocket>(
        (char*)(vm["can_port"].as<std::string>().c_str())); // NOLINT
    auto ctx = std::make_shared<crf::devices::canopendevices::CANOpenContext>(can_socket);

    if (!ctx->initialize()) {
        std::puts("It could not initialize context");
        return -1;
    }

    std::shared_ptr<crf::actuators::gripper::SchunkGripperCANOpen> gripper;
    if (vm.count("gripper")) {
        if (vm["gripper"].as<std::string>() == "schunk") {
            gripper = std::make_shared<crf::actuators::gripper::SchunkGripperCANOpen>(
                can_socket, ctx);
        }
    }

    std::ifstream robotData(vm["configuration"].as<std::string>());
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);

    std::shared_ptr<crf::actuators::pilzarm::PilzArm> arm =
        std::make_shared<crf::actuators::pilzarm::PilzArm>(
            can_socket,
            robotJSON,
            gripper,
            ctx);

    if (!arm->initialize()) {
        std::cout << "Could not initialize the arm" << std::endl;
        return -1;
    }

    std::shared_ptr<crf::control::robotarmcontroller::RobotArmControllerManager> manager(
        new crf::control::robotarmcontroller::RobotArmControllerManager(arm, gripper));
    std::shared_ptr<
      crf::control::robotarmcontroller::RobotArmControllerCommunicationPointFactory>
        communicationPointFactory(
          new crf::control::robotarmcontroller::RobotArmControllerCommunicationPointFactory(
            manager));

    std::unique_ptr<crf::communication::communicationpointserver::CommunicationPointServer> networkServer;  // NOLINT
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
        networkServer.reset(new crf::communication::communicationpointserver::CommunicationPointServer(  // NOLINT
            server, communicationPointFactory));
        if (!networkServer->initialize()) {
            std::cout << "Failed to initialize network server, check logger for details" << std::endl;  // NOLINT
            return -1;
        }

        std::cout << "Started network server on port " << net_port << std::endl;
    }

    std::signal(SIGTSTP, signal_handler);
    std::cout << "Communication point started correctly\n";
    std::cout << "Use Ctrl-Z for a smooth ending, or Ctrl-C to abort the execution\n";
    while (gSignalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return 0;
}
