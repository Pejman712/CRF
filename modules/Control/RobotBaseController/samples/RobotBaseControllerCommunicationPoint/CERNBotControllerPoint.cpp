/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */


#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include <vector>
#include <csignal>
#include <fstream>

#include "CERNBot/CERNBot.hpp"
#include "RobotBaseController/RobotBaseControllerCommunicationPoint/RobotBaseControllerCommunicationPointFactory.hpp"
#include "RobotBaseController/RobotBaseControllerCommunicationPoint/RobotBaseControllerManager.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "Sockets/TCP/TCPServer.hpp"
#include "CANSocket/CANSocket.hpp"

namespace po = boost::program_options;

using crf::communication::cansocket::CANSocket;
using crf::communication::communicationpointserver::CommunicationPointServer;

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
        ("configuration", po::value<std::string>(), "Configuration file path for the CERNBot Base")
        ("can_port", po::value<std::string>(), "CAN port name (e.g. can0)")  // NOLINT
        ("port", po::value<unsigned int>(), "Network port [1-65535]");

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
    if (!vm.count("port")) {
        std::cout << "Missing port to host the communication point" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    if (!vm.count("can_port")) {
        std::cout << "Missing can port" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    int net_port = vm["port"].as<unsigned int>();
    if ((net_port < 1) || (net_port > 65535)) {
        std::cout << "Wrong network parameters" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    std::shared_ptr<CANSocket> canSocket = std::make_shared<CANSocket>(
        reinterpret_cast<const char*>(vm["can_port"].as<std::string>().c_str()));
    std::ifstream robotData(vm["configuration"].as<std::string>());
    nlohmann::json baseJSON = nlohmann::json::parse(robotData);

    std::shared_ptr<crf::actuators::robotbase::CERNBot> base =
        std::make_shared<crf::actuators::robotbase::CERNBot>(
            canSocket,
            baseJSON);

    if (!base->initialize()) {
        std::cout << "Could not initialize the arm" << std::endl;
        return -1;
    }

    std::shared_ptr<crf::control::robotbasecontroller::RobotBaseControllerManager> manager(
        new crf::control::robotbasecontroller::RobotBaseControllerManager(base));

    std::shared_ptr<crf::control::robotbasecontroller::RobotBaseControllerCommunicationPointFactory> communicationPointFactory( // NOLINT
        new crf::control::robotbasecontroller::RobotBaseControllerCommunicationPointFactory(
            manager));

    std::unique_ptr<CommunicationPointServer> networkServer;

    std::shared_ptr<crf::communication::sockets::ISocketServer> server =
        std::make_shared<crf::communication::sockets::TCPServer>(net_port);
    networkServer.reset(new CommunicationPointServer(server, communicationPointFactory));
    if (!networkServer->initialize()) {
        std::cout << "Failed to initialize network server, check logger for details" << std::endl;  // NOLINT
        return -1;
    }

    std::cout << "Started network server on port " << net_port << std::endl;
    std::cout << "Communication point started correctly\n";
    std::cout << "Use Ctrl-Z for a smooth ending, or Ctrl-C to abort the execution\n";
    std::signal(SIGTSTP, signal_handler);
    while (gSignalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return 0;
}
