/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <csignal>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>

#include <boost/program_options.hpp>

#include "TIMRPWagon/TIMS300RPWagon/TIMS300RPWagon.hpp"
#include "TIMRPWagon/TIMRPWagonCommunicationPoint/TIMRPWagonManager.hpp"
#include "TIMRPWagon/TIMRPWagonCommunicationPoint/TIMRPWagonCommunicationPointFactory.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "Sockets/IPC/UnixSocketServer.hpp"
#include "Sockets/TCP/TCPServer.hpp"

namespace po = boost::program_options;

using crf::communication::communicationpointserver::CommunicationPointServer;
using crf::communication::sockets::TCPServer;
using crf::communication::sockets::UnixSocketServer;
using crf::actuators::timrpwagon::TIMRPWagonCommunicationPointFactory;

namespace {
volatile std::sig_atomic_t signalStatus;
void signalHandler(int signal) {
    std::cout << ": Stop signal [SIGTSTP] received" << std::endl;
    signalStatus = signal;
}
}  // unnamed namespace

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("configuration", po::value<std::string>(), "Configuration file path.")
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp) [Required if port is set].")  // NOLINT
        ("port", po::value<unsigned int>(), "Network port [1-65535] [Required if protocol is set].")
        ("ipc_name", po::value<std::string>(), "IPC socket name.");

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

    if (!vm.count("configuration")) {
        std::cout << "Missing configuration file." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!(vm.count("protocol") && vm.count("port")) && !vm.count("ipc_name")) {
        std::cout << "Missing at least one socket connection." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::ifstream timrpwagonData(vm["configuration"].as<std::string>());
    nlohmann::json timrpwagonJSON = nlohmann::json::parse(timrpwagonData);

    std::shared_ptr<crf::actuators::timrpwagon::TIMS300RPWagon> timrpwagon =
        std::make_shared<crf::actuators::timrpwagon::TIMS300RPWagon>(timrpwagonJSON);

    std::shared_ptr<crf::actuators::timrpwagon::TIMRPWagonManager> manager =
        std::make_shared<crf::actuators::timrpwagon::TIMRPWagonManager>(timrpwagon);

    std::shared_ptr<TIMRPWagonCommunicationPointFactory> communicationPointFactory =
        std::make_shared<crf::actuators::timrpwagon::TIMRPWagonCommunicationPointFactory>(manager);

    std::unique_ptr<CommunicationPointServer> networkServer;
    if (vm.count("protocol") && vm.count("port")) {
        std::string net_protocol = vm["protocol"].as<std::string>();
        int net_port = vm["port"].as<unsigned int>();

        if ((net_protocol != "tcp") || (net_port < 1) || (net_port > 65535)) {
            std::cout << "Wrong network parameters." << std::endl << std::endl;
            std::cout << desc << std::endl;
            return -1;
        }

        std::shared_ptr<crf::communication::sockets::ISocketServer> server;
        if (net_protocol == "tcp") {
            server = std::make_shared<TCPServer>(net_port);
        }
        networkServer.reset(new CommunicationPointServer(server, communicationPointFactory));
        if (!networkServer->initialize()) {
            std::cout << "Failed to initialize network server, more details in EventLogger." << std::endl;  // NOLINT
            return -1;
        }

        std::cout << "TIMRPWagon communication point on " << net_port <<
            " started. Type [CTRL+Z] to close it and deinitialize the TIMRPWagon" << std::endl;
    }

    std::unique_ptr<CommunicationPointServer> ipcServer;
    if (vm.count("ipc_name")) {
        std::string ipc_name = vm["ipc_name"].as<std::string>();
        std::shared_ptr<crf::communication::sockets::ISocketServer> server;
        server = std::make_shared<UnixSocketServer>(ipc_name);
        ipcServer.reset(new CommunicationPointServer(server, communicationPointFactory));

        if (!ipcServer->initialize()) {
            std::cout << "Failed to initialize ips server, more details in EventLogger." << std::endl;  // NOLINT
            return -1;
        }

        std::cout << "TIMRPWagon communication point on " << ipc_name <<
            " started. Type [CTRL+Z] to close it deinitialize the TIMRPWagon" << std::endl;
    }

    std::signal(SIGTSTP, signalHandler);
    while (signalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "Received closing command" << std::endl;

    return 0;
}
