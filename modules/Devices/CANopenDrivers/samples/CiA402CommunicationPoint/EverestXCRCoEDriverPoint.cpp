/*
 * © Copyright CERN 2024.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <csignal>

#include <boost/program_options.hpp>
#include <nlohmann/json.hpp>

#include "CANopenDrivers/CoEMaster/CoEMaster.hpp"
#include "CANopenDrivers/CiA402/CoEDrivers/EverestXCRCoEDriver/EverestXCRCoEDriver.hpp"
#include "CANopenDrivers/CiA402/CiA402CommunicationPoint/CiA402Manager.hpp"
#include "CANopenDrivers/CiA402/CiA402CommunicationPoint/CiA402CommunicationPointFactory.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "Sockets/IPC/UnixSocketServer.hpp"
#include "Sockets/TCP/TCPServer.hpp"

using crf::devices::canopendrivers::CoEMaster;
using crf::devices::canopendrivers::ICiA402Driver;
using crf::devices::canopendrivers::EverestXCRCoEDriver;
using crf::devices::canopendrivers::PositionReference;

namespace po = boost::program_options;

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    std::cout << "Caught signal Ctrl-Z: " << signal << std::endl;
    gSignalStatus = signal;
}
}  // unnamed namespace

int main(int argc, char *argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("ethercat_port", po::value<std::string>(), "Ethercat port (e.g. enp5s0, eno1, ...)")
        ("configuration", po::value<std::string>(), "Path to the configuration file of the driver")
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

    if (!vm.count("protocol") || !vm.count("port") ||
        !vm.count("ethercat_port") || !vm.count("configuration")) {
        std::cout << "Missing parameters" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::string ifname = vm["ethercat_port"].as<std::string>();
    std::string configFile = vm["configuration"].as<std::string>();

    nlohmann::json config = nlohmann::json::parse(std::ifstream(configFile));

    std::shared_ptr<CoEMaster> master =
        std::make_shared<CoEMaster>(ifname, 1, std::chrono::milliseconds(1), config, 4096);

    if (!master->initialize()) {
        std::puts("Master initialization failed");
        return -1;
    }

    uint32_t slaveID = config["SlaveID"].get<uint32_t>();

    std::shared_ptr<ICiA402Driver> driver = std::make_shared<EverestXCRCoEDriver>(
        master, slaveID, config);

    auto manager = std::make_shared<crf::devices::canopendrivers::CiA402Manager>(driver);

    std::shared_ptr<crf::devices::canopendrivers::CiA402CommunicationPointFactory> communicationPointFactory(  // NOLINT
        new crf::devices::canopendrivers::CiA402CommunicationPointFactory(manager));

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
        networkServer.reset(
            new crf::communication::communicationpointserver::CommunicationPointServer(
                server, communicationPointFactory));
        if (!networkServer->initialize()) {
            std::cout << "Failed to initialize network server, check logger for details" << std::endl;  // NOLINT
            return -1;
        }

        std::cout << "Started network server on port " << net_port << std::endl;
    }

    std::unique_ptr<crf::communication::communicationpointserver::CommunicationPointServer> ipcServer;  // NOLINT
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
}
