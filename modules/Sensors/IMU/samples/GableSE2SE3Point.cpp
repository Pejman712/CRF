/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2024
 *  ==================================================================================================
 */

#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include <vector>
#include <csignal>

#include "IMU/IMUMockConfiguration.hpp"
#include "IMU/IMUCommunicationPoint/IMUManager.hpp"
#include "IMU/IMUCommunicationPoint/IMUCommunicationPointFactory.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "IMU/Gable/SE2SE3/GableSE2SE3.hpp"
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
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp) [Required if port is set]")  // NOLINT
        ("ethernet_port", po::value<std::string>(), "Ethernet Port")
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

    if (!vm.count("protocol") || !vm.count("port") || !vm.count("ethernet_port")) {
        std::cout << "Missing device parameters" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::string ethernetPort((vm["ethernet_port"].as<std::string>()));
    uint16_t deviceId = 1;
    int IOMapSize = 137;
    int nSlaves = 1;
    std::shared_ptr<crf::devices::ethercatdrivers::EtherCATMaster> master =
        std::make_shared<crf::devices::ethercatdrivers::EtherCATMaster>(
            ethernetPort, nSlaves, std::chrono::milliseconds(1), IOMapSize);

    if (!master->initialize()) {
        std::puts("Master failed to initialize!");
        return -1;
    }

    std::shared_ptr<crf::sensors::imu::IIMU> sensor =
        std::make_shared<crf::sensors::imu::GableSE2SE3>(master, deviceId);

    auto manager = std::make_shared<crf::sensors::imu::IMUManager>(sensor);

    std::shared_ptr<crf::sensors::imu::IMUCommunicationPointFactory> communicationPointFactory(  // NOLINT
        new crf::sensors::imu::IMUCommunicationPointFactory(manager));

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
            new crf::communication::communicationpointserver::CommunicationPointServer(server,
                communicationPointFactory));
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

    return 0;
}
