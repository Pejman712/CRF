/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2024
 *  ==================================================================================================
 */

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <csignal>
#include <fstream>

#include <boost/program_options.hpp>
#include <nlohmann/json.hpp>

#include "CANopenDrivers/CoEMaster/CoEMaster.hpp"

#include "IMU/IMUCommunicationPoint/IMUManager.hpp"
#include "IMU/IMUCommunicationPoint/IMUCommunicationPointFactory.hpp"
#include "IMU/Gable/SE2SE3/GableSE2SE3.hpp"
#include "IMU/Gable/SE1/GableSE1.hpp"

#include "CANopenDrivers/CiA402/CoEDrivers/ZeroErrERobCoEDriver/ZeroErrERobCoEDriver.hpp"
#include "CANopenDrivers/CiA402/CiA402CommunicationPoint/CiA402Manager.hpp"
#include "CANopenDrivers/CiA402/CiA402CommunicationPoint/CiA402CommunicationPointFactory.hpp"

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
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp) [Required if port is set]")  // NOLINT
        ("ethercat_port", po::value<std::string>(), "Ethernet Port")
        ("configuration", po::value<std::string>(), "Path to the configuration file of the driver")
        ("portDriver", po::value<unsigned int>(), "Network port [1-65535] [Required if protocol is set]")  // NOLINT
        ("portIMU", po::value<unsigned int>(), "Network port [1-65535] [Required if protocol is set]");  // NOLINT

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

    if (!vm.count("protocol") || !vm.count("portDriver") ||
        !vm.count("portIMU") || !vm.count("ethercat_port")) {
        std::cout << "Missing device parameters" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::string ethernetPort((vm["ethercat_port"].as<std::string>()));
    std::string configFile = vm["configuration"].as<std::string>();
    nlohmann::json config = nlohmann::json::parse(std::ifstream(configFile));
    int nSlaves = 2;

    uint16_t driverid = 1;
    uint16_t IMUid = 2;

    std::shared_ptr<crf::devices::canopendrivers::CoEMaster> master =
        std::make_shared<crf::devices::canopendrivers::CoEMaster>(
            ethernetPort, nSlaves, std::chrono::milliseconds(1), config);

    std::shared_ptr<crf::sensors::imu::IIMU> sensor =
        std::make_shared<crf::sensors::imu::GableSE2SE3>(master, IMUid);

    std::shared_ptr<crf::devices::canopendrivers::ICiA402Driver> driver =
        std::make_shared<crf::devices::canopendrivers::ZeroErrERobCoEDriver>(
            master, driverid, config);

    if (!master->initialize()) {
        std::puts("Master failed to initialize!");
        return -1;
    }

    // Driver Comm point

    auto managerDriver = std::make_shared<crf::devices::canopendrivers::CiA402Manager>(driver);

    std::shared_ptr<crf::devices::canopendrivers::CiA402CommunicationPointFactory> communicationPointFactoryDriver(  // NOLINT
        new crf::devices::canopendrivers::CiA402CommunicationPointFactory(managerDriver));

    std::unique_ptr<crf::communication::communicationpointserver::CommunicationPointServer> networkServerDriver;  // NOLINT
    if (vm.count("protocol") && vm.count("portDriver")) {
        std::string net_protocol = vm["protocol"].as<std::string>();
        int net_port = vm["portDriver"].as<unsigned int>();

        if ((net_protocol != "tcp") || (net_port < 1) || (net_port > 65535)) {
            std::cout << "Wrong network parameters" << std::endl << std::endl;
            std::cout << desc << std::endl;
            return -1;
        }

        std::shared_ptr<crf::communication::sockets::ISocketServer> server;
        if (net_protocol == "tcp") {
            server = std::make_shared<crf::communication::sockets::TCPServer>(net_port);
        }
        networkServerDriver.reset(
            new crf::communication::communicationpointserver::CommunicationPointServer(
                server, communicationPointFactoryDriver));
        if (!networkServerDriver->initialize()) {
            std::cout << "Failed to initialize network server, check logger for details" << std::endl;  // NOLINT
            return -1;
        }

        std::cout << "Started Driver network server on port " << net_port << std::endl;
    }

    std::unique_ptr<crf::communication::communicationpointserver::CommunicationPointServer> ipcServerDriver;  // NOLINT
    if (vm.count("ipc_name")) {
        std::string ipc_name = vm["ipc_name"].as<std::string>();
        std::shared_ptr<crf::communication::sockets::ISocketServer> server;
        server = std::make_shared<crf::communication::sockets::UnixSocketServer>(ipc_name);
        ipcServerDriver.reset(
            new crf::communication::communicationpointserver::CommunicationPointServer(
                server, communicationPointFactoryDriver));

        if (!ipcServerDriver->initialize()) {
            std::cout << "Failed to initialize ips server, check logger for details" << std::endl;  // NOLINT
            return -1;
        }

        std::cout << "Started ipc server on resource " << ipc_name << std::endl;
    }

    // IMU Comm point

    auto managerIMU = std::make_shared<crf::sensors::imu::IMUManager>(sensor);

    std::shared_ptr<crf::sensors::imu::IMUCommunicationPointFactory> communicationPointFactoryIMU(  // NOLINT
        new crf::sensors::imu::IMUCommunicationPointFactory(managerIMU));

    std::unique_ptr<crf::communication::communicationpointserver::CommunicationPointServer> networkServerIMU;  // NOLINT
    if (vm.count("protocol") && vm.count("portIMU")) {
        std::string net_protocol = vm["protocol"].as<std::string>();
        int net_port = vm["portIMU"].as<unsigned int>();

        if ((net_protocol != "tcp") || (net_port < 1) || (net_port > 65535)) {
            std::cout << "Wrong network parameters" << std::endl << std::endl;
            std::cout << desc << std::endl;
            return -1;
        }

        std::shared_ptr<crf::communication::sockets::ISocketServer> server;
        if (net_protocol == "tcp") {
            server = std::make_shared<crf::communication::sockets::TCPServer>(net_port);
        }
        networkServerIMU.reset(
            new crf::communication::communicationpointserver::CommunicationPointServer(
                server, communicationPointFactoryIMU));
        if (!networkServerIMU->initialize()) {
            std::cout << "Failed to initialize network server, check logger for details" << std::endl;  // NOLINT
            return -1;
        }

        std::cout << "Started IMU network server on port " << net_port << std::endl;
    }

    std::unique_ptr<crf::communication::communicationpointserver::CommunicationPointServer> ipcServerIMU;  // NOLINT
    if (vm.count("ipc_name")) {
        std::string ipc_name = vm["ipc_name"].as<std::string>();
        std::shared_ptr<crf::communication::sockets::ISocketServer> server;
        server = std::make_shared<crf::communication::sockets::UnixSocketServer>(ipc_name);
        ipcServerIMU.reset(
            new crf::communication::communicationpointserver::CommunicationPointServer(
                server, communicationPointFactoryIMU));

        if (!ipcServerIMU->initialize()) {
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
