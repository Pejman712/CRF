/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <csignal>
#include <iostream>
#include <memory>
#include <string>

#include <boost/program_options.hpp>

#include "EtherCATDevices/TIMRobotArmWagonMotors/TIMRobotArmWagonMotors.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "Shielding/RadioactiveSourceShielding/RadioactiveSourceShielding.hpp"
#include "Shielding/ShieldingCommunicationPoint/ShieldingManager.hpp"
#include "Shielding/ShieldingCommunicationPoint/ShieldingCommunicationPointFactory.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "Sockets/IPC/UnixSocketServer.hpp"
#include "Sockets/TCP/TCPServer.hpp"

namespace po = boost::program_options;

using crf::communication::communicationpointserver::CommunicationPointServer;
using crf::communication::sockets::TCPServer;
using crf::communication::sockets::UnixSocketServer;
using crf::actuators::shielding::ShieldingCommunicationPointFactory;

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
        ("ethercat_port", po::value<std::string>(), "EtherCAT port name (e.g. enp5s0).")
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

    if (!vm.count("ethercat_port")) {
        std::cout << "Missing EtherCAT port." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!(vm.count("protocol") && vm.count("port")) && !vm.count("ipc_name")) {
        std::cout << "Missing at least one socket connection." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::string etherCATPort(vm["ethercat_port"].as<std::string>());
    crf::devices::ethercatdevices::TIMRobotArmWagonMotors etherCATMotors(etherCATPort);

    if (!etherCATMotors.initialize()) {
        std::cout << "Cannot initialize BLM Wagon motors." << std::endl;
        return -1;
    }
    std::cout << "Manager and Motors initialized." << std::endl;

    auto motor = etherCATMotors.getShielding();
    if (!motor) {
        std::cout << "Cannot retrieve shielding motor." << std::endl;
        return -1;
    }

    std::shared_ptr<crf::actuators::shielding::RadioactiveSourceShielding> stabilizer(
        new crf::actuators::shielding::RadioactiveSourceShielding(motor.value()));
    if (!stabilizer->initialize()) {
        std::cout << "Failed to initialize the stabilizer." << std::endl;
        return -1;
    }
    stabilizer->deinitialize();

    std::shared_ptr<crf::actuators::shielding::ShieldingManager> manager(
        new crf::actuators::shielding::ShieldingManager(stabilizer));
    std::shared_ptr<ShieldingCommunicationPointFactory> communicationPointFactory(
        new ShieldingCommunicationPointFactory(manager));

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

        std::cout << "Started network server on port " << net_port << std::endl;
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

        std::cout << "Started IPC server on resource " << ipc_name << std::endl;
    }

    std::signal(SIGTSTP, signalHandler);
    std::cout << "Type [CTRL+Z] to close the communication and deinitialize the stabilizer" << std::endl;  // NOLINT
    while (signalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "Received closing command" << std::endl;
    return 0;
}
