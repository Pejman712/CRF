/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Contributors: Francesco Riccardi CERN EN/SMM/MRO
 *  ==================================================================================================
 */

#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include "IPC/NetworkIPC.hpp"
#include "NetworkServer/TcpServer.hpp"
#include "EtherCATDevices/EtherCATManager.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "EtherCATDevices/EtherCATMotor.hpp"
#include "Tools/Screwdriver.hpp"
#include "ComponentAccessControl/SimpleAccessControl.hpp"
#include "Tools/ToolCommunicationPoint.hpp"


namespace po = boost::program_options;

using crf::communication::networkserver::INetworkServer;
using crf::communication::networkserver::TcpServer;
using crf::communication::ipc::NetworkIPC;
using crf::devices::ethercatdevices::EtherCATManager;
using crf::devices::ethercatdevices::IEtherCATMotor;
using crf::devices::ethercatdevices::EtherCATMotor;
using crf::devices::tools::Screwdriver;
using crf::devices::tools::ToolCommunicationPoint;
using crf::communication::componentaccesscontrol::SimpleAccessControl;

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("EtherCAT_port", po::value<std::string>(), "EtherCAT communication port (e.g. eno1)")
        ("EtherCAT_slaves_number", po::value<unsigned int>(), "Number of EtherCAT slaves")
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp)")
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

    if (!vm.count("EtherCAT_port")) {
        std::cout << "Missing EtherCAT_port" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("EtherCAT_slaves_number")) {
        std::cout << "Missing EtherCAT_slaves_number" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("protocol") || !vm.count("port")) {
        std::cout << "Missing network protocol or port" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    int IOMapSize = 4096;
    std::shared_ptr<EtherCATManager> manager = std::make_shared<EtherCATManager>(
        vm["EtherCAT_port"].as<std::string>(),
        vm["EtherCAT_slaves_number"].as<unsigned int>(),
        IOMapSize, 1000);
    std::shared_ptr<IEtherCATMotor> motor = std::make_shared<EtherCATMotor>(1, manager);
    std::shared_ptr<Screwdriver> screwdriver = std::make_shared<Screwdriver>(motor, 1);

    if (!manager->initialize()) {
        std::cout << "Could not initialize the EtherCAT Manager" << std::endl;
        return -1;
    }

    if (!motor->initialize()) {
        std::cout << "Could not initialize the EtherCAT Motor" << std::endl;
        return -1;
    }

    if (!manager->configureIOMap()) {
        std::cout << "Could not configure the EtherCAT IOMap" << std::endl;
        return -1;
    }

    if (!motor->bindPDOs()) {
        std::cout << "Could not bind the motor with EtherCAT PDOs" << std::endl;
        return -1;
    }

    if (!manager->enterOp()) {
        std::cout << "Could not enter in Operation" << std::endl;
        return -1;
    }

    if (!screwdriver->initialize()) {
        std::cout << "Could not initialize the Screwdriver" << std::endl;
        return -1;
    }

    std::string net_protocol = vm["protocol"].as<std::string>();
    int net_port = vm["port"].as<unsigned int>();

    if ((net_protocol != "tcp") || (net_port < 1) || (net_port > 65535)) {
        std::cout << "Wrong network parameters" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::shared_ptr<INetworkServer> server;
    if (net_protocol == "tcp") {
        server = std::make_shared<TcpServer>(net_port);
    }
    auto networkIPC = std::make_shared<NetworkIPC>(server);

    auto accessControl = std::make_shared<SimpleAccessControl>();
    auto communicationPoint = std::make_shared<ToolCommunicationPoint>(screwdriver, networkIPC,
        networkIPC, 1, accessControl);

    if (!communicationPoint->initialize()) {
        std::cout << "Could not initialize communication point" << std::endl;
        return -1;
    }
    networkIPC->open();
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
