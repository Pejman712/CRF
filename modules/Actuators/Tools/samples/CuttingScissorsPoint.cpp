/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Contributors: Giacomo Lunghi CERN EN/SMM/MRO
 *  ==================================================================================================
 */

#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <string>

#include "IPC/NetworkIPC.hpp"
#include "NetworkServer/TcpServer.hpp"
#include "CANSocket/CANSocket.hpp"
#include "CanOpenDevices/CanOpenContext.hpp"
#include "CanOpenDevices/CanOpenIOs/CanOpenIOModule.hpp"
#include "Tools/CuttingScissors.hpp"
#include "ComponentAccessControl/SimpleAccessControl.hpp"
#include "Tools/ToolCommunicationPoint.hpp"


namespace po = boost::program_options;

using crf::communication::networkserver::INetworkServer;
using crf::communication::networkserver::TcpServer;
using crf::communication::ipc::NetworkIPC;
using crf::devices::canopendevices::ObjectDictionary;
using crf::devices::canopendevices::CanOpenContext;
using crf::devices::canopendevices::CanOpenIOModule;
using crf::devices::tools::CuttingScissors;
using crf::devices::tools::ToolCommunicationPoint;
using crf::communication::componentaccesscontrol::SimpleAccessControl;

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("can_port", po::value<std::string>(), "Port of the can converter (e.g. can0)")
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
        std::cout << "Missing can_port" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("protocol") || !vm.count("port")) {
        std::cout << "Missing network protocol or port" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    auto canSocket = std::make_shared<CANSocket>(vm["can_port"].as<std::string>());
    auto ctx = std::make_shared<CanOpenContext>(canSocket);
    ctx->setGuardFrequency(std::chrono::milliseconds(150));

    if (!ctx->initialize()) {
        std::cout << "Could not initialize the CanOpenContext" << std::endl;
        return -1;
    }

    auto device = std::make_shared<CanOpenIOModule>(0x01, canSocket);
    if (!ctx->addDevice(device)) {
        std::cout << "Could not add the CanOpenIOModule to the context" << std::endl;
        return -1;
    }

    auto cuttingScissors = std::make_shared<CuttingScissors>(device, 0);
    if (!cuttingScissors->initialize()) {
        std::cout << "Could not initialize the cutting scissors" << std::endl;
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
    auto communicationPoint = std::make_shared<ToolCommunicationPoint>(
        cuttingScissors, networkIPC, networkIPC, 1, accessControl);

    if (!communicationPoint->initialize()) {
        std::cout << "Could not initialize communication point" << std::endl;
        return -1;
    }

    networkIPC->open();

    while (true) {
        sleep(1);
    }

    return 0;
}
