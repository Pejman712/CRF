/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>
#include <iostream>
#include <memory>

#include <boost/program_options.hpp>

#include "Sockets/ISocket.hpp"
#include "Sockets/TCP/TCPSocket.hpp"
#include "Sockets/IPC/UnixSocket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "MechanicalStabilizer/MechanicalStabilizerClient/MechanicalStabilizerClient.hpp"

namespace po = boost::program_options;

using crf::communication::sockets::ISocket;
using crf::communication::sockets::TCPSocket;
using crf::communication::sockets::UnixSocket;
using crf::communication::datapacketsocket::PacketSocket;
using crf::actuators::mechanicalstabilizer::MechanicalStabilizerClient;

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp) [Required if port is set].") // NOLINT
        ("host", po::value<std::string>(), "IP of the server device")
        ("port", po::value<unsigned int>(), "Network port [1-65535] [Required if protocol is set].")
        ("ipc_name", po::value<std::string>(), "IPC socket name");

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
    if (!(vm.count("protocol") && vm.count("port") && vm.count("host")) && !vm.count("ipc_name")) {
        std::cout << "Missing at least one socket connection." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::shared_ptr<ISocket> client;
    if (vm.count("port") && vm.count("host")) {
        client = std::make_shared<TCPSocket>(vm["host"].as<std::string>(),
            vm["port"].as<unsigned int>());
    } else if (vm.count("ipc_name")) {
        client = std::make_shared<UnixSocket>(vm["ipc_name"].as<std::string>());
    } else {
        std::cout << "Not able to create a connection with the given information" << std::endl;
        return 0;
    }
    std::shared_ptr<PacketSocket> socket(new PacketSocket(client));

    std::shared_ptr<MechanicalStabilizerClient> stabilizer(new MechanicalStabilizerClient(socket,
        std::chrono::milliseconds(20000), 0, 1));

    if (!stabilizer->initialize()) {
        std::cout << "ERROR: Stabilizer NOT initialized." << std::endl;
        return -1;
    }
    std::cout << "Stabilizer initialized." << std::endl;

    sleep(3);

    auto val = stabilizer->isDeactivated();

    if (val.value()) {
        std::cout << "The Stabilizer is deactivated." << std::endl;
    }

    sleep(3);

    val = stabilizer->isActivated();

    if (val.value()) {
        std::cout << "The Stabilizer is activated." << std::endl;
    }

    sleep(3);

    if (!stabilizer->activate()) {
        std::cout << "ERROR: Stabilizer is NOT activating." << std::endl;
        return -1;
    }
    std::cout << "Stabilizer activating function." << std::endl;

    sleep(3);

    if (!stabilizer->deactivate()) {
        std::cout << "ERROR: Stabilizer is NOT deactivating." << std::endl;
        return -1;
    }
    std::cout << "Stabilizer deactivating function." << std::endl;

    sleep(3);

    if (!stabilizer->deinitialize()) {
        std::cout << "Cannot deinitialize Stabilizer." << std::endl;
        return -1;
    }
    std::cout << "Stabilizer deinitialized." << std::endl;

    return 0;
}
