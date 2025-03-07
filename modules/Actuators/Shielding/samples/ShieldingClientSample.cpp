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
#include "Shielding/ShieldingClient/ShieldingClient.hpp"

namespace po = boost::program_options;

using crf::communication::sockets::ISocket;
using crf::communication::sockets::TCPSocket;
using crf::communication::sockets::UnixSocket;
using crf::communication::datapacketsocket::PacketSocket;

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

    std::shared_ptr<ISocket> clientSocket;
    if (vm.count("port") && vm.count("host")) {
        clientSocket = std::make_shared<TCPSocket>(vm["host"].as<std::string>(),
            vm["port"].as<unsigned int>());
    } else if (vm.count("ipc_name")) {
        clientSocket = std::make_shared<UnixSocket>(vm["ipc_name"].as<std::string>());
    } else {
        std::cout << "Not able to create a connection with the given information" << std::endl;
        return 0;
    }
    std::shared_ptr<PacketSocket> socket(new PacketSocket(clientSocket));

    std::chrono::milliseconds serverReplyTimeout(25000);
    int priorityClient = 60;
    float streamerFrequency = 0;
    std::shared_ptr<crf::actuators::shielding::ShieldingClient> shielding(
        new crf::actuators::shielding::ShieldingClient(socket, serverReplyTimeout,
            streamerFrequency, priorityClient));

    if (shielding->initialize()) {
        std::cout << "Shielding initialized" << std::endl;
    } else {
        std::cout << "Shielding NOT initialized" << std::endl;
        return -1;
    }
    sleep(3);

    if (shielding->close()) {
        std::cout << "Shielding is closing" << std::endl;
    } else {
        std::cout << "Shielding is NOT closing" << std::endl;
    }
    sleep(3);

    if (shielding->isClosed()) {
        std::cout << "Shielding is closed" << std::endl;
    } else {
        std::cout << "Shielding is NOT closed" << std::endl;
    }
    if (shielding->isOpen()) {
        std::cout << "Shielding is opened" << std::endl;
    } else {
        std::cout << "Shielding is NOT opened" << std::endl;
    }
    sleep(3);

    if (shielding->open()) {
        std::cout << "Shielding is opening" << std::endl;
    } else {
        std::cout << "Shielding is NOT opening" << std::endl;
    }
    sleep(3);
    if (shielding->isClosed()) {
        std::cout << "Shielding is closed" << std::endl;
    } else {
        std::cout << "Shielding is NOT closed" << std::endl;
    }
    if (shielding->isOpen()) {
        std::cout << "Shielding is opened" << std::endl;
    } else {
        std::cout << "Shielding is NOT opened" << std::endl;
    }
    sleep(3);

    if (!shielding->deinitialize()) {
        std::cout << "Cannot deinitialize Shielding." << std::endl;
        return -1;
    }
    std::cout << "Shielding deinitialized." << std::endl;

    return 0;
}
