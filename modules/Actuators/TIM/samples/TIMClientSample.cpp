/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
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
#include "TIM/TIMClient/TIMClient.hpp"

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

    std::chrono::milliseconds serverReplyTimeout(50000);
    int priorityClient = 60;
    float streamerFrequency = 10;
    std::shared_ptr<crf::actuators::tim::TIMClient> tim(
        new crf::actuators::tim::TIMClient(socket, serverReplyTimeout,
            streamerFrequency, priorityClient));
    sleep(3);
    if (tim->initialize()) {
        std::cout << "TIM initialized" << std::endl;
    }

    sleep(3);
    std::optional<bool> result = tim->setCurrentPosition(2000);
    if (!result) {
        std::cout << "Could not set current position" << std::endl;
        return -1;
    }
    std::cout << "setCurrentPosition() = " << result.value() << std::endl;
    sleep(5);
    std::optional<bool> moveOpt = tim->moveToTarget(2001, 0.3);
    if (!moveOpt) {
        std::puts("Error checking if move was succesfull");
        return false;
    }
    if (!moveOpt.value()) {
        std::puts("Movement not succesfull");
    }
    sleep(40);
    return 0;
}
