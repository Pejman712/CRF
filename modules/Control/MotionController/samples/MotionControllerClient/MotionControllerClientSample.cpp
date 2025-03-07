/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <vector>
#include <memory>
#include <iostream>
#include <boost/program_options.hpp>
#include <csignal>

#include "Sockets/TCP/TCPSocket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "MotionController/MotionControllerClient/MotionControllerClient.hpp"

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
    if (!vm.count("port") || !vm.count("host")) {
        std::cout << "Missing at least one socket connection." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::shared_ptr<crf::communication::sockets::TCPSocket> clientSocket =
        std::make_shared<crf::communication::sockets::TCPSocket>(
            vm["host"].as<std::string>(),
            vm["port"].as<unsigned int>());

    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket =
        std::make_shared<crf::communication::datapacketsocket::PacketSocket>(
            clientSocket);

    std::chrono::milliseconds serverReplyTimeout(3000);
    int priorityClient = 60;
    float streamerFrequency = 1;
    crf::control::motioncontroller::MotionControllerClient motionController(
        socket,
        serverReplyTimeout,
        streamerFrequency,
        priorityClient);

    if (!motionController.initialize()) {
        std::cout << "Error initializing client" << std::endl;
        return -1;
    }

    while (gSignalStatus != SIGTSTP) {
        Signals signals =  motionController.getSignals();
        std::cout << "Current Joint Position = " << signals.joints.positions.value() << std::endl;  // NOLINT
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
