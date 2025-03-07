/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>
#include <csignal>
#include <fstream>
#include <stdio.h>
#include <memory>

#include <boost/program_options.hpp>

#include "CANSocket/CANSocket.hpp"
#include "CERNBot2/CERNBot2.hpp"

namespace po = boost::program_options;

using crf::actuators::robotbase::CERNBot2;
using crf::communication::cansocket::CANSocket;

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    gSignalStatus = signal;
}
}  // unnamed namespace

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("can_port", po::value<std::string>(), "CAN port name (e.g. can1)")  // NOLINT
        ("configuration", po::value<std::string>(), "Configuration file path for the CERNBot2");

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
        std::cout << "Missing can port" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("configuration")) {
        std::cout << "Missing configuration file" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    std::signal(SIGTSTP, signal_handler);
    printf("Starting CERNBot2\n");
    std::shared_ptr<CANSocket> socket = std::make_shared<CANSocket>(
        reinterpret_cast<const char*>(vm["can_port"].as<std::string>().c_str()));

    std::ifstream configFile(vm["configuration"].as<std::string>());  // NOLINT
    if ((configFile.rdstate() & std::ifstream::failbit) != 0) {
        std::cout << "Provided configuration file does not exists" << std::endl;
        return -1;
    }

    std::shared_ptr<CERNBot2> bot = std::make_shared<CERNBot2>(
        socket, nlohmann::json::parse(configFile));
    if (!bot->initialize()) {
        std::cout << "Could not initialize the CERNBot2" << std::endl;
        return -1;
    }

    std::cout << "CERNBot2 started correctly\n";
    while (gSignalStatus != SIGKILL) {
        usleep(5000);
        std::cout << bot->getTaskVelocity().get() << std::endl;
    }

    std::cout << "CERNBot2 process clean up\n";
    bot->deinitialize();
    return 0;
}
