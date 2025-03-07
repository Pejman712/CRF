/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO 2020
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
#include "CERNBot/CERNBot.hpp"


namespace po = boost::program_options;

using crf::actuators::robotbase::CERNBot;
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
        ("can_port", po::value<std::string>(), "CAN port name (e.g. can1)")
        ("configuration", po::value<std::string>(), "Configuration file path for the CERNBot");

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
    printf("Starting CERNBot\n");
    std::shared_ptr<CANSocket> socket = std::make_shared<CANSocket>(
        reinterpret_cast<const char*>(vm["can_port"].as<std::string>().c_str()));

    std::ifstream configFile(vm["configuration"].as<std::string>());
    if ((configFile.rdstate() & std::ifstream::failbit) != 0) {
        std::cout << "Provided configuration file does not exists" << std::endl;
        return -1;
    }
    std::shared_ptr<CERNBot> bot = std::make_shared<CERNBot>(
        socket, nlohmann::json::parse(configFile));
    if (!bot->initialize()) {
        std::cout << "Could not initialize the CERNBot" << std::endl;
        return -1;
    }

    std::cout << "CERNBot started correctly\n";
    while (gSignalStatus != SIGKILL) {
        usleep(5000);
        std::cout << bot->getTaskVelocity().get() << std::endl;
    }
    std::cout << "CERNBot process clean up\n";
    bot->deinitialize();
    return 0;
}
