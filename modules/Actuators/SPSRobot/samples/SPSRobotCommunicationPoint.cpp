/* © Copyright CERN 2018.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
*/
/*
#include <string>
#include <csignal>
#include <fstream>
#include <stdio.h>
#include <memory>

#include <boost/program_options.hpp>

#include "IPC/FIFOZMQ.hpp"
#include "IPC/MMAPZMQ.hpp"
#include "IPC/NetworkIPC.hpp"
#include "NetworkServer/TcpServer.hpp"

#include "SPSRobot/SPSRobot.hpp"
#include "IPC/FIFO.hpp"
#include "IPC/MMAP.hpp"
#include "RobotBaseCommunicationPoint/RobotBaseCommunicationPoint.hpp"

namespace po = boost::program_options;

using cern::communication::ipc::NetworkIPC;
using cern::communication::ipc::FIFOZMQ;
using cern::communication::ipc::MMAPZMQ;
using cern::communication::networkserver::INetworkServer;
using cern::communication::networkserver::TcpServer;
using cern::robots::robotbase::SPSRobot;
using cern::communication::robotbasecommunicationpoint::RobotBaseCommunicationPoint;

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
        ("ethercat_port", po::value<std::string>(), "EtherCAT port name (e.g. enp5s0)")  // NOLINT
        ("configuration", po::value<std::string>(), "Configuration file path for the SPSRobot")
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp) [Required if port is set]")  // NOLINT
        ("port", po::value<unsigned int>(), "Network port [1-65535] [Required if protocol is set]")
        ("input_fifo", po::value<std::string>(), "Input FIFO [Required if output mmap is set]")
        ("output_mmap", po::value<std::string>(), "Output MMAP [Required if input fifo is set]");

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

    if (!vm.count("ethercat_port")) {
        std::cout << "Missing ethercat port" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("configuration")) {
        std::cout << "Missing configuration file" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    std::signal(SIGTSTP, signal_handler);
    printf("\n\nStarting SPSRobot\n");

    std::ifstream configFile(vm["configuration"].as<std::string>());
    if ((configFile.rdstate() & std::ifstream::failbit) != 0) {
        std::cout << "Provided configuration file does not exists" << std::endl;
        return -1;
    }

    std::shared_ptr<SPSRobot> bot = std::make_shared<SPSRobot>(
        reinterpret_cast<const char*>(vm["ethercat_port"].as<std::string>().c_str()),
        nlohmann::json::parse(configFile));
    if (!bot->initialize()) {
        std::cout << "Could not initialize the SPSRobot" << std::endl;
        return -1;
    }
    RobotBaseCommunicationPoint communicationPoint(bot);
    std::shared_ptr<NetworkIPC> networkIPC = nullptr;
    if (vm.count("protocol") && vm.count("port")) {
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
        networkIPC = std::make_shared<NetworkIPC>(server);
        if (!communicationPoint.addIpcPair({networkIPC, networkIPC})) {
            std::cout << "Could not add IPC pair" << std::endl;
        }
    }

    if (vm.count("input_fifo") && (vm.count("output_mmap"))) {
        std::string input_fifo_filename = vm["input_fifo"].as<std::string>();
        std::string output_mmap_filename = vm["output_mmap"].as<std::string>();

        auto fifo = FIFOZMQ::CreateReaderPtrNonBlock(input_fifo_filename,
            std::chrono::milliseconds(2000));
        auto mmap = MMAPZMQ::CreateWriterPtr(output_mmap_filename);

        if (!communicationPoint.addIpcPair({fifo, mmap})) {
            std::cout << "Could not add IPC pair" << std::endl;
        }
    }

    if (!communicationPoint.initialize()) {
        std::cout << "Could not initialize the RobotBaseCommunicationPoint" << std::endl;
        return -1;
    }

    std::cout << "SPSRobot started correctly\n";
    while (gSignalStatus != SIGKILL) {
        usleep(5000);
        //std::cout << bot->getCartesianVelocity().get() << std::endl;
    }
    std::cout << "SPSRobot process clean up\n";
    communicationPoint.deinitialize();
    bot->deinitialize();
    return 0;
}
*/

/* © Copyright CERN 2018.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
*/
#include <string>
#include <csignal>
#include <fstream>
#include <stdio.h>
#include <memory>

#include <boost/program_options.hpp>

#include "IPC/FIFOZMQ.hpp"
#include "IPC/MMAPZMQ.hpp"
#include "IPC/NetworkIPC.hpp"
#include "NetworkServer/TcpServer.hpp"

#include "SPSRobot/SPSRobot.hpp"
#include "IPC/FIFO.hpp"
#include "IPC/MMAP.hpp"
#include "RobotBaseCommunicationPoint/RobotBaseCommunicationPoint.hpp"


namespace po = boost::program_options;

using cern::communication::ipc::NetworkIPC;
using cern::communication::ipc::FIFOZMQ;
using cern::communication::ipc::MMAPZMQ;
using cern::communication::networkserver::INetworkServer;
using cern::communication::networkserver::TcpServer;
using cern::robots::robotbase::SPSRobot;
using cern::communication::robotbasecommunicationpoint::RobotBaseCommunicationPoint;

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
        ("ethercat_port", po::value<std::string>(), "EtherCAT port name (e.g. eno1)")  // NOLINT
        ("configuration", po::value<std::string>(), "Configuration file path for the SPSRobot")
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp) [Required if port is set]")  // NOLINT
        ("port", po::value<unsigned int>(), "Network port [1-65535] [Required if protocol is set]")
        ("input_fifo", po::value<std::string>(), "Input FIFO [Required if output mmap is set]")
        ("output_mmap", po::value<std::string>(), "Output MMAP [Required if input fifo is set]");

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

    if (!vm.count("ethercat_port")) {
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
    printf("Starting SPSRobot\n");

    std::ifstream configFile(vm["configuration"].as<std::string>());  // NOLINT
    if ((configFile.rdstate() & std::ifstream::failbit) != 0) {
        std::cout << "Provided configuration file does not exists" << std::endl;
        return -1;
    }
    std::shared_ptr<SPSRobot> bot = std::make_shared<SPSRobot>(reinterpret_cast<const char*>(vm["ethercat_port"].as<std::string>().c_str()),
        nlohmann::json::parse(configFile));
    if (!bot->initialize()) {
        std::cout << "Could not initialize the SPSRobot" << std::endl;
        return -1;
    }

    RobotBaseCommunicationPoint communicationPoint(bot);
    std::shared_ptr<NetworkIPC> networkIPC = nullptr;
    if (vm.count("protocol") && vm.count("port")) {
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
        networkIPC = std::make_shared<NetworkIPC>(server);
        if (!communicationPoint.addIpcPair({networkIPC, networkIPC})) {
            std::cout << "Could not add IPC pair" << std::endl;
        }
    }

    if (vm.count("input_fifo") && (vm.count("output_mmap"))) {
        std::string input_fifo_filename = vm["input_fifo"].as<std::string>();
        std::string output_mmap_filename = vm["output_mmap"].as<std::string>();

        auto fifo = FIFOZMQ::CreateReaderPtrNonBlock(input_fifo_filename,
            std::chrono::milliseconds(2000));
        auto mmap = MMAPZMQ::CreateWriterPtr(output_mmap_filename);

        if (!communicationPoint.addIpcPair({fifo, mmap})) {
            std::cout << "Could not add IPC pair" << std::endl;
        }
    }

    if (!communicationPoint.initialize()) {
        std::cout << "Could not initialize the RobotBaseCommunicationPoint" << std::endl;
        return -1;
    }

    if (networkIPC != nullptr) {
        std::cout << "Waiting for client communication..." << std::endl;
        networkIPC->open();
    }

    std::cout << "SPS Robot started correctly\n";
    while (gSignalStatus != SIGKILL) {
        usleep(5000);
    }
    std::cout << "SPS Robot process clean up\n";
    communicationPoint.deinitialize();
    bot->deinitialize();
    return 0;
}