/*
 * © Copyright CERN 2024.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <csignal>

#include <boost/program_options.hpp>
#include <nlohmann/json.hpp>

#include "CANopenDrivers/CoEMaster/CoEMaster.hpp"
#include "CANopenDrivers/CiA402/CoEDrivers/ELMOGoldCoEDriver/ELMOGoldCoEDriver.hpp"

using crf::devices::canopendrivers::CoEMaster;
using crf::devices::canopendrivers::ELMOGoldCoEDriver;
using crf::devices::canopendrivers::PositionReference;

namespace po = boost::program_options;

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    std::cout << "Caught signal Ctrl-Z: " << signal << std::endl;
    gSignalStatus = signal;
}
}  // unnamed namespace

int main(int argc, char *argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("ethercat_port", po::value<std::string>(), "Ethercat port (e.g. enp5s0, eno1, ...)")
        ("configuration", po::value<std::string>(), "Path to the configuration file of the driver");

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

    if (!vm.count("ethercat_port") || !vm.count("configuration")) {
        std::cout << "Missing parameters" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::string ifname = vm["ethercat_port"].as<std::string>();
    std::string configFile = vm["configuration"].as<std::string>();

    nlohmann::json config = nlohmann::json::parse(std::ifstream(configFile));

    uint64_t slaveID = config.at("SlaveID").get<uint64_t>();

    std::shared_ptr<CoEMaster> master =
        std::make_shared<CoEMaster>(ifname, 1, std::chrono::milliseconds(1), config, 4096);

    if (!master->initialize()) {
        std::puts("Master initialization failed");
        return -1;
    }

    ELMOGoldCoEDriver driver(master, 1, config);

    if (!driver.initialize()) {
        std::puts("Driver initialization failed");
        return -1;
    }

    auto printData = [&driver] () {
        auto posExp = driver.getPosition();
        auto velExp = driver.getVelocity();
        auto tqeExp = driver.getTorque();
        auto modExp = driver.getModeOfOperation();
        if (posExp) std::cout << "Pos: " << posExp.value() << std::endl;
        if (velExp) std::cout << "Vel: " << velExp.value() << std::endl;
        if (tqeExp) std::cout << "Tqe: " << tqeExp.value() << std::endl;
        if (modExp) std::cout << "Mod: " << modExp << std::endl;
        auto status = driver.getMotorStatus();
        for (auto error : status) {
            std::cout << "Status: " << error << std::endl;
        }
    };

    std::signal(SIGTSTP, signal_handler);
    std::cout << "Use Ctrl-Z for a smooth ending, or Ctrl-C to abort the execution\n";
    while (gSignalStatus != SIGTSTP) {
        printData();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
