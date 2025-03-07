/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
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

#include <nlohmann/json.hpp>
#include <boost/program_options.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/loop_driver.hpp>

#include "CANopenDrivers/CANopenMaster/CANopenMaster.hpp"
#include "CANopenDrivers/CiA402/CANDrivers/CiA402CANDriver/CiA402CANDriver.hpp"

using crf::devices::canopendrivers::CANopenMaster;
using crf::devices::canopendrivers::CiA402CANDriver;
using crf::devices::canopendrivers::PositionReference;

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("can_port", po::value<std::string>(), "CAN port (e.g. can0, can1, ...)")
        ("configuration", po::value<std::string>(), "Path to the configuration file of the driver")
        ("dcf_path", po::value<std::string>(), "Path to the master DCF file")
        ("slaveID", po::value<uint64_t>(), "ID of the slave (e.g. 1, 2, 3, ...)");

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

    if (!vm.count("can_port") || !vm.count("configuration") || !vm.count("slaveID") || !vm.count("dcf_path")) {  // NOLINT
        std::cout << "Missing parameters" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::string canPort = vm["can_port"].as<std::string>();
    std::string jsonPath = vm["configuration"].as<std::string>();
    uint64_t slaveID = vm["slaveID"].as<uint64_t>();
    std::string dcfPath = vm["dcf_path"].as<std::string>();

    lely::io::IoGuard io_guard;
    lely::io::Context ctx;
    lely::io::Poll poll(ctx);
    lely::ev::Loop loop(poll.get_poll());
    auto exec = loop.get_executor();
    lely::io::Timer timer(poll, exec, CLOCK_MONOTONIC);
    lely::io::CanController ctrl(canPort.c_str());
    lely::io::CanChannel chan(poll, exec);

    chan.open(ctrl);
    auto master = std::make_shared<CANopenMaster>(timer, chan, dcfPath, "", 1);
    nlohmann::json data = nlohmann::json::parse(std::ifstream(jsonPath));
    CiA402CANDriver driver(master, slaveID, data);

    // Create a signal handler.
    lely::io::SignalSet sigset(poll, exec);
    sigset.insert(SIGHUP);
    sigset.insert(SIGINT);
    sigset.insert(SIGTERM);
    sigset.submit_wait([&](int ) {
        sigset.clear();
        master->AsyncDeconfig().submit(exec, [&]() {
            ctx.shutdown();
        });
    });

    master->Reset();
    std::future<bool> res = std::async(std::launch::async , [&loop] () {
        loop.run();
        return true;
    });

    auto printData = [&driver] () {
        auto posExp = driver.getPosition();
        auto velExp = driver.getVelocity();
        auto tqeExp = driver.getTorque();
        if (posExp) std::cout << "Pos: " << posExp.value() << std::endl;
        if (velExp) std::cout << "Vel: " << velExp.value() << std::endl;
        if (tqeExp) std::cout << "Tqe: " << tqeExp.value() << std::endl;
        auto status = driver.getMotorStatus();
        for (auto error : status) {
            std::cout << "Status; " << error << std::endl;
        }
    };

    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::puts("Profile Position tests");

    driver.setProfilePosition(6.28, 10, 20, 20, PositionReference::Relative);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    driver.setProfilePosition(0, 62.8, 100, 100, PositionReference::Absolute);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    printData();
    driver.stop();

    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::puts("Profile Velocity tests");

    driver.setProfileVelocity(3.14, 100, 100);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    printData();
    driver.stop();

    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::puts("Profile Torque tests");

    driver.setProfileTorque(1);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    printData();
    driver.stop();

    std::this_thread::sleep_for(std::chrono::seconds(2));
    res.get();
    return 0;
}
