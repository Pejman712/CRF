/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 *
 */
#include <string>
#include <initializer_list>
#include <cmath>
#include <chrono>
#include <memory>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>
#include <bitset>

#include <nlohmann/json.hpp>

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
#include "CANopenDrivers/CiA402/CANDrivers/ERB415CANDriver/ERB415CANDriver.hpp"
#include "Robot/CiA402Robot/CiA402Robot.hpp"

#include "crf/expected.hpp"

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cout << "Input arguments not correct! Expected Input:" << std::endl;
        std::cout << "[1] Master DCF file" << std::endl;
        std::cout << "[2] CAN port (can0, can1, can2.. , ...)" << std::endl;
        std::cout << "[3] Robot JSON path" << std::endl;
        return -1;
    }

    std::string path = argv[1];  // path dcf
    std::string canPort = argv[2];  // can0
    std::string robotJSONPath = argv[3];  // Robot configuration

    crf::actuators::robot::CiA402RobotConfiguration configuration(
        nlohmann::json::parse(std::ifstream(robotJSONPath)));

    lely::io::IoGuard io_guard;
    lely::io::Context ctx;
    lely::io::Poll poll(ctx);
    lely::ev::Loop loop(poll.get_poll());
    auto exec = loop.get_executor();
    lely::io::Timer timer(poll, exec, CLOCK_MONOTONIC);
    lely::io::CanController ctrl(canPort.c_str());
    lely::io::CanChannel chan(poll, exec);
    chan.open(ctrl);

    std::shared_ptr<crf::devices::canopendrivers::CANopenMaster> master =
        std::make_shared<crf::devices::canopendrivers::CANopenMaster>(
            timer, chan, path, "", 1);

    std::vector<std::shared_ptr<crf::devices::canopendrivers::ICiA402Driver>> drivers;

    for (uint64_t i = 0; i < configuration.getNumberOfMotors(); i++) {
        drivers.push_back(std::make_shared<crf::devices::canopendrivers::ERB415CANDriver>(
            master, i + 3, configuration.getMotorConfigFiles()[i]));
    }

    lely::io::SignalSet sigset(poll, exec);

    // Watch for Ctrl+C or process termination.
    sigset.insert(SIGHUP);
    sigset.insert(SIGINT);
    sigset.insert(SIGTERM);

    // Submit a task to be executed when a signal is raised. We don't care which.
    sigset.submit_wait([&](int /*signo*/) {
        sigset.clear();
        master->AsyncDeconfig().submit(exec, [&]() {
            ctx.shutdown();
        });
    });

    master->Reset();

    std::future<bool> res = std::async(std::launch::async, [&loop]() {
        loop.run();
        return true;
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Create robot

    crf::actuators::robot::CiA402Robot robot(
        drivers, configuration);

    std::puts("Ready to start test");

    if (!robot.initialize()) {
        std::puts("Error initializing");
        return -1;
    }

    std::puts("Starting Test!!");

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Read the actual joint positions
    crf::expected<crf::utility::types::JointPositions> qAct = robot.getJointPositions();
    if (!qAct) {
        std::cout << "Error Response Code: " << qAct.get_response() << std::endl;
    } else {
        std::cout << "qAct = " << qAct.value() << std::endl;
    }

    return 0;
}
