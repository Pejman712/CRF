/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include <boost/program_options.hpp>

#include "Gripper/SchunkGripperCANOpen/SchunkGripperCANOpen.hpp"
#include "CANSocket/CANSocket.hpp"

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("can_port", po::value<std::string>(), "CAN port name (e.g. can0)");

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

    auto canSocket = std::make_shared<crf::communication::cansocket::CANSocket>(
        reinterpret_cast<const char*>(vm["can_port"].as<std::string>().c_str()));
    auto ctx = std::make_shared<crf::devices::canopendevices::CANOpenContext>(canSocket);

    if (!ctx->initialize()) {
        std::puts("Could not initialize context");
    return -1;
    }

    auto gripper = std::make_shared<crf::actuators::gripper::SchunkGripperCANOpen>(
        canSocket, ctx);

    gripper->initialize();

    std::cout << "Seting gripper to middle" << std::endl;
    gripper->setPosition(50);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Opening" << std::endl;
    gripper->setPosition(crf::actuators::gripper::IGripper::Gripper_Open);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Closing" << std::endl;
    gripper->setPosition(crf::actuators::gripper::IGripper::Gripper_Closed);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "Opening fast" << std::endl;
    gripper->setVelocity(-100);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Closing very slow" << std::endl;
    gripper->setVelocity(5);
    std::this_thread::sleep_for(std::chrono::seconds(18));

    gripper->deinitialize();

    return 0;
}
