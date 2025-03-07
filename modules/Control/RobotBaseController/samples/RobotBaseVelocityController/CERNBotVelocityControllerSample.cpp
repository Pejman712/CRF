/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include <vector>
#include <fstream>

#include "CERNBot/CERNBot.hpp"
#include "RobotBaseController/RobotBaseVelocityController/RobotBaseVelocityController.hpp"
#include "CANSocket/CANSocket.hpp"

namespace po = boost::program_options;

using crf::communication::cansocket::CANSocket;

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("configuration", po::value<std::string>(), "Configuration file path for the CERNBot Base")
        ("can_port", po::value<std::string>(), "CAN port name (e.g. can0)");  // NOLINT

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
    if (!vm.count("configuration")) {
        std::cout << "Missing configuration file" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    if (!vm.count("can_port")) {
        std::cout << "Missing can port" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    std::shared_ptr<CANSocket> canSocket = std::make_shared<CANSocket>(
        reinterpret_cast<const char*>(vm["can_port"].as<std::string>().c_str()));
    std::ifstream robotData(vm["configuration"].as<std::string>());
    nlohmann::json baseJSON = nlohmann::json::parse(robotData);

    std::shared_ptr<crf::actuators::robotbase::CERNBot> base =
        std::make_shared<crf::actuators::robotbase::CERNBot>(
            canSocket,
            baseJSON);

    std::shared_ptr<crf::control::robotbasecontroller::RobotBaseVelocityController> controller(
        new crf::control::robotbasecontroller::RobotBaseVelocityController(base));

    std::cout << "Pos: " << controller->getPosition() << std::endl;
    std::cout << "Vel: " << controller->getVelocity() << std::endl;
    return 0;
}
