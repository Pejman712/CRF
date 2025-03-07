/* © Copyright CERN 2020.  All rights reserved. This software is released under a CERN proprietary software licence.
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
#include "SPSRobot/SPSRobot.hpp"
#include "RobotBase/EtherCATRobotBase.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include <boost/program_options.hpp>
#include <initializer_list>

using crf::actuators::robotbase::EtherCATRobotBase;
using crf::devices::ethercatdevices::IEtherCATMotor;
using crf::actuators::robotbase::SPSRobot;

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("ethercat_port", po::value<std::string>(), "EtherCAT port name (e.g. enp2s0)")  // NOLINT
        ("configuration", po::value<std::string>(), "Configuration file path for the SPSRobot");
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

    std::ifstream configFile(vm["configuration"].as<std::string>());
    if ((configFile.rdstate() & std::ifstream::failbit) != 0) {
        std::cout << "Provided configuration file does not exists" << std::endl;
        return -1;
    }

    printf("\n\nStarting SPSRobot\n\n\n");
    

    std::shared_ptr<SPSRobot> bot = std::make_shared<SPSRobot>(reinterpret_cast<const char*>(vm["ethercat_port"].as<std::string>().c_str()),
        nlohmann::json::parse(configFile));

    if (!bot->initialize()) {
        std::cout << "Could not initialize the SPSRobot" << std::endl;
        return -1;
    }
    else{
        std::cout << "SPSRobot INITIALIZED \n" << std::endl;
    }

/*
    std::initializer_list<float> values {0, 0, 0, 0, 0, 0.5};
    cern::utility::types::CartesianVelocity velocity(values);
    bot->setCartesianVelocity(velocity);
    sleep(5);
    velocity = {0, 0, 0, 0, 0, 0};;
    bot->setCartesianVelocity(velocity);
*/
    sleep(5);

    if (!bot->deinitialize()) {
        std::cout << "Could not deinitialize the SPSRobot" << std::endl;
        return -1;
    }
    return 0;
}
