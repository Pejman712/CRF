/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *  ==================================================================================================
 */

#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <thread>
#include <string>

#include "SerialCommunication/SerialCommunication.hpp"
#include "RPSensor/AtomtexBDKG24/AtomtexBDKG24.hpp"

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("device", po::value<std::string>(), "Serial port name (e.g. /dev/ttyUSB0)");

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

    if (!vm.count("device")) {
        std::cout << "Missing device name" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    auto serial = std::make_shared<crf::communication::serialcommunication::SerialCommunication>(
        vm["device"].as<std::string>(), 19200, true, false, 8, std::chrono::seconds(2));

    crf::sensors::rpsensor::AtomtexBDKG24 rpSensor(serial);
    if (!rpSensor.initialize()) {
        std::cout << "Could not initialize rpsensor" << std::endl;
        return -1;
    }

    while (true) {
        std::cout << "Instantaneous Dose = " << rpSensor.getDoseRate().value() << "  |  " <<
            "Cumulative Dose = " << rpSensor.getCumulativeDose().value() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
