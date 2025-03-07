/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <csignal>
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "Radars/FraunhoferRadar/FraunhoferRadar.hpp"
#include "SerialCommunication/SerialCommunication.hpp"

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    gSignalStatus = signal;
}
}   // unnamed namespace

int main(int argc, char *argv[]) {
    crf::utility::logger::EventLogger logger("Radar Sample");
    if (argc < 3) {
        logger->error(R"(ArgCount is less than expected. Command example for use:
          [1] radar config file path
          [2] serial port (e.g. /dev/ttyACM0)
        )");
        return -1;
    }
    std::string configFile(argv[1]);
    std::ifstream config(configFile);
    nlohmann::json jConfig;
    config >> jConfig;
    std::signal(SIGTSTP, signal_handler);
    logger->debug("serial comm port opening: {}", argv[2]);
    auto serial = std::make_shared<crf::communication::serialcommunication::SerialCommunication>(
      argv[2], 38400, true, 8, std::chrono::milliseconds(500));
    auto radar = std::make_unique<crf::sensors::fraunhoferradar::FraunhoferRadar>(jConfig, serial);
    radar->initialize();
    int i = 0;
    while (gSignalStatus != SIGTSTP) {
        logger->debug("max Observation frequency: {}",
          radar->getMaxObservationFrequency());
        auto radarData = radar->getFrame();
        if (radarData.empty()) {
            logger->debug("radar packet empty, smth wrong");
            return -1;
        }
        logger->debug(
          "radar Dimensions: {}, {}, cycle: {}", radarData.size(), radarData[0].size(), i);
        i += 1;
    }
}
