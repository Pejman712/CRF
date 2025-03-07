/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <csignal>
#include <vector>
#include <fstream>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "Radars/FraunhoferRadar/FraunhoferRadar.hpp"
#include "PeakDetection/CFAR.hpp"

volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    gSignalStatus = signal;
}

int main(int argc, char *argv[]) {
    crf::utility::logger::EventLogger logger("CFAR Sample");
    if (argc < 2) {
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

    auto serial = std::make_shared<crf::communication::serialcommunication::SerialCommunication>(
      argv[2], 38400, true, 8, std::chrono::milliseconds(500));

    crf::sensors::fraunhoferradar::FraunhoferRadar radar(jConfig, serial);
    radar.initialize();

    crf::algorithms::peakdetection::CFAR cfar(10, 40, 0.001);
    while (gSignalStatus != SIGTSTP) {
        auto radarData = radar.getFrame();
        if (radarData.empty()) {
            logger->debug("Radar packet empty, smth wrong");
            return -1;
        }
        std::cout << "radarData.size(): " << radarData.size() << '\n';
        std::cout << "radarData[0].size(): " << radarData[0].size() << '\n';
        for (int i = 0; i < radarData.size(); i++) {
            std::vector<float> absFrequencyVector;
            for (int j = 0; j < radarData[i].size(); j++) {
                absFrequencyVector.push_back(std::abs(radarData[i][j]));
            }
            std::vector<int> peakVec = cfar.findPeaks(absFrequencyVector);
            if (!peakVec.empty()) {
                logger->debug("Nearest peak detected at: {}", peakVec[0]);
            }
        }
    }
}
