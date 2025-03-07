/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <memory>
#include <csignal>
#include <vector>
#include <fstream>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "SerialCommunication/SerialCommunication.hpp"
#include "FourierTransform/FFT.hpp"
#include "Radars/FraunhoferRadar/FraunhoferRadar.hpp"

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

    crf::math::fouriertransform::FFT fft;
    crf::sensors::fraunhoferradar::FraunhoferRadar radar(jConfig, serial);
    radar.initialize();

    while (gSignalStatus != SIGTSTP) {
        std::vector<std::vector<float>> radarData = radar.getFrame();
        if (radarData.empty()) {
            logger->debug("radar packet empty, smth wrong");
            return -1;
        }
        std::vector<std::complex<float>> fftData = fft.getFFT(radarData[0], 5, true);
    }
}
