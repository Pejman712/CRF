/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <csignal>
#include <memory>
#include <fstream>
#include <thread>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "SerialCommunication/SerialCommunication.hpp"
#include "PeakDetection/CFAR.hpp"
#include "PeakDetection/GradientPeakDetection.hpp"
#include "Radars/FraunhoferRadar/FraunhoferRadar.hpp"
#include "StateEstimator/DefaultMeasurementModel.hpp"
#include "StateEstimator/DefaultSystemModel.hpp"
#include "StateEstimator/StateEstimator.hpp"
#include "HealthDetection/HealthDetection.hpp"

#define STATEVECTORSIZE 1
#define MEASUREMENTVECTORSIZE 1

volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    gSignalStatus = signal;
}

int main(int argc, char *argv[]) {
    crf::utility::logger::EventLogger logger("Radar Sample");
    if (argc < 3) {
        logger->error(R"(ArgCount is less than expected. Command example for use:
            [1] radar config file path
            [2] serial port (e.g. /dev/ttyACM0)
            [3] folder where to save recordings (e.g. /tmp/radarData/ automatically creates folder)
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
    auto cfar =
        std::make_shared<crf::algorithms::peakdetection::CFAR>(10, 40, 0.001);
    auto heartPeakDetector =
        std::make_shared<crf::algorithms::peakdetection::GradientPeakDetection>(0.1);
    auto respirationPeakDetector =
        std::make_shared<crf::algorithms::peakdetection::GradientPeakDetection>(0.5);
    auto radar =
        std::make_shared<crf::sensors::fraunhoferradar::FraunhoferRadar>(jConfig, serial);
    auto measurementModel =
        std::make_shared<crf::algorithms::stateestimator::DefaultMeasurementModel<
        STATEVECTORSIZE, MEASUREMENTVECTORSIZE>>();
    auto systemModel =
        std::make_shared<crf::algorithms::stateestimator::DefaultSystemModel<STATEVECTORSIZE>>();
    auto type = crf::algorithms::stateestimator::StateEstimatorFilterType::UNSCENTED_KF;
    std::array<float, STATEVECTORSIZE> initialState{0};
    auto stateEstimator = std::make_shared<crf::algorithms::stateestimator::StateEstimator<
        STATEVECTORSIZE, MEASUREMENTVECTORSIZE>>
        (initialState, type, measurementModel, systemModel);

    std::string recordDir;
    if (argc > 3) {
        recordDir = argv[3];
    } else {
        recordDir = "none";
    }

    auto healthDetector = std::make_unique<crf::applications::healthdetection::HealthDetection>(
        radar, cfar, heartPeakDetector, respirationPeakDetector, stateEstimator, recordDir);
    if (!healthDetector->initialize()) {
        logger->error("Could not initialize");
        return -1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(10));
    while (gSignalStatus != SIGTSTP) {
        auto vitalSignPacket = healthDetector->getVitalSigns();
        if (!vitalSignPacket) {
            logger->info("Respiration not detected");
        } else {
            logger->info("RR: {} bpm, HR: ", vitalSignPacket.get().respirationRate,
                vitalSignPacket.get().heartRate);
        }
    }
}
