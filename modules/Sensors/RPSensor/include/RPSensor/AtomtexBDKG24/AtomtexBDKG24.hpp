/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Alessandro Mosca CERN EN/STI/ECE 2017
 *          Giacomo Lunghi CERN EN/SMM/MRO 2017
 *          Jorge Camarero Vera CERN EN/STI/ECE 2017
 *          Carlos Veiga Almagro CERN EN/STI/ECE 2017
 *          David Blanco Mulero CERN EN/STI/ECE 2017
 *          Pawel Ptasznik CERN EN/STI/ECE 2017
 * Contributors: Alejandro Diaz Rosales BE/CEM/MRO 2021
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "RPSensor/IRPSensor.hpp"
#include "EventLogger/EventLogger.hpp"
#include "SerialCommunication/ISerialCommunication.hpp"

namespace crf {
namespace sensors {
namespace rpsensor {

class AtomtexBDKG24 : public IRPSensor {
 public:
    explicit AtomtexBDKG24(
        std::shared_ptr<communication::serialcommunication::ISerialCommunication> serial);
    AtomtexBDKG24() = delete;
    AtomtexBDKG24(const AtomtexBDKG24&) = delete;
    AtomtexBDKG24(AtomtexBDKG24&&) = delete;
    ~AtomtexBDKG24() override;

    bool initialize() override;
    bool deinitialize() override;

    std::optional<float> getDoseRate() override;
    std::optional<float> getCumulativeDose() override;
    bool resetCumulativeDose() override;

 private:
    std::shared_ptr<communication::serialcommunication::ISerialCommunication> serial_;
    utility::logger::EventLogger logger_;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastHardwareAccess_;
    const std::chrono::milliseconds maximunRequestFrecuency_ = std::chrono::milliseconds(50);
    float doseRate_;
    float cumulativeDose_;
    bool initialized_;

    bool interrogateHardware();
    std::string computeCRC(const std::string&);
};

}  // namespace rpsensor
}  // namespace sensors
}  // namespace crf
