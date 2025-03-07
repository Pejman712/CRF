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

#include <memory>
#include <cstring>
#include <string>
#include <thread>
#include <optional>

#include "RPSensor/AtomtexBDKG24/AtomtexBDKG24.hpp"
#include "RPSensor/AtomtexBDKG24/AtomtexBDKG24CRCCodes.hpp"

namespace crf {
namespace sensors {
namespace rpsensor {

AtomtexBDKG24::AtomtexBDKG24(
    std::shared_ptr<crf::communication::serialcommunication::ISerialCommunication> serial) :
    serial_(serial),
    logger_("AtomtexBDKG24"),
    doseRate_(0),
    cumulativeDose_(0),
    initialized_(false) {
    logger_->debug("CTor");
}

AtomtexBDKG24::~AtomtexBDKG24() {
    logger_->debug("DTor");
    deinitialize();
}

bool AtomtexBDKG24::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        return true;
    }
    if (!serial_->initialize()) {
        logger_->error("Could not initialize serial");
        return false;
    }
    if (!interrogateHardware()) {
        logger_->error("Could not initialize RP sensor");
        return false;
    }
    initialized_ = true;
    return true;
}

bool AtomtexBDKG24::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        return true;
    }
    if (!serial_->deinitialize()) {
        return false;
    }
    initialized_ = false;
    return true;
}

std::optional<float> AtomtexBDKG24::getDoseRate() {
    logger_->debug("getDoseRate");
    if (!initialized_) {
        return std::nullopt;
    }
    if (!interrogateHardware()) {
        return std::nullopt;
    }
    return doseRate_;
}

std::optional<float> AtomtexBDKG24::getCumulativeDose() {
    logger_->debug("getCumulativeDose");
    if (!initialized_) {
        return std::nullopt;
    }
    if (!interrogateHardware()) {
        return std::nullopt;
    }
    return cumulativeDose_;
}

bool AtomtexBDKG24::resetCumulativeDose() {
    logger_->debug("resetCumulativeDose");

    if (!initialized_) {
        return false;
    }

    cumulativeDose_ = 0;

    const unsigned char request[6] = {0x01, 0x05, 0x00, 0x23, 0xFF, 0x00};
    std::string requestString(reinterpret_cast<const char*>(request), 6);
    requestString.append(computeCRC(requestString));

    int written = serial_->write(requestString);
    if (written != static_cast<int>(requestString.length())) {
        logger_->warn("Error writing to serial");
        return false;
    }

    std::string response;
    int read = serial_->read(&response, 8);
    if (read != 8) {
        logger_->warn("Could not reset cumulative dose");
        return false;
    }

    return true;
}

bool AtomtexBDKG24::interrogateHardware() {
    logger_->debug("interrogateHardware");

    std::chrono::milliseconds difference = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - lastHardwareAccess_);
    if (difference < maximunRequestFrecuency_) {
        return true;
    }

    const unsigned char request[6] = {0x01, 0x04, 0x00, 0x02, 0x00, 0x0C};
    std::string requestString(reinterpret_cast<const char*>(request), 6);
    requestString.append(computeCRC(requestString));

    int written = serial_->write(requestString);
    if (written != static_cast<int>(requestString.length())) {
        logger_->warn("Error writing to serial");
        return false;
    }

    std::string response;
    int read = serial_->read(&response, 29);
    if (read != 29) {
        logger_->warn("Error reading from serial");
        return false;
    }

    char doseRateChar[4] = {response[18], response[17], response[16], response[15]};
    std::memcpy(&doseRate_, doseRateChar, 4);
    char cumulativeDoseChar[4] = { response[26], response[25], response[24], response[23] };
    std::memcpy(&cumulativeDose_, cumulativeDoseChar, 4);
    lastHardwareAccess_ = std::chrono::high_resolution_clock::now();
    return true;
}

std::string AtomtexBDKG24::computeCRC(const std::string& request) {
    logger_->debug("computeCRC");
    unsigned char uchCRC[2] = { 0xFF, 0xFF };  // High CRC byte initialized
    unsigned char uIndex = 0;  // Will index into CRC lookup
    int j = 0;
    for (int i = request.length(); i > 0; i--) {
        uIndex = (unsigned char)((unsigned char)uchCRC[0] ^ (unsigned char)request[j]);
        uchCRC[0] = (unsigned char)((unsigned char)uchCRC[1] ^ auchCRCHi[uIndex]);
        uchCRC[1] = auchCRCLo[uIndex];
        j++;
    }
    unsigned char inverted[2] = { uchCRC[1], uchCRC[0] };
    return std::string(reinterpret_cast<char*>(inverted), 2);
}

}  // namespace rpsensor
}  // namespace sensors
}  // namespace crf
