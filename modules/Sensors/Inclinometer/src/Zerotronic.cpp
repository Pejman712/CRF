/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>
#include <string>
#include <thread>

#include <boost/algorithm/string.hpp>

#include "SerialCommunication/SerialCommunication.hpp"
#include "Inclinometer/Zerotronic.hpp"

/*
 * With baud rate 9600 we can expect one byte to be transmitted in less than 1 ms
 * Let's wait for 1ms, it should be enough
 */
#define SERIAL_WRITE_DELAY_MS 100
#define MAX_BUFF_SIZE 256

using crf::communication::serialcommunication::ISerialCommunication;

namespace crf {
namespace sensors {
namespace inclinometer {

Zerotronic::Zerotronic(std::shared_ptr<ISerialCommunication> sComm):
    logger_("Zerotronic"),
    sComm_(sComm),
    initialized_(false) {
        logger_->debug("CTor");
}

Zerotronic::~Zerotronic() {
    logger_->debug("DTor");
}

bool Zerotronic::initialize() {
    logger_->info("initialize");
    if (initialized_) {
        return false;
    }
    if (!sComm_->initialize()) {
        return false;
    }
    initialized_ = true;
    return true;
}

bool Zerotronic::deinitialize() {
    logger_->info("deinitialize");
    if (!initialized_) {
        return false;
    }
    if (!sComm_->deinitialize()) {
        return false;
    }
    initialized_ = false;
    return true;
}

std::vector<double> Zerotronic::getInclination() {
    std::string command("P");
    if (sComm_->write(command) == -1) {
        logger_->warn("Failed to write command to inclinometer");
        return std::vector<double>();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(SERIAL_WRITE_DELAY_MS));
    std::string buff;
    sComm_->read(&buff, MAX_BUFF_SIZE);
    std::vector<std::string> splittedMessage;
    boost::algorithm::split(splittedMessage, buff, boost::is_any_of(" "));
    double v1, v2;
    if (splittedMessage.size() < 5) {
        logger_->warn("Expected to receive at least 5 different values");
        logger_->warn("Instead got msg: {}, spited into {} parts",
            buff, splittedMessage.size());
        return std::vector<double>();
    }
    try {
        v1 = std::stod(splittedMessage[2]);
        v2 = std::stod(splittedMessage[4]);
    } catch (const std::exception& e) {
        logger_->warn("Exception occurred during floating point conversions: {}", e.what());
        return std::vector<double>();
    }
    return std::vector<double>({v1, v2});
}

}  // namespace inclinometer
}  // namespace sensors
}  // namespace crf
