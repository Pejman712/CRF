/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include "LeakDetector/PhoenixL300i.hpp"

namespace {
std::ostream& operator<<(std::ostream& stream, const std::vector<uint8_t>& telegram) {
    stream << "[";
    for (auto byte : telegram) {
        stream << std::hex << static_cast<unsigned>(byte) << ", ";
    }
    stream << "]";
    return stream;
}
}  // unnamed namespace

namespace crf {
namespace sensors {
namespace leakdetector {

#define SLAVE_MAX_RESPONSE_LEN 255

PhoenixL300i::PhoenixL300i(std::shared_ptr<IPC> ipc,
    std::shared_ptr<communication::serialcommunication::ISerialCommunication> sComm):
    logger_("PhoenixL300i"),
    ipc_(ipc),
    sComm_(sComm),
    telegram_() {
    logger_->debug("CTor");
    std::vector<uint8_t> initMsg = telegram_.makeTelegram(PhoenixTelegram::start);
    std::string initMsgStr(initMsg.begin(), initMsg.end());
    logger_->debug("Going to send initial telegram: {}", initMsgStr);
    sComm_->write(std::string(reinterpret_cast<char*>(initMsg.data()), initMsg.size()));
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(100));
    std::vector<uint8_t> slaveResponse = getSlaveResponse();
    std::string slaveResponseStr(slaveResponse.begin(), slaveResponse.end());
    logger_->debug("Slave replied: {}", slaveResponseStr);
    if (!telegram_.checkSlaveResponse(slaveResponse)) {
        logger_->warn("Slave replied with ERR to the initial msg");
    }
}

PhoenixL300i::~PhoenixL300i() {
    logger_->debug("DTor");
}

float PhoenixL300i::getLeakRate() {
    logger_->debug("getLeakRate");
    std::vector<uint8_t> msg = telegram_.makeTelegram(PhoenixTelegram::leakRateMbarLS);
    sendTelegramAndWaitALittleBit(msg);
    std::vector<uint8_t> slaveResponse = getSlaveResponse();
    std::string slaveResponseStr(slaveResponse.begin(), slaveResponse.end());
    logger_->debug("Slave replied: {}", slaveResponseStr);
    if (!telegram_.checkSlaveResponse(slaveResponse)) {
        logger_->warn("Slave repplied with ERR, returning 0");
        return 0;
    }
    return telegram_.getFloatFromTelegram(slaveResponse);
}

float PhoenixL300i::getInternalPressure() {
    logger_->debug("getInternalPressure");
    std::vector<uint8_t> msg = telegram_.makeTelegram(PhoenixTelegram::internalPressure1Mbar);
    sendTelegramAndWaitALittleBit(msg);
    std::vector<uint8_t> slaveResponse = getSlaveResponse();
    std::string slaveResponseStr(slaveResponse.begin(), slaveResponse.end());
    logger_->debug("Slave replied: {}", slaveResponseStr);
    if (!telegram_.checkSlaveResponse(slaveResponse)) {
        logger_->warn("Slave repplied with ERR, returning 0");
        return 0;
    }
    return telegram_.getFloatFromTelegram(slaveResponse);
}

bool PhoenixL300i::clearError() {
    logger_->debug("clearError");
    std::vector<uint8_t> msg = telegram_.makeTelegram(PhoenixTelegram::clearError);
    sendTelegramAndWaitALittleBit(msg);
    std::vector<uint8_t> slaveResponse = getSlaveResponse();
    std::string slaveResponseStr(slaveResponse.begin(), slaveResponse.end());
    logger_->debug("Slave replied: {}", slaveResponseStr);
    if (!telegram_.checkSlaveResponse(slaveResponse)) {
        logger_->warn("Slave repplied with ERR, returning FALSE");
        return false;
    }
    return true;
}

bool PhoenixL300i::sendTelegramAndWaitALittleBit(const std::vector<uint8_t>& telegram) {
    sComm_->write(std::string((char*)telegram.data(), telegram.size())); // NOLINT
    // TODO: think about a better waiting time NOLINT
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(100));
    return true;
}

std::vector<uint8_t> PhoenixL300i::getSlaveResponse() {
    std::string buffer;
    if (sComm_->read(&buffer, SLAVE_MAX_RESPONSE_LEN) == -1) {
        logger_->warn("Failed to read slave response");
        return std::vector<uint8_t>();
    }
    uint8_t telegramLen = buffer[1] + 2;
    std::vector<uint8_t> slaveResponse(buffer.data(), buffer.data()+telegramLen);
    return slaveResponse;
}

}  // namespace leakdetector
}  // namespace sensors
}  // namespace crf
