/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <cstdint>
#include <vector>

#include <boost/crc.hpp>

#include "LeakDetector/PhoenixTelegram.hpp"

namespace crf {
namespace sensors {
namespace leakdetector {

/*
 * Structure of the master telegram:
 * ENQ | LEN | ADR | CmdH | CmdL | DATA (n bytes) | CRC
 *  0  |  1  |  2  |  3   |  4   |  5             | 5+n
 *
 * For more info see 3.2.1 section in the manual
 */
#define MASTER_ENQ 0
#define START_MASTER_REQ_VAL 0x05
#define MASTER_LEN 1
#define MASTER_ADR 2
#define MASTER_CmdH 3
#define MASTER_CmdL 4
#define MASTER_DATA 5
/*
 * Structure of the slave telegram:
 * STX | LEN | StwH | StwL | CmdH | CmdL | DATA (n bytes) | CRC
 *  0  |  1  |  2   |  3   |  4   |  5   |  6             | 6+n
 *
 * For more info see 3.2.1 section in the manual
 */
#define SLAVE_STX 0
#define START_SLAVE_REP_VAL 0x02
#define SLAVE_LEN 1
#define SLAVE_StwH 2
#define SLAVE_StwL 3
#define SLAVE_CmdH 4
#define SLAVE_CmdL 5
#define SLAVE_DATA 6

#define CRC_POLYNOMIAL 0x31

#define TELEGRAM_LENGTH_WITHOUT_DATA 6

#define FLOAT_SLAVE_RESPONSE_LENGTH 11

namespace {
    std::string errNumberToDebugStr(uint8_t num) {
        switch (num) {
            case 1: return "CRC-failure";
            case 2: return "Illegal telegram lenght";
            case 10: return "command doesn't exist";
            case 11: return "Data length is not correct for the command";
            case 12: return "Read not allowed";
            case 13: return "Write not allowed";
            case 14: return "Array-Index out of range or missing";
            case 20: return "Control actually not allowed with this interface";
            case 21: return "Password not OK";
            case 22: return "Command actually not allowed (e.g. calibration during Run-Up)";
            case 30: return "Data not in range";
            case 31: return "No data available";
            default : return "Unrecognized error number";
        }
    }
}  // unnamed namespace

PhoenixTelegram::PhoenixTelegram(): logger_("PhoenixTelegram") {
    logger_->debug("CTor");
    uint8_t write = 0x20;
    uint8_t read = 0x00;
    std::array<uint8_t, 2> cmd;
    cmd[0] = write;
    cmd[1] = 0x01;
    cmdMap_[start] = cmd;
    cmd[0] = write;
    cmd[1] = 0x02;
    cmdMap_[stop] = cmd;
    cmd[0] = write;
    cmd[1] = 0x05;
    cmdMap_[clearError] = cmd;
    cmd[0] = read;
    cmd[1] = 0x81;
    cmdMap_[leakRateMbarLS] = cmd;
    cmd[0] = read;
    cmd[1] = 0x82;
    cmdMap_[internalPressure1SelUnit] = cmd;
    cmd[0] = read;
    cmd[1] = 0x83;
    cmdMap_[internalPressure1Mbar] = cmd;
}

std::vector<uint8_t> PhoenixTelegram::makeTelegram(Cmd cmd) {
    logger_->debug("makeTelegram: {}", cmd);
    int n = 0;  // 0 bytes of additional data
    std::vector<uint8_t> telegram;
    telegram.resize(TELEGRAM_LENGTH_WITHOUT_DATA);
    telegram[MASTER_ENQ] = START_MASTER_REQ_VAL;
    telegram[MASTER_LEN] = TELEGRAM_LENGTH_WITHOUT_DATA - 2;  // without ENQ and LEN
    telegram[MASTER_ADR] = 0;  // whatever, I think it is neglected
    telegram[MASTER_CmdH] = cmdMap_[cmd][0];  // CmdH
    telegram[MASTER_CmdL] = cmdMap_[cmd][1];  // CmdL
    telegram[MASTER_DATA + n] = calculateCrc(telegram.data(), telegram.size()-1);
    logger_->debug("CRC: {}", (unsigned)telegram[MASTER_DATA + n]);
    return telegram;
}

bool PhoenixTelegram::checkSlaveResponse(const std::vector<uint8_t>& telegram) {
    if (telegram.size() < SLAVE_DATA) {
        logger_->error("Telegram too short!");
        return false;
    }
    if (telegram[SLAVE_STX] != START_SLAVE_REP_VAL) {
        logger_->error("Slave response should start with {}, instead it is {}",
            START_SLAVE_REP_VAL, telegram[SLAVE_STX]);
        return false;
    }
    uint8_t len = telegram[SLAVE_LEN];
    if (telegram.size() != len + 2) {
        logger_->error("Telegram length invalid, should be: {}, is: {}",
            static_cast<int>(len) + 2, telegram.size());
        return false;
    }
    if ((telegram[SLAVE_StwH] & 0x80) == 0x80) {
        logger_->warn("Slave replied with Syntax/Command error: {}",
            errNumberToDebugStr(telegram[SLAVE_DATA]));
        return false;
    }
    if (telegram.back() != calculateCrc(telegram.data(), telegram.size()-1)) {
        logger_->warn("Invalid checksum");
        return false;
    }
    return true;
}

float PhoenixTelegram::getFloatFromTelegram(const std::vector<uint8_t>& telegram) {
    if (telegram.size() != FLOAT_SLAVE_RESPONSE_LENGTH) {
        logger_->error("Msg size is {} instead of {}", telegram.size(),
            FLOAT_SLAVE_RESPONSE_LENGTH);
        return 0;
    }
    float* tmp = new float();
    uint8_t* ptr = reinterpret_cast<uint8_t*>(tmp);
    int telegramFloatOffset = 6;
    for (int i=3; i >= 0; i--) {
        *ptr = telegram[telegramFloatOffset+i];
        ptr++;
    }
    float retVal = *tmp;
    delete tmp;
    return retVal;
}

uint8_t PhoenixTelegram::calculateCrc(const uint8_t* data, int numBytes) {
    boost::crc_optimal <8, CRC_POLYNOMIAL, 0, 0, true, true> crc;
    crc.process_bytes(data, numBytes);
    return crc.checksum();
}

}  // namespace leakdetector
}  // namespace sensors
}  // namespace crf
