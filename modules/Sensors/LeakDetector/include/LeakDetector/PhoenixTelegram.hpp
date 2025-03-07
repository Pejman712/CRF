#pragma once

/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <array>
#include <cstdint>
#include <map>
#include <vector>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace leakdetector {

class PhoenixTelegram {
 public:
    PhoenixTelegram();
    enum Cmd {
        start = 1,
        stop = 2,
        clearError = 5,
        leakRateMbarLS = 129,
        internalPressure1SelUnit = 130,
        internalPressure1Mbar = 131
    };
    std::vector<uint8_t> makeTelegram(Cmd cmd);
    bool checkSlaveResponse(const std::vector<uint8_t>& telegram);
    float getFloatFromTelegram(const std::vector<uint8_t>& telegram);

 private:
    uint8_t calculateCrc(const uint8_t* data, int numBytes);
    utility::logger::EventLogger logger_;
    std::map<Cmd, std::array<uint8_t, 2> > cmdMap_;
};

}  // namespace leakdetector
}  // namespace sensors
}  // namespace crf
