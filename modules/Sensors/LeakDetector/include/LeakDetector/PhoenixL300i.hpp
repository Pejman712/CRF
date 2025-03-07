#pragma once

/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <cstdint>
#include <memory>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "IPC/IPC.hpp"
#include "SerialCommunication/SerialCommunication.hpp"

#include "LeakDetector/ILeakDetector.hpp"
#include "LeakDetector/PhoenixTelegram.hpp"

namespace crf {
namespace sensors {
namespace leakdetector {

class PhoenixL300i: public ILeakDetector {
 public:
    PhoenixL300i() = delete;
    PhoenixL300i(std::shared_ptr<IPC> ipc,
        std::shared_ptr<communication::serialcommunication::ISerialCommunication> sComm);
    PhoenixL300i(const PhoenixL300i&) = delete;
    PhoenixL300i(PhoenixL300i&&) = delete;
    ~PhoenixL300i() override;

    float getLeakRate() override;
    float getInternalPressure() override;
    bool clearError() override;

 private:
    bool sendTelegramAndWaitALittleBit(const std::vector<uint8_t>& telegram);
    std::vector<uint8_t> getSlaveResponse();
    utility::logger::EventLogger logger_;
    std::shared_ptr<IPC> ipc_;
    std::shared_ptr<communication::serialcommunication::ISerialCommunication> sComm_;
    PhoenixTelegram telegram_;
};

}  // namespace leakdetector
}  // namespace sensors
}  // namespace crf
