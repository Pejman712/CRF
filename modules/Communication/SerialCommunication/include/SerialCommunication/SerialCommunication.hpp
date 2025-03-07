/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2018
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include "EventLogger/EventLogger.hpp"
#include "SerialCommunication/ISerialCommunication.hpp"

namespace crf {
namespace communication {
namespace serialcommunication {

class SerialCommunication: public ISerialCommunication {
 public:
    SerialCommunication() = delete;
    SerialCommunication(const std::string& deviceName, int baudRate,
        bool blocking = false, bool parityBit = false, int charSize = 8,
        std::chrono::duration<float> timeout = std::chrono::duration<float>(0));
    SerialCommunication(const SerialCommunication& other) = delete;
    SerialCommunication(SerialCommunication&& other) = delete;
    ~SerialCommunication() override;

    bool initialize() override;
    bool deinitialize() override;
    int read(std::string* buff, int length) override;
    int write(const std::string& buff) override;

 private:
    utility::logger::EventLogger logger_;
    std::string deviceName_;
    int baudRate_;
    bool blocking_;
    bool parityBit_;
    int charSize_;
    std::chrono::duration<float> timeout_;
    int serialFd_;
};

}  // namespace serialcommunication
}  // namespace communication
}  // namespace crf
