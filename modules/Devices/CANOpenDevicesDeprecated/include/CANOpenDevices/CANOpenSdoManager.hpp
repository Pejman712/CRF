/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <optional>

#include "CANOpenDevices/ObjectDictionary.hpp"
#include "CANSocket/ICANSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

class CANOpenSdoManager {
 public:
    CANOpenSdoManager() = delete;
    CANOpenSdoManager(uint8_t id,
        std::shared_ptr<communication::cansocket::ICANSocket> socket,
        std::shared_ptr<ObjectDictionary> dictionary,
        const std::chrono::milliseconds& sdoResponseTimeout);

    template<typename T>
    std::optional<T> readRegister(const std::string& name);

    template<typename T>
    std::optional<T> readRegister(uint16_t index, uint8_t subindex);

    template<typename T>
    bool writeRegister(const std::string& name, T value);

    template<typename T>
    bool writeRegister(uint16_t index, uint8_t subindex, T value);

 private:
    crf::utility::logger::EventLogger logger_;
    uint8_t id_;
    std::shared_ptr<communication::cansocket::ICANSocket> socket_;
    std::shared_ptr<ObjectDictionary> dictionary_;
    const std::chrono::milliseconds sdoResponseTimeout_;

    can_frame buildSdoReadFrame(const ObjectDictionaryRegister& reg);

    template<typename T>
    can_frame buildSdoWriteFrame(const ObjectDictionaryRegister& reg, T value);
};

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
