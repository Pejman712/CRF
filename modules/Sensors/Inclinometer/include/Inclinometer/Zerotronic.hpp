#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "SerialCommunication/SerialCommunication.hpp"
#include "Inclinometer/IInclinometer.hpp"

namespace crf {
namespace sensors {
namespace inclinometer {

class Zerotronic: public IInclinometer {
 public:
    Zerotronic() = delete;
    explicit Zerotronic(
        std::shared_ptr<communication::serialcommunication::ISerialCommunication> sComm);
    Zerotronic(const IInclinometer& other) = delete;  // NOLINT
    Zerotronic(IInclinometer&& other) = delete;  // NOLINT
    ~Zerotronic() override;
    bool initialize() override;
    bool deinitialize() override;
    std::vector<double> getInclination() override;
 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<communication::serialcommunication::ISerialCommunication> sComm_;
    bool initialized_;
};

}  // namespace inclinometer
}  // namespace sensors
}  // namespace crf
