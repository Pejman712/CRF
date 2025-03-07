/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "CANSocket/ICANSocket.hpp"
#include "CANOpenDevices/ICANOpenDevice.hpp"
#include "CANOpenDevices/ICANOpenContext.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

class CANOpenContext : public ICANOpenContext {
 public:
    CANOpenContext() = delete;
    explicit CANOpenContext(std::shared_ptr<communication::cansocket::ICANSocket>);
    CANOpenContext(const CANOpenContext&) = delete;
    CANOpenContext(CANOpenContext&&) = delete;

    ~CANOpenContext() override;

    bool initialize() override;
    bool deinitialize() override;

    bool addDevice(std::shared_ptr<ICANOpenDevice>) override;

    bool sendSync() override;
    bool sendGuard() override;
    bool setSyncFrequency(const std::chrono::milliseconds& frequency) override;
    bool setGuardFrequency(const std::chrono::milliseconds& frequency) override;

 private:
    utility::logger::EventLogger logger_;

    std::shared_ptr<communication::cansocket::ICANSocket> socket_;
    std::map<int, std::shared_ptr<ICANOpenDevice> > devices_;

    bool initialized_;
    std::atomic<bool> stopThreads_;

    std::thread receiverThread_;
    void receiver();

    std::atomic<bool> sendSync_;
    std::chrono::milliseconds syncFrequency_;
    std::atomic<bool> sendGuard_;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastGuardSent_;
    std::chrono::milliseconds guardFrequency_;
    std::thread syncGuardSenderThread_;
    void syncGuardSender();
};

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
