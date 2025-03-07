#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <sys/time.h>
#include <memory>
#include <thread>

#include "IPC/NetworkIPC.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace communication {
namespace timesynchronizationserver {

class TimeSynchronizationServer : public utility::commoninterfaces::IInitializable {
 public:
    TimeSynchronizationServer() = delete;
    explicit TimeSynchronizationServer(
        const std::shared_ptr<IPC>& ipc);
    ~TimeSynchronizationServer() override;

    bool initialize() override;
    bool deinitialize() override;
 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<IPC> ipc_;
    std::atomic<bool> stopThread_;
    std::thread runThread_;
    void run();
};

}  // namespace timesynchronizationserver
}  // namespace communication
}  // namespace crf
