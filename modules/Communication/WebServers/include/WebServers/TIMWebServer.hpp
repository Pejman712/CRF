#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <future>
#include <map>
#include <memory>
#include <restbed>
#include <string>

#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"
#include "TIM/ITIM.hpp"
#include "WebServers/TIM/TIMWebPoint.hpp"

namespace crf {
namespace communication {
namespace webservers {

struct HttpResponse {
    int status;
    std::string contentType;
    std::string bytes;
};

class TIMWebServer : public utility::commoninterfaces::IInitializable {
 public:
    TIMWebServer() = delete;
    explicit TIMWebServer(const std::string& serverConfigFile);
    TIMWebServer(TIMWebServer&&) = delete;
    TIMWebServer(const TIMWebServer&) = delete;
    ~TIMWebServer() override;

    bool initialize() override;
    bool deinitialize() override;

    bool addTIM(int id, std::shared_ptr<TIMWebPoint> tim);

 private:
    utility::logger::EventLogger logger_;

    std::mutex mutex_;
    std::condition_variable condVar_;
    bool initialized_;
    std::atomic<bool> serviceStarted_;
    std::atomic<bool> closing_;

    int port_;
    int initalizeTimeoutSec_;
    std::string context_;

    std::future<bool> serverReturnValue_;
    restbed::Service server_;
    std::shared_ptr<restbed::Settings> settings_;

    std::map<int, std::shared_ptr<TIMWebPoint> > tims_;

    bool parseConfigFile(const std::string& serverConfigFile);
    void serviceReady(restbed::Service&);
};

}  // namespace webservers
}  // namespace communication
}  // namespace crf
