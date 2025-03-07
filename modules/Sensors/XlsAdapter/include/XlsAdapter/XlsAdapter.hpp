#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "NetworkClient/INetworkClient.hpp"
#include "XlsAdapter/IXlsAdapter.hpp"

#include "XlsAdapter/XlsMessage.hpp"

namespace crf {
namespace sensors {
namespace xlsadapter {

/*
 * Class for communication with XLS Sensors using TCP/IP protocol
 *
 * Basic features:
 *  - initialize()/deinitialize() to/from sensors on a given IP and port using NetworkClient;
 *  - getData() command subscribes for the transmission service will continue until client is disconnected
 *    or an unsubscribe command is sent
 *  - changeMode() sends appropriate code to the device to switch modes
 *  - initialized_ flag tells if connection to the device has been established correctly
 *
 */
class XlsAdapter: public IXlsAdapter {
 public:
    XlsAdapter() = delete;
    XlsAdapter(const XlsAdapter&) = delete;
    XlsAdapter(XlsAdapter&&) = delete;
    explicit XlsAdapter(
        std::shared_ptr<communication::networkclient::INetworkClient> tcpCommPoint);
    ~XlsAdapter() override;
    bool initialize() override;
    bool deinitialize() override;
    std::vector<float> getData() override;
    bool changeMode(Mode m) override;
 private:
    std::string getCurrentMode();
    utility::logger::EventLogger logger_;
    std::shared_ptr<communication::networkclient::INetworkClient> tcpCommPoint_;
    bool initialized_;
};

}  // namespace xlsadapter
}  // namespace sensors
}  // namespace crf
