/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <iostream>
#include <errno.h>
#include <memory>
#include <string.h>
#include <string>
#include <thread>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <curl/curl.h>

#include <IPC/IPC.hpp>
#include <TeltonikaRUT/TeltonikaRUTPackets.hpp>
#include <CommonInterfaces/IInitializable.hpp>

namespace crf {
namespace devices {

using json = nlohmann::json;

class TeltonikaRUT : commoninterfaces::IInitializable {
 public:
    TeltonikaRUT() = delete;
    TeltonikaRUT(std::string ip_address, int port, std::string username, std::string password,
        std::shared_ptr<IPC> outputIPC);
    ~TeltonikaRUT() = default;

    bool initialize() override;
    bool deinitialize() override;

    void joinThreads();

    inline std::string getCellName() { return statusPacket.cellName; }
    inline std::string getConnectionType() { return statusPacket.connectionType; }
    inline float getLTErsrq () { return statusPacket.LTErsrq; }
    inline float getLTEsinr () { return statusPacket.LTEsinr; }
    inline float getLTErsrp () { return statusPacket.LTErsrp; }
    inline float getWCDMAecic() { return statusPacket.WCDMAecic; }
    inline float getWCDMArscp() { return statusPacket.WCDMArscp; }
    inline float getGSMSignalStrength() { return statusPacket.GSMSignalStrength; }
    inline long getGSMBytesReceived () { return statusPacket.GSMBytesReceived; } // NOLINT
    inline long getGSMBytesSent() { return statusPacket.GSMBytesSent; } // NOLINT

 private:
    std::shared_ptr<IPC> outputIPC;
    CURL* curl;
    struct curl_slist *headers;

    std::string ip_address_;
    int port_;

    std::string session_id_;
    std::string username_;
    std::string password_;

    Packets::GSMInfoStatusPacket statusPacket;

    volatile bool _stop;

    std::thread updateDataThread;
    void updateSignalInfo();
};

}  // namespace devices
}  // namespace crf
