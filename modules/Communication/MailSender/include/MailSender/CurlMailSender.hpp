/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#pragma once

#include <curl/curl.h>

#include <memory>
#include <string>

#include "MailSender/Mail.hpp"
#include "MailSender/IMailSender.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace communication {
namespace mailsender {

class CurlMailSender : public IMailSender {
 public:
    CurlMailSender();
    ~CurlMailSender() override = default;
    bool initialize() override;
    bool deinitialize() override;
    bool send(const Mail& mail) override;

 private:
    struct CurlStringData {
        explicit CurlStringData(const std::string& data);
        std::string msg;
        size_t bytesleft;
    };
    std::string buildPayloadText(const Mail& mail);
    std::string generateMessageId();
    std::string dateTimeNow();
    const std::string smtpServer_;
    bool isInitialized_;
    utility::logger::EventLogger logger_;
    std::shared_ptr<CURL> curl_;
    static size_t read_callback(char *buffer, size_t size, size_t nitems, void *userdata);
};

}  // namespace mailsender
}  // namespace communication
}  // namespace crf
