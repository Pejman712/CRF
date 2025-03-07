/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <vector>

#include "MailSender/MailAddress.hpp"

namespace crf {
namespace communication {
namespace mailsender {

class Mail {
 public:
    Mail() = delete;
    explicit Mail(const MailAddress& sender);
    ~Mail() = default;

    bool addReceiver(const MailAddress& receiver);
    bool setSubject(const std::string& subject);
    bool setContent(const std::string& content);

    MailAddress getSender() const;
    std::string getSubject() const;
    std::string getContent() const;
    std::vector<MailAddress> getReceivers() const;

 private:
    MailAddress sender_;
    std::string subject_;
    std::string content_;
    std::vector<MailAddress> receivers_;
};

}  // namespace mailsender
}  // namespace communication
}  // namespace crf
