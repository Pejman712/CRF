/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#include <vector>
#include <string>

#include "MailSender/Mail.hpp"

namespace crf {
namespace communication {
namespace mailsender {

Mail::Mail(const MailAddress& sender) :
    sender_(sender),
    subject_(),
    content_(),
    receivers_() {
}

bool Mail::addReceiver(const MailAddress& receiver) {
    receivers_.push_back(receiver);
    return true;
}

bool Mail::setSubject(const std::string& subject) {
    subject_ = subject;
    return true;
}

bool Mail::setContent(const std::string& content) {
    content_ = content;
    return true;
}

MailAddress Mail::getSender() const {
    return sender_;
}

std::string Mail::getSubject() const {
    return subject_;
}

std::string Mail::getContent() const {
    return content_;
}

std::vector<MailAddress> Mail::getReceivers() const {
    return receivers_;
}

}  // namespace mailsender
}  // namespace communication
}  // namespace crf
