/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#include <regex>
#include <string>

#include "MailSender/MailAddress.hpp"

namespace crf {
namespace communication {
namespace mailsender {

MailAddress::MailAddress(const std::string& address) {
    const std::regex pattern("^[a-zA-Z0-9._-]+@[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])+\\.[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])*$");  // NOLINT
    if (!std::regex_match(address, pattern))
        throw std::invalid_argument("Not valid email address");
    address_ = address;
}

std::string MailAddress::getAddress() const {
    return address_;
}

}  // namespace mailsender
}  // namespace communication
}  // namespace crf
