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

namespace crf {
namespace communication {
namespace mailsender {

class MailAddress {
 public:
    MailAddress() = delete;
    explicit MailAddress(const std::string& address);
    ~MailAddress() = default;

    std::string getAddress() const;

 private:
    std::string address_;
};

}  // namespace mailsender
}  // namespace communication
}  // namespace crf
