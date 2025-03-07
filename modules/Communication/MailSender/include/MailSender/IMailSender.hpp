/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#pragma once

#include "CommonInterfaces/IInitializable.hpp"
#include "MailSender/Mail.hpp"

namespace crf {
namespace communication {
namespace mailsender {

class IMailSender : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IMailSender() = default;

    virtual bool initialize() = 0;
    virtual bool deinitialize() = 0;
    virtual bool send(const Mail& mail) = 0;
};

}  // namespace mailsender
}  // namespace communication
}  // namespace crf
