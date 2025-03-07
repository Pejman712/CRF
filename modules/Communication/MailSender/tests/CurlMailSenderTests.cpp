/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2017
 * 
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "MailSender/CurlMailSender.hpp"

using testing::_;
using testing::Return;

using crf::communication::mailsender::CurlMailSender;
using crf::communication::mailsender::IMailSender;
using crf::communication::mailsender::Mail;
using crf::communication::mailsender::MailAddress;

class CurlMailSenderShould: public ::testing::Test {
 protected:
    CurlMailSenderShould() {
        sut_.reset(new CurlMailSender);
    }
    std::unique_ptr<IMailSender> sut_;
};

TEST_F(CurlMailSenderShould, returnFalseIfInitializedOrDeinitializedTwice) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(CurlMailSenderShould, notSendMailAndReturnFalseIfNotInitialized) {
    MailAddress sender("test@cern.ch");
    MailAddress receiver1("test@cern.ch");
    Mail mail(sender);
    mail.addReceiver(receiver1);
    mail.setSubject("Subj1");
    mail.setContent("Xxx");
    ASSERT_FALSE(sut_->send(mail));
}

TEST_F(CurlMailSenderShould, sendMailAndReturnTrue) {
    ASSERT_TRUE(sut_->initialize());
    MailAddress sender("test@cern.ch");
    MailAddress receiver1("test@cern.ch");
    Mail mail(sender);
    mail.addReceiver(receiver1);
    mail.setSubject("Subj1");
    mail.setContent("Xxx");
    ASSERT_TRUE(sut_->send(mail));
}
