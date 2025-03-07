/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE 2017
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "MailSender/MailAddress.hpp"
#include "MailSender/Mail.hpp"

using testing::_;
using testing::Return;

using crf::communication::mailsender::MailAddress;
using crf::communication::mailsender::Mail;

class MailShould: public ::testing::Test {
 protected:
};

TEST_F(MailShould, correctlyAssignMethodsTests) {
    MailAddress sender("sender@ciao.ciao");

    MailAddress receiver1("receiver1@ciao.ciao");
    MailAddress receiver2("receiver2@ciao.ciao");
    MailAddress receiver3("receiver3@ciao.ciao");

    Mail mail(sender);

    mail.addReceiver(receiver1);
    mail.addReceiver(receiver2);
    mail.addReceiver(receiver3);
    mail.setSubject("Ciao");
    mail.setContent("CiaoCiaoCiao");

    ASSERT_EQ(mail.getSender().getAddress(), "sender@ciao.ciao");
    ASSERT_EQ(mail.getSubject(), "Ciao");
    ASSERT_EQ(mail.getContent(), "CiaoCiaoCiao");
    ASSERT_EQ(mail.getReceivers().size(), 3);
}
