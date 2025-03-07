/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE 2017
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "MailSender/MailAddress.hpp"

using testing::_;
using testing::Return;

using crf::communication::mailsender::MailAddress;

class MailAddressShould: public ::testing::Test {
 protected:
    std::unique_ptr<MailAddress> address_;
};

TEST_F(MailAddressShould, copyConstructorTest) {
    MailAddress address1("ciao@ciao.ciao");
    MailAddress address2(address1);

    ASSERT_TRUE(address1.getAddress() == address2.getAddress());
}

TEST_F(MailAddressShould, initializeEmailAddressTests) {
    std::vector<std::string> goodAddresses;
    goodAddresses.push_back("prova@cern.ch");
    goodAddresses.push_back("name.surname@cern.ch");
    goodAddresses.push_back("name.middlename.surname@cern.ch");

    std::vector<std::string> badAddresses;
    badAddresses.push_back("provacern.ch");
    badAddresses.push_back("prova@csernch");
    badAddresses.push_back("provacernch");
    badAddresses.push_back("##@cern.ch");

    for (int i=0; i < goodAddresses.size(); i++) {
        ASSERT_NO_THROW(address_.reset(new MailAddress(goodAddresses[i])));
        ASSERT_TRUE(address_->getAddress() == goodAddresses[i]);
    }

    for (int i=0; i < badAddresses.size(); i++) {
        ASSERT_THROW(address_.reset(new MailAddress(badAddresses[i])), std::invalid_argument);
    }
}
