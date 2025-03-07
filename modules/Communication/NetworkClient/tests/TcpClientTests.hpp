#pragma once

/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "EventLogger/EventLogger.hpp"

#include "Mocks/Utility/SocketInterfaceMock.hpp"

class TcpClientShould: public ::testing::Test {
 public:
    int assignAddrinfo(const char*, const char*, const struct addrinfo*, struct addrinfo **res);
    int assignDataToTheReceiver(int, void* buff, size_t length, int);
 protected:
    TcpClientShould();
    ~TcpClientShould();

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<testing::NiceMock<crf::utility::commutility::SocketInterfaceMock> >
        socketInterfaceMock_;
    int somePort_;
    int someClientSocket_;
    std::string someAddr_;
    struct addrinfo addrinfo_;
    struct sockaddr sockaddr_;
    std::unique_ptr<crf::communication::networkclient::INetworkClient> sut_;
};
