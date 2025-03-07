/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2017
 *
 *  ==================================================================================================
 */

#include <sys/stat.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <future>
#include <string>
#include <thread>

#include "EventLogger/EventLogger.hpp"
#include "SerialCommunication/SerialCommunication.hpp"

using crf::utility::logger::EventLogger;
using crf::communication::serialcommunication::ISerialCommunication;
using crf::communication::serialcommunication::SerialCommunication;

/*
 * Linux null modem emulator is required to test some functionalities of this SerialCommunication.
 * If the appropriate devices are not found in the system, some tests will be aborted and just
 * return success.
 */
namespace {
bool fileExists(const std::string& name) {
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}
}  // unnamed namespace

class SerialCommunicationShould: public ::testing::Test {
 protected:
    SerialCommunicationShould(): logger_("SerialCommunicationShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        device0Filename_ = "/dev/tnt0";
        device1Filename_ = "/dev/tnt1";
    }
    ~SerialCommunicationShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    EventLogger logger_;
    std::string device0Filename_;
    std::string device1Filename_;
    std::unique_ptr<ISerialCommunication> sut_;
};

TEST_F(SerialCommunicationShould, returnFalseIfInitializedOrDeinitializedTwice) {
    if (!fileExists(device0Filename_)) {
        return;
    }
    sut_.reset(new SerialCommunication(device0Filename_, 9600));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    // again :P
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(SerialCommunicationShould, returnFalseWhenFailToOpenDevicde) {
    sut_.reset(new SerialCommunication("garbage device", 9600));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(SerialCommunicationShould, returnFalseWhenUsedWithUnsupportedBaudrate) {
    if (!fileExists(device0Filename_)) {
        return;
    }
    sut_.reset(new SerialCommunication(device0Filename_, 123456789));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(SerialCommunicationShould, returnFalseWhenUsedWithUnsupportedCharsize) {
    if (!fileExists(device0Filename_)) {
        return;
    }
    sut_.reset(new SerialCommunication(device0Filename_, 9600, false, false, 69));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(SerialCommunicationShould, returnNegativeWhenAttemptToReadWriteFromUninitializedDevice) {
    if (!fileExists(device0Filename_)) {
        return;
    }
    sut_.reset(new SerialCommunication(device0Filename_, 9600));
    std::string buff;
    ASSERT_EQ(-1, sut_->read(&buff, 1));
    ASSERT_EQ(-1, sut_->write("xxx"));
}

TEST_F(SerialCommunicationShould, correctlySendAndReceiveDataOnBothCommPoints) {
    if (!fileExists(device0Filename_) || !fileExists(device1Filename_)) {
        return;
    }
    SerialCommunication com0(device0Filename_, 115200);
    SerialCommunication com1(device1Filename_, 115200);
    using namespace std::literals::string_literals;  // NOLINT
    std::string expectedMessage
        = "xxx xxx yyy SomeComplicated message \0 ..."s;
    std::string receivedMessage;
    /*
     * Double initialization to test resources availability after shutting down
     */
    ASSERT_TRUE(com0.initialize());
    ASSERT_TRUE(com0.deinitialize());
    ASSERT_TRUE(com0.initialize());
    ASSERT_TRUE(com1.initialize());
    ASSERT_TRUE(com1.deinitialize());
    ASSERT_TRUE(com1.initialize());
    ASSERT_EQ(expectedMessage.size(), com0.write(expectedMessage));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_EQ(expectedMessage.size(), com1.read(&receivedMessage,
        expectedMessage.size()));
    ASSERT_EQ(expectedMessage, receivedMessage);
}

TEST_F(SerialCommunicationShould, blockUntilRequestedNumOfBytesReceived) {
    if (!fileExists(device0Filename_) || !fileExists(device1Filename_)) {
        return;
    }
    SerialCommunication com0(device0Filename_, 115200, true);
    SerialCommunication com1(device1Filename_, 115200);
    ASSERT_TRUE(com0.initialize());
    ASSERT_TRUE(com1.initialize());
    int numBytesToReceive = 69;
    std::string toWrite(numBytesToReceive, 'x');
    std::future<std::string> receivedMsg = std::async(std::launch::async,
        [&com0, numBytesToReceive] () {
            std::string msg;
            com0.read(&msg, numBytesToReceive);
            return msg;
        });
    com1.write(toWrite.substr(0, 10));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_EQ(std::future_status::timeout, receivedMsg.wait_for(std::chrono::seconds(0)));
    com1.write(toWrite.substr(10));
    ASSERT_EQ(std::future_status::ready, receivedMsg.wait_for(std::chrono::seconds(1)));
    ASSERT_EQ(toWrite, receivedMsg.get());
}

TEST_F(SerialCommunicationShould, returnZeroIfTimeoutOnBlockingRead) {
    if (!fileExists(device0Filename_)) {
        return;
    }
    SerialCommunication com0(device0Filename_, 115200, true, false, 8,
        std::chrono::milliseconds(500));
    ASSERT_TRUE(com0.initialize());

    std::string msg;
    int numBytesToReceive = 1;
    int received = com0.read(&msg, numBytesToReceive);
    ASSERT_EQ(received, 0);
}
