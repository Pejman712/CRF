/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>

#include "Mocks/Communication/SerialCommunicationMock.hpp"

#include "Inclinometer/Zerotronic.hpp"

using crf::communication::serialcommunication::SerialCommunicationMock;
using crf::communication::serialcommunication::SerialCommunication;
using crf::sensors::inclinometer::IInclinometer;
using crf::sensors::inclinometer::Zerotronic;

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

class ZerotronicShould: public ::testing::Test {
 protected:
    ZerotronicShould() {
        requestedOper_ = 0;
        simulatedReply_ = "";
        sCommMock_.reset(new SerialCommunicationMock);
        ON_CALL(*sCommMock_, initialize()).WillByDefault(Return(true));
        ON_CALL(*sCommMock_, deinitialize()).WillByDefault(Return(true));
        ON_CALL(*sCommMock_, write(_)).WillByDefault(Invoke(
            [this](const std::string& buffer) -> int {
                if (buffer.size() == 1) {
                    requestedOper_ = buffer[0];
                }
                return 1;
            }));
        ON_CALL(*sCommMock_, read(_, _)).WillByDefault(Invoke(
            [this](std::string* buffer, int length) -> int {
                if (requestedOper_ == 'P' && length >= simulatedReply_.size()) {
                    *buffer = simulatedReply_;
                    requestedOper_ = 0;
                    return simulatedReply_.size();
                }
                return 0;
            }));
        sut_.reset(new Zerotronic(sCommMock_));
    }
    char requestedOper_;
    std::string simulatedReply_;
    std::shared_ptr<SerialCommunicationMock> sCommMock_;
    std::unique_ptr<IInclinometer> sut_;
};

TEST_F(ZerotronicShould, returnFalseIfInitializedOrDeinitializedTwice) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    // pfff ... fuck it, let's do again :P
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(ZerotronicShould, returnTwoElementsVectorWithExpectedValues) {
    simulatedReply_.clear();
    double firstExpectedValue = 69.69;
    double secondExpectedValue = -111.111;
    simulatedReply_ = "xxx yyy " + std::to_string(firstExpectedValue)
        + " blebleble " + std::to_string(secondExpectedValue) + " zzz";
    auto result = sut_->getInclination();
    ASSERT_EQ(2, result.size());
    ASSERT_EQ(firstExpectedValue, result[0]);
    ASSERT_EQ(secondExpectedValue, result[1]);
}

TEST_F(ZerotronicShould, returnEmptyVectorWhenReceivedWrongMessage) {
    simulatedReply_.clear();
    simulatedReply_ = "SomeKindOfWrongData xxx";
    auto result = sut_->getInclination();
    ASSERT_EQ(0, result.size());
}

TEST_F(ZerotronicShould, returnEmptyVectorWhenReceivedNotFloatingPointsValues) {
    simulatedReply_.clear();
    simulatedReply_ = "xxx yyy garbage1 zzz not6969ANum32.32Ber xxx";
    auto result = sut_->getInclination();
    ASSERT_EQ(0, result.size());
}

TEST_F(ZerotronicShould, DISABLED_liveExample) {
    /*
     * Example on how Zetronic can be used in a production
     * Replace /dev/ttyUSB0 with your serial comm name
     * I don't know if it will work, note the usege of 7-bits long character
     * in serial communication
     */
    auto serial = std::make_shared<SerialCommunication>("/dev/ttyUSB0", 9600, false, 7);
    sut_.reset(new Zerotronic(serial));
    sut_->initialize();
    crf::utility::logger::EventLogger logger("ZerotronicShould");
    for (int i = 0; i < 10; i++) {
        auto result = sut_->getInclination();
        if (result.size() < 2) {
            logger->warn("result.size() == {}", result.size());
            continue;
        }
        logger->info("result[0]: {}, result[1]: {}", result[0], result[1]);
    }
}
