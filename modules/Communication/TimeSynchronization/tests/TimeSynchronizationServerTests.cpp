/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Mocks/Communication/IpcMock.hpp"
#include "TimeSynchronization/TimeSynchronizationServer.hpp"

using testing::_;
using testing::Return;
using testing::NiceMock;
using testing::Invoke;

using crf::communication::timesynchronizationserver::TimeSynchronizationServer;

class TimeSynchronizationServerShould : public ::testing::Test {
 protected:
    TimeSynchronizationServerShould() :
        logger_("TimeSynchronizationServerShould") {
            logger_->info("{} BEGIN",
                        testing::UnitTest::GetInstance()->current_test_info()->name());
            ipcMock_.reset(new NiceMock<IpcMock>);

            ON_CALL(*ipcMock_, read(_, _)).WillByDefault(
                Invoke([this](std::string& bytes, Packets::PacketHeader& header) {
                    communication::datapackets::JSONPacket json;
                    json.data_["time"] = 1234;
                    bytes = json.serialize();
                    header = json.getHeader();
                    return true;
                }));
    }

    ~TimeSynchronizationServerShould() {
        logger_->info("{} END with {}",
                      testing::UnitTest::GetInstance()->current_test_info()->name(),
                      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<IpcMock> ipcMock_;
    std::unique_ptr<TimeSynchronizationServer> sut_;
};

TEST_F(TimeSynchronizationServerShould, initializeDeinitializeSequence) {
    sut_.reset(
        new TimeSynchronizationServer(ipcMock_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(TimeSynchronizationServerShould, correctlyProcessInputMessage) {
    sut_.reset(
        new TimeSynchronizationServer(ipcMock_));
    std::condition_variable cv;
    std::mutex m;
    bool ipcMsgFlag(false);
    std::chrono::milliseconds ms(0);
    EXPECT_CALL(*ipcMock_, write(_, _)).WillOnce(
        Invoke([this, &ms, &m, &cv, &ipcMsgFlag](const std::string& bytes, const Packets::PacketHeader& header) {  // NOLINT
            communication::datapackets::JSONPacket sentPacket;
            sentPacket.deserialize(bytes);
            ms = std::chrono::milliseconds(sentPacket.data_["time"].get<int64_t>());
            std::lock_guard<std::mutex> l(m);
            ipcMsgFlag = true;
            cv.notify_all();
            return true;
        })).WillRepeatedly(Return(true));
    std::unique_lock<std::mutex> l(m);
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(cv.wait_for(
        l, std::chrono::seconds(1), [&ipcMsgFlag](){ return ipcMsgFlag; }));
    ASSERT_GT(ms.count(), 0);
    ASSERT_TRUE(sut_->deinitialize());
}
