/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <condition_variable>
#include <future>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <iostream>
#include <thread>
#include <string>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "CANSocket/ICANSocket.hpp"
#include "CANSocket/CANSocketMock.hpp"
#include "CANOpenDevices/ObjectDictionary.hpp"
#include "CANOpenDevices/ObjectDictionaryRegister.hpp"
#include "CANOpenDevices/CANOpenContext.hpp"
#include "CANOpenDevices/CANOpenMotors/CANOpenMotorMock.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::communication::cansocket::ICANSocket;
using crf::communication::cansocket::CANSocketMock;
using crf::devices::canopendevices::CANOpenContext;
using crf::devices::canopendevices::ICANOpenMotor;
using crf::devices::canopendevices::CANOpenMotorMock;
using crf::devices::canopendevices::ObjectDictionary;
using crf::devices::canopendevices::ObjectDictionary;
using crf::devices::canopendevices::ObjectDictionaryRegister;

MATCHER_P(canFramesAreEqual, other, "Equality matcher for can_frame") {
    if (arg->can_id != other.can_id) return false;
    if (arg->can_dlc != other.can_dlc) return false;
    for (int i=0; i < 8; i++) {
        if (arg->data[i] != other.data[i]) return false;
    }
    return true;
}

class CANOpenContextShould : public ::testing::Test {
 protected:
    CANOpenContextShould() :
        logger_("CANOpenContextShould"),
        testDirName_(),
        socket_(new NiceMock<CANSocketMock>),
        motor1_(new NiceMock<CANOpenMotorMock>),
        motor2_(new NiceMock<CANOpenMotorMock>) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("modules/"));
        testDirName_ += "modules/Devices/CANOpenDevicesDeprecated/tests/configurations/";
    }

    ~CANOpenContextShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        motor1Dictionary_.reset(new ObjectDictionary(testDirName_+"CiA402.json"));
        motor2Dictionary_.reset(new ObjectDictionary(testDirName_+"CiA402.json"));

        ON_CALL(*motor1_, getCANID()).WillByDefault(Return(1));
        ON_CALL(*motor1_, getObjectDictionary()).WillByDefault(Return(motor1Dictionary_));
        ON_CALL(*motor2_, getCANID()).WillByDefault(Return(2));
        ON_CALL(*motor2_, getObjectDictionary()).WillByDefault(Return(motor2Dictionary_));

        ON_CALL(*socket_, initialize()).WillByDefault(Return(true));
        ON_CALL(*socket_, deinitialize()).WillByDefault(Return(true));
        ON_CALL(*socket_, getName()).WillByDefault(Return("can0"));

        sdoFrameMotor1_.can_id = 0x581;
        sdoFrameMotor1_.can_dlc = 0x08;
        sdoFrameMotor1_.data[0] = 0x4f;
        sdoFrameMotor1_.data[1] = 0x00;
        sdoFrameMotor1_.data[2] = 0x20;
        sdoFrameMotor1_.data[3] = 0x00;
        sdoFrameMotor1_.data[4] = 0x01;
        sdoFrameMotor1_.data[5] = 0x00;
        sdoFrameMotor1_.data[6] = 0x00;
        sdoFrameMotor1_.data[7] = 0x00;

        sdoFrameMotor2_.can_id = 0x582;
        sdoFrameMotor2_.can_dlc = 0x08;
        sdoFrameMotor2_.data[0] = 0x4f;
        sdoFrameMotor2_.data[1] = 0x00;
        sdoFrameMotor2_.data[2] = 0x20;
        sdoFrameMotor2_.data[3] = 0x00;
        sdoFrameMotor2_.data[4] = 0x02;
        sdoFrameMotor2_.data[5] = 0x00;
        sdoFrameMotor2_.data[6] = 0x00;
        sdoFrameMotor2_.data[7] = 0x00;

        sdoFrameUnknownMotor_.can_id = 0x583;
        sdoFrameUnknownMotor_.can_dlc = 0x08;
        sdoFrameUnknownMotor_.data[0] = 0x4f;
        sdoFrameUnknownMotor_.data[1] = 0x00;
        sdoFrameUnknownMotor_.data[2] = 0x20;
        sdoFrameUnknownMotor_.data[3] = 0x00;
        sdoFrameUnknownMotor_.data[4] = 0x02;
        sdoFrameUnknownMotor_.data[5] = 0x00;
        sdoFrameUnknownMotor_.data[6] = 0x00;
        sdoFrameUnknownMotor_.data[7] = 0x00;

        syncFrame_ = {};
        syncFrame_.can_id = 0x80;
        syncFrame_.can_dlc = 0x00;

        guardFrame_ = {};
        guardFrame_.can_id = 0x700 + 0x25;
        guardFrame_.can_dlc = 0x01;
        guardFrame_.data[0] = 0x05;
    }

    can_frame sdoFrameMotor1_;
    can_frame sdoFrameMotor2_;
    can_frame sdoFrameUnknownMotor_;
    can_frame syncFrame_;
    can_frame guardFrame_;

    crf::utility::logger::EventLogger logger_;
    std::string testDirName_;
    std::shared_ptr<CANSocketMock> socket_;
    std::shared_ptr<CANOpenMotorMock> motor1_;
    std::shared_ptr<CANOpenMotorMock> motor2_;

    std::shared_ptr<ObjectDictionary> motor1Dictionary_;
    std::shared_ptr<ObjectDictionary> motor2Dictionary_;

    std::unique_ptr<CANOpenContext> context_;
};

TEST_F(CANOpenContextShould, initializeDeinitializeSequence) {
    context_.reset(new CANOpenContext(socket_));
    ASSERT_TRUE(context_->initialize());
    ASSERT_FALSE(context_->initialize());
    ASSERT_TRUE(context_->deinitialize());
    ASSERT_FALSE(context_->deinitialize());

    ASSERT_TRUE(context_->initialize());
    ASSERT_FALSE(context_->initialize());
    ASSERT_TRUE(context_->deinitialize());
    ASSERT_FALSE(context_->deinitialize());
}

TEST_F(CANOpenContextShould, failsToAddTwiceTheSameMotor) {
    context_.reset(new CANOpenContext(socket_));

    ASSERT_TRUE(context_->addDevice(motor1_));
    ASSERT_FALSE(context_->addDevice(motor1_));
    ASSERT_TRUE(context_->initialize());
    ASSERT_TRUE(context_->addDevice(motor2_));
    ASSERT_FALSE(context_->addDevice(motor2_));
    ASSERT_TRUE(context_->deinitialize());
}

TEST_F(CANOpenContextShould, cantSendSyncIfNotInitialized) {
    context_.reset(new CANOpenContext(socket_));

    EXPECT_CALL(*socket_, write(canFramesAreEqual(syncFrame_)))
        .Times(AtLeast(1)).WillRepeatedly(Return(0));
    ASSERT_TRUE(context_->addDevice(motor1_));
    ASSERT_FALSE(context_->sendSync());
    ASSERT_TRUE(context_->initialize());
    ASSERT_TRUE(context_->sendSync());
    ASSERT_FALSE(context_->setSyncFrequency(std::chrono::milliseconds(20)));
    ASSERT_TRUE(context_->deinitialize());
}

TEST_F(CANOpenContextShould, cantSendGuardIfNotInitialized) {
    context_.reset(new CANOpenContext(socket_));
    EXPECT_CALL(*socket_, write(canFramesAreEqual(guardFrame_)))
        .Times(AtLeast(1)).WillRepeatedly(Return(0));
    ASSERT_TRUE(context_->addDevice(motor1_));
    ASSERT_FALSE(context_->sendGuard());
    ASSERT_TRUE(context_->initialize());
    ASSERT_TRUE(context_->sendGuard());
    ASSERT_FALSE(context_->setGuardFrequency(std::chrono::milliseconds(20)));
    ASSERT_TRUE(context_->deinitialize());
}

TEST_F(CANOpenContextShould, correctlySendGuardThread) {
    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;
    std::unique_lock<std::mutex> lock(mtx);

    EXPECT_CALL(*socket_, write(canFramesAreEqual(guardFrame_)))
        .Times(AtLeast(1)).WillOnce(Invoke([this, &mtx, &cv, &done](can_frame* frame) {
            std::unique_lock<std::mutex> lock(mtx);
            cv.notify_all();
            done = true;
            return 0;
        })).WillRepeatedly(Return(0));
    context_.reset(new CANOpenContext(socket_));

    ASSERT_TRUE(context_->addDevice(motor1_));
    ASSERT_TRUE(context_->setGuardFrequency(std::chrono::milliseconds(1)));
    ASSERT_TRUE(context_->initialize());
    ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(2), [&done]() { return done; }));
    ASSERT_TRUE(context_->deinitialize());
}

TEST_F(CANOpenContextShould, correctlySendSyncThread) {
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> done(false);
    std::unique_lock<std::mutex> lock(mtx);
    EXPECT_CALL(*socket_, write(canFramesAreEqual(syncFrame_)))
        .Times(AtLeast(1)).WillOnce(Invoke([this, &mtx, &cv, &done](can_frame* frame) {
            std::unique_lock<std::mutex> lock(mtx);
            cv.notify_all();
            done = true;
            return 0;
        })).WillRepeatedly(Return(0));

    context_.reset(new CANOpenContext(socket_));

    ASSERT_TRUE(context_->addDevice(motor1_));
    ASSERT_TRUE(context_->setSyncFrequency(std::chrono::milliseconds(1)));
    ASSERT_TRUE(context_->initialize());
    ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(2), [&done]() { return done.load(); }));
    ASSERT_TRUE(context_->deinitialize());
}

TEST_F(CANOpenContextShould, correctlyProcessPackets) {
    EXPECT_CALL(*socket_, read(_)).WillOnce(Invoke([this](can_frame* frame) {
        std::memcpy(frame, &sdoFrameMotor1_, sizeof(can_frame));
        return sizeof(sdoFrameMotor1_);
    })).WillOnce(Invoke([this](can_frame* frame) {
        std::memcpy(frame, &sdoFrameMotor2_, sizeof(can_frame));
        return sizeof(sdoFrameMotor2_);
    })).WillRepeatedly(Invoke([this](can_frame* frame) {
        std::memcpy(frame, &sdoFrameUnknownMotor_, sizeof(can_frame));
        return sizeof(sdoFrameUnknownMotor_);
    }));

    context_.reset(new CANOpenContext(socket_));

    ASSERT_TRUE(context_->addDevice(motor1_));
    ASSERT_TRUE(context_->addDevice(motor2_));
    ASSERT_TRUE(context_->initialize());

    ASSERT_TRUE(motor1Dictionary_->waitForSdoResponse(
        motor1Dictionary_->getRegister("nodeID").get(), std::chrono::seconds(1)));
    ASSERT_TRUE(motor2Dictionary_->waitForSdoResponse(
        motor2Dictionary_->getRegister("nodeID").get(), std::chrono::seconds(1)));

    ASSERT_FALSE(motor1Dictionary_->waitForSdoResponse(
        motor1Dictionary_->getRegister("nodeID").get(), std::chrono::milliseconds(100)));
    ASSERT_FALSE(motor2Dictionary_->waitForSdoResponse(
         motor2Dictionary_->getRegister("nodeID").get(), std::chrono::milliseconds(100)));

    ASSERT_EQ(motor1Dictionary_->getRegister(0x2000).get().getValue<uint8_t>(), 0x01);
    ASSERT_EQ(motor2Dictionary_->getRegister(0x2000).get().getValue<uint8_t>(), 0x02);

    ASSERT_TRUE(context_->deinitialize());
}
