/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <linux/can.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <queue>
#include <cstring>
#include <vector>
#include <boost/optional.hpp>
#include <thread>

#include "EventLogger/EventLogger.hpp"
#include "SchunkArm/ISchunkDevice.hpp"
#include "SchunkArm/SchunkDevice.hpp"
#include "SchunkArm/SchunkCommands.hpp"
#include "CANSocket/CANSocketMock.hpp"

using crf::utility::logger::EventLogger;
using crf::actuators::schunkarm::ISchunkDevice;
using crf::actuators::schunkarm::SchunkDevice;
using crf::communication::cansocket::CANSocketMock;

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;
using testing::AllOf;

MATCHER_P2(FrameDataEquals, index, value, "") {
    return (arg->data[index] == value);
}

MATCHER_P2(FrameIntValueEquals, firstByte, value, "") {
    int readValue;
    std::memcpy(&readValue, &arg->data[firstByte], sizeof(int));
    return (readValue == value);
}

class SchunkDeviceShould: public ::testing::Test {
 protected:
    SchunkDeviceShould():
    tempCanFrame_{},
    logger_("SchunkDeviceShould") {
        logger_->info("{} BEGIN",
                      testing::UnitTest::GetInstance()->current_test_info()->name());
        canSocketMock_.reset(new NiceMock<CANSocketMock>);
        ON_CALL(*canSocketMock_, write(_)).WillByDefault(Invoke(
                [this](can_frame* frame) {
                    return putAcknowledmentIntoQueue(frame);}));
        ON_CALL(*canSocketMock_, read(_)).WillByDefault(Invoke(
                [this](can_frame* frame) {
                    return feedTheCanbus(frame);}));
    }
    ~SchunkDeviceShould() {
        logger_->info("{} END with {}",
                      testing::UnitTest::GetInstance()->current_test_info()->name(),
                      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    void putStateMessageToQueue(int canID) {
        tempCanFrame_.can_id = MESSAGE_FROM_DEVICE+canID;
        tempCanFrame_.can_dlc = 0x07;
        tempCanFrame_.data[0] = tempCanFrame_.can_dlc - 1;
        tempCanFrame_.data[1] = FRAG_START;
        tempCanFrame_.data[2] = GET_STATE;
        int position = startPosition_ * radToMilliDeg_;
        int speed = startVelocity_ * radToMilliDeg_;
        std::memcpy(&tempCanFrame_.data[3], &position, sizeof(int));
        std::memcpy(&tempCanFrame_.data[7], &speed, sizeof(char));
        initializer.push(tempCanFrame_);
        std::memcpy(&tempCanFrame_.data[1], &speed, sizeof(int));
        tempCanFrame_.data[1] = FRAG_END;
        tempCanFrame_.data[5] = robotDefaultState_;
        tempCanFrame_.data[6] = robotDefaultError_;
        initializer.push(tempCanFrame_);
    }
    int putAcknowledmentIntoQueue(can_frame* frame) {
        can_frame ackCanFrame{};
        int motorID = (frame->can_id & GET_DEVICE_CODE);
        ackCanFrame.can_id = MESSAGE_FROM_DEVICE + motorID;
        ackCanFrame.can_dlc = 0x07;
        ackCanFrame.data[0] = tempCanFrame_.can_dlc - 1;
        ackCanFrame.data[1] = frame->data[1];

        if (frame->data[1] == GET_STATE) {
            int period;
            std::memcpy( &period, &frame->data[2], sizeof(int));
            if (frame->data[6] == GET_ONLY_STATE) {
                ackCanFrame.data[2] = REFERENCED_BREAKON;
            } else if (period == 0 &&
                       frame->data[6] == GET_POS_VEL) {
                ackCanFrame.data[1] = FRAG_START;
                ackCanFrame.data[2] = GET_STATE;
                int position = startPosition_ * radToMilliDeg_;
                int speed = startVelocity_ * radToMilliDeg_;
                std::memcpy(&ackCanFrame.data[3], &position, sizeof(int));
                std::memcpy(&ackCanFrame.data[7], &speed, sizeof(char));
                initializer.push(ackCanFrame);
                std::memcpy(&ackCanFrame.data[1], &speed, sizeof(int));
                ackCanFrame.data[1] = FRAG_END;
                ackCanFrame.data[5] = robotDefaultState_;
                ackCanFrame.data[6] = robotDefaultError_;
            } else if (period != 0 &&
                       frame->data[6] == GET_POS_VEL) {
                sendStatePeriodic_.push_back(motorID);
                return 16;
            }
        } else if (frame->data[1] == GET_CONFIG) {
            if (frame->data[2] == MAX_POS) {
                ackCanFrame.data[1] = GET_CONFIG;
                ackCanFrame.data[2] = MAX_POS;
                std::memcpy(&ackCanFrame.data[3], &maxPos, sizeof(int));
            } else if (frame->data[2] == MIN_POS) {
                ackCanFrame.data[1] = GET_CONFIG;
                ackCanFrame.data[2] = MIN_POS;
                std::memcpy(&ackCanFrame.data[3], &minPos, sizeof(int));
            } else if (frame->data[2] == MAX_SPEED) {
                ackCanFrame.data[1] = GET_CONFIG;
                ackCanFrame.data[2] = MAX_SPEED;
                std::memcpy(&ackCanFrame.data[3], &maxSpeed, sizeof(int));
            } else if (frame->data[2] == MAX_ACCELERATION) {
                ackCanFrame.data[1] = GET_CONFIG;
                ackCanFrame.data[2] = MAX_ACCELERATION;
                std::memcpy(&ackCanFrame.data[3], &maxAcc, sizeof(int));
            } else if (frame->data[2] == MAX_CURRENT) {
                ackCanFrame.data[1] = GET_CONFIG;
                ackCanFrame.data[2] = MAX_CURRENT;
                std::memcpy(&ackCanFrame.data[3], &maxCur, sizeof(int));
            }
        } else {
            ackCanFrame.data[2] = ASCII_O;
            ackCanFrame.data[3] = ASCII_K;
        }

        initializer.push(ackCanFrame);
        return 16;
    }
    int feedTheCanbus(can_frame* frame) {
        if (initializer.empty()) {
            for (unsigned int i = 0; i < sendStatePeriodic_.size(); i++) {
                putStateMessageToQueue(sendStatePeriodic_.at(i));
            }
            std::chrono::milliseconds duration(50);
            std::this_thread::sleep_for(duration);
        }

        if (!initializer.empty()) {
            std::memcpy(frame, &initializer.front(), sizeof(can_frame));
            initializer.pop();
            return 16;
        } else {
            std::chrono::milliseconds duration(5);
            std::this_thread::sleep_for(duration);
            return -1;
        }
    }

    // Vector for initializing the SchunkDevice
    std::queue<can_frame> initializer;
    std::vector<int> sendStatePeriodic_;
    can_frame tempCanFrame_;
    const int maxPos = 170000;
    const int minPos = -170000;
    const int maxSpeed = 120000;
    const int maxAcc = 500000;
    const int maxCur = 3900;
    const float startPosition_ = 1.7;
    const float startVelocity_ = 1.2;
    u_int8_t robotDefaultState_ = REFERENCED_BREAKON;
    u_int8_t robotDefaultError_ = 0x00;

    EventLogger logger_;
    std::shared_ptr<CANSocketMock> canSocketMock_;
    std::unique_ptr<SchunkDevice> sut_;
    const int canID_ = 3;
    const float radToMilliDeg_ = 180.0 / M_PI *1000;
    const float millidegToRad_ = M_PI / 180 /1000;
    const float toleranceForFloatEquality = 0.001;
};

TEST_F(SchunkDeviceShould, cleanCANBusHappyPath) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, CMD_ACK))).
        Times(6).WillRepeatedly(Return(16));
    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, GET_STATE))).
        Times(6).WillRepeatedly(Return(16));
    ASSERT_TRUE(SchunkDevice::cleanCanBus(canSocketMock_));
}

TEST_F(SchunkDeviceShould, cleanCANBusReturnFalseIfCANNNotAvailable) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    EXPECT_CALL(*canSocketMock_, write(_)).WillOnce(Return(-1)).WillRepeatedly(DoDefault());
    ASSERT_FALSE(SchunkDevice::cleanCanBus(canSocketMock_));
}

TEST_F(SchunkDeviceShould, returnFalseIfInitializedOrDeinitializedTwice) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, initializeReturnFalseIfCanNotAvailable) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));

    EXPECT_CALL(*canSocketMock_, write(_)).WillOnce(Return(-1)).WillRepeatedly(DoDefault());
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*canSocketMock_, read(_)).WillOnce(Return(-1)).WillRepeatedly(DoDefault());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
}

TEST_F(SchunkDeviceShould, initializeReturnFalseIfBadDeviceState) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    robotDefaultState_ = STATE_NOT_REFERENCED;
    ASSERT_FALSE(sut_->initialize());
    robotDefaultState_ = REFERENCED_ERROR;
    robotDefaultError_ = ERROR_CONFIG_MEMORY;
    ASSERT_FALSE(sut_->initialize());
    robotDefaultError_ = ERROR_SOFT_LOW;
    ASSERT_TRUE(sut_->initialize());
}

TEST_F(SchunkDeviceShould, handleErrorTest) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, CMD_ACK))).WillOnce(Return(16));
    ASSERT_TRUE(sut_->handleError(SchunkDevice::translateErrorToString(ERROR_SOFT_LOW)));
    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, CMD_STOP))).WillOnce(Return(16));
    ASSERT_FALSE(sut_->handleError(SchunkDevice::translateErrorToString(ERROR_COMMUTATION)));
}

TEST_F(SchunkDeviceShould, applyBreakTest) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    EXPECT_CALL(*canSocketMock_, write(_)).WillOnce(Return(-1));
    ASSERT_FALSE(sut_->applyBreak());
    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, CMD_STOP))).WillOnce(Return(16));
    ASSERT_TRUE(sut_->applyBreak());
}

TEST_F(SchunkDeviceShould, fastStopTest) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    EXPECT_CALL(*canSocketMock_, write(_)).WillOnce(Return(-1));
    ASSERT_FALSE(sut_->fastStop());
    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, CMD_FAST_STOP))).WillOnce(Return(16));
    ASSERT_TRUE(sut_->fastStop());
}

TEST_F(SchunkDeviceShould, testGettersandDefaultValueInitialization) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));

    ASSERT_EQ(sut_->getMotorPosition(), boost::none);
    ASSERT_EQ(sut_->getMotorVelocity(), boost::none);
    ASSERT_EQ(sut_->getMaxPosition(), boost::none);
    ASSERT_EQ(sut_->getMinPosition(), boost::none);
    ASSERT_EQ(sut_->getMaxVelocity(), boost::none);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_NEAR(sut_->getMotorPosition().get(), startPosition_, toleranceForFloatEquality);
    ASSERT_NEAR(sut_->getMotorVelocity().get(), startVelocity_, toleranceForFloatEquality);
    ASSERT_EQ(sut_->getMaxPosition().get(), maxPos * millidegToRad_);
    ASSERT_EQ(sut_->getMinPosition().get(), minPos * millidegToRad_);
    ASSERT_EQ(sut_->getMaxVelocity().get(), maxSpeed * millidegToRad_);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, getStateReturnFalseIfCanNotAvailable) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));

    EXPECT_CALL(*canSocketMock_, write(_)).WillOnce(Return(-1)).WillRepeatedly(DoDefault());
    ASSERT_FALSE(sut_->getState());
    EXPECT_CALL(*canSocketMock_, read(_)).WillOnce(Return(-1)).WillRepeatedly(DoDefault());
    ASSERT_FALSE(sut_->getState());
}

TEST_F(SchunkDeviceShould, getStateHappyPath) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    ASSERT_TRUE(sut_->initialize());

    tempCanFrame_.can_id = MESSAGE_FROM_DEVICE+canID_;
    tempCanFrame_.can_dlc = 0x07;
    tempCanFrame_.data[0] = tempCanFrame_.can_dlc - 1;
    tempCanFrame_.data[1] = FRAG_START;
    tempCanFrame_.data[2] = GET_STATE;
    int value = 2.3 * radToMilliDeg_;
    int speed = -0.8 * radToMilliDeg_;
    std::memcpy(&tempCanFrame_.data[3], &value, sizeof(int));
    std::memcpy(&tempCanFrame_.data[7], &speed, sizeof(char));
    initializer.push(tempCanFrame_);
    std::memcpy(&tempCanFrame_.data[1], &speed, sizeof(int));
    tempCanFrame_.data[1] = FRAG_END;
    tempCanFrame_.data[5] = REFERENCED_BREAKON;
    initializer.push(tempCanFrame_);

    ASSERT_TRUE(sut_->getState());

    ASSERT_NEAR(sut_->getMotorPosition().get(), 2.3, toleranceForFloatEquality);
    ASSERT_NEAR(sut_->getMotorVelocity().get(), -0.8, toleranceForFloatEquality);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, setTargetValueFunctionReturnFalseIfNotInitialized) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));

    ASSERT_FALSE(sut_->setTargetVelocityToPercentage(0));
    ASSERT_FALSE(sut_->setTargetAccelerationToPercentage(0));
    ASSERT_FALSE(sut_->setTargetCurrentToPercentage(0));
}

TEST_F(SchunkDeviceShould, setTargetValueFunctionReturnFalseIfOutOfBounds) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setTargetVelocityToPercentage(-3));
    ASSERT_FALSE(sut_->setTargetAccelerationToPercentage(-4));
    ASSERT_FALSE(sut_->setTargetCurrentToPercentage(-5));
    ASSERT_FALSE(sut_->setTargetVelocityToPercentage(106));
    ASSERT_FALSE(sut_->setTargetAccelerationToPercentage(107));
    ASSERT_FALSE(sut_->setTargetCurrentToPercentage(108));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, setTargetValueFunctionReturnFalseIfWriteFails) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    ASSERT_TRUE(sut_->initialize());
    // Set if write fails
    EXPECT_CALL(*canSocketMock_, write(_)).Times(3).
    WillRepeatedly(Return(-1));
    ASSERT_FALSE(sut_->setTargetVelocityToPercentage(0));
    ASSERT_FALSE(sut_->setTargetAccelerationToPercentage(0));
    ASSERT_FALSE(sut_->setTargetCurrentToPercentage(0));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, setTargetValueFunctionHappyPath) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    ASSERT_TRUE(sut_->initialize());

    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, SET_TARGET_VEL))).WillOnce(Return(16));
    ASSERT_TRUE(sut_->setTargetVelocityToPercentage(50));

    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, SET_TARGET_ACC))).WillOnce(Return(16));
    ASSERT_TRUE(sut_->setTargetAccelerationToPercentage(50));

    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, SET_TARGET_CUR))).WillOnce(Return(16));
    ASSERT_TRUE(sut_->setTargetCurrentToPercentage(50));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, getStatePeriodicTest) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    ASSERT_FALSE(sut_->getStatePeriodic(10));
    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*canSocketMock_, write(_)).WillOnce(Return(-1));
    ASSERT_FALSE(sut_->getStatePeriodic(10));
    EXPECT_CALL(*canSocketMock_, write(AllOf(FrameDataEquals(1, GET_STATE),
            FrameIntValueEquals(2, 10)))).WillOnce(Return(16));
    ASSERT_TRUE(sut_->getStatePeriodic(10));
}

TEST_F(SchunkDeviceShould, parseStatePeriodicReturnFalseIfNotInitialized) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));

    ASSERT_FALSE(sut_->parseStatePeriodic(tempCanFrame_));
    ASSERT_TRUE(sut_->initialize());

    tempCanFrame_.can_id = MESSAGE_FROM_DEVICE+canID_;
    tempCanFrame_.can_dlc = 0x07;
    tempCanFrame_.data[0] = tempCanFrame_.can_dlc - 1;
    tempCanFrame_.data[1] = FRAG_START;
    ASSERT_TRUE(sut_->parseStatePeriodic(tempCanFrame_));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, parseStatePeriodicReturnFalseIfNotRightMessage) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));

    ASSERT_FALSE(sut_->parseStatePeriodic(tempCanFrame_));
    ASSERT_TRUE(sut_->initialize());

    // wrong CAN ID
    tempCanFrame_.can_id = MESSAGE_FROM_DEVICE+canID_+1;
    tempCanFrame_.can_dlc = 0x07;
    tempCanFrame_.data[0] = tempCanFrame_.can_dlc - 1;
    tempCanFrame_.data[1] = FRAG_START;
    ASSERT_FALSE(sut_->parseStatePeriodic(tempCanFrame_));

    // wrong message code
    tempCanFrame_.can_id = MESSAGE_FROM_DEVICE+canID_;
    tempCanFrame_.can_dlc = 0x07;
    tempCanFrame_.data[0] = tempCanFrame_.can_dlc - 1;
    tempCanFrame_.data[1] = FRAG_START - 2;
    ASSERT_FALSE(sut_->parseStatePeriodic(tempCanFrame_));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, parseStatePeriodicReturnTrue) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    ASSERT_TRUE(sut_->initialize());

    tempCanFrame_.can_id = MESSAGE_FROM_DEVICE+canID_;
    tempCanFrame_.can_dlc = 0x07;
    tempCanFrame_.data[0] = tempCanFrame_.can_dlc;
    tempCanFrame_.data[1] = FRAG_START;
    tempCanFrame_.data[2] = GET_STATE;
    int position = 2.3 * radToMilliDeg_;
    int speed = -0.8 * radToMilliDeg_;
    std::memcpy(&tempCanFrame_.data[3], &position, sizeof(int));
    std::memcpy(&tempCanFrame_.data[7], &speed, sizeof(char));

    // update position
    ASSERT_TRUE(sut_->parseStatePeriodic(tempCanFrame_));
    ASSERT_NEAR(sut_->getMotorPosition().get(), 2.3, toleranceForFloatEquality);

    // update velocity
    std::memcpy(&tempCanFrame_.data[1], &speed, sizeof(int));
    tempCanFrame_.data[1] = FRAG_END;
    tempCanFrame_.data[5] = robotDefaultState_;
    tempCanFrame_.data[6] = robotDefaultError_;
    ASSERT_TRUE(sut_->parseStatePeriodic(tempCanFrame_));
    ASSERT_NEAR(sut_->getMotorVelocity().get(), -0.8, toleranceForFloatEquality);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, setPositionReturnFalseIfNotInitialized) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    ASSERT_FALSE(sut_->setPosition(1.0));
    ASSERT_TRUE(sut_->initialize());
    int goalPosInMillideg = 1.0 * radToMilliDeg_;
    EXPECT_CALL(*canSocketMock_, write(FrameIntValueEquals(2, goalPosInMillideg))).
                WillOnce(Return(16));
    ASSERT_TRUE(sut_->setPosition(1.0));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, setPositionReturnFalseIfWrongInput) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setPosition(M_PI * 2));
    ASSERT_FALSE(sut_->setPosition(M_PI * -2));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, setPositionReturnFalseIfCanDown) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    ASSERT_TRUE(sut_->initialize());

    // CAN read returns error
    EXPECT_CALL(*canSocketMock_, write(_)).WillOnce(Return(-1))
            .WillRepeatedly(Return(16));

    ASSERT_FALSE(sut_->setPosition(M_PI/2));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, setVelocityReturnFalseIfNotInitialized) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    ASSERT_FALSE(sut_->setVelocity(1.0));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setVelocity(M_PI / 2));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, setVelocityReturnFalseIfVelocityMovesOutOfPosLimits) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    ASSERT_TRUE(sut_->initialize());

    // Check when robot reached limits
    tempCanFrame_.can_id = MESSAGE_FROM_DEVICE+canID_;
    tempCanFrame_.data[1] = FRAG_START;
    tempCanFrame_.data[2] = GET_STATE;
    int position = sut_->getMaxPosition().get()*radToMilliDeg_*0.995;
    std::memcpy(&tempCanFrame_.data[3], &position, sizeof(int));
    ASSERT_TRUE(sut_->parseStatePeriodic(tempCanFrame_));
    ASSERT_FALSE(sut_->setVelocity(0.2));
    ASSERT_TRUE(sut_->setVelocity(-M_PI / 2));

    tempCanFrame_.data[1] = FRAG_START;
    tempCanFrame_.data[2] = GET_STATE;
    position = sut_->getMinPosition().get()*radToMilliDeg_*0.995;
    std::memcpy(&tempCanFrame_.data[3], &position, sizeof(int));
    ASSERT_TRUE(sut_->parseStatePeriodic(tempCanFrame_));
    ASSERT_FALSE(sut_->setVelocity(-0.1));
    ASSERT_TRUE(sut_->setVelocity(M_PI / 2));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, setVelocityReturnFalseIfVelocityOutOfBounds) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    ASSERT_TRUE(sut_->initialize());
    // Check if speed is too much
    ASSERT_FALSE(sut_->setVelocity(M_PI * 2));
    ASSERT_FALSE(sut_->setVelocity(M_PI * -2));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkDeviceShould, setVelocityHappyPath) {
    ASSERT_NO_THROW(sut_.reset(new SchunkDevice(
            canSocketMock_, canID_)));
    ASSERT_TRUE(sut_->initialize());
    int vel1 = M_PI / 2 * radToMilliDeg_;
    EXPECT_CALL(*canSocketMock_, write(FrameIntValueEquals(2, vel1))).
    WillOnce(Return(16));
    ASSERT_TRUE(sut_->setVelocity(M_PI / 2));
    int vel2 = -M_PI / 2 * radToMilliDeg_;
    EXPECT_CALL(*canSocketMock_, write(FrameIntValueEquals(2, vel2))).
    WillOnce(Return(16));
    ASSERT_TRUE(sut_->setVelocity(-M_PI / 2));

    ASSERT_TRUE(sut_->deinitialize());
}
