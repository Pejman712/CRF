/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <linux/can.h>
#include <boost/optional.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <queue>
#include <cstring>
#include <vector>
#include <thread>

#include "EventLogger/EventLogger.hpp"
#include "SchunkArm/SchunkCommands.hpp"
#include "Gripper/IGripper.hpp"
#include "SchunkArm/SchunkGripper.hpp"
#include "CANSocket/CANSocketMock.hpp"

using crf::utility::logger::EventLogger;
using crf::actuators::gripper::IGripper;
using crf::actuators::schunkarm::SchunkGripper;
using crf::communication::cansocket::CANSocketMock;

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;
using testing::AtLeast;
using testing::AllOf;

MATCHER_P2(FrameDataEquals, index, value, "") {
    return (arg->data[index] == value);
}

MATCHER_P2(FrameIntValueEquals, firstByte, value, "") {
    int readValue;
    std::memcpy(&readValue, &arg->data[firstByte], sizeof(int));
    return (readValue == value);
}

class SchunkGripperShould: public ::testing::Test {
 protected:
    SchunkGripperShould():
        tempCanFrame_{},
        logger_("SchunkGripperShould") {
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
    ~SchunkGripperShould() {
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
        int position = startPosition * radToMilliDeg_;
        int speed = startVelocity * radToMilliDeg_;
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
                int position = startPosition * radToMilliDeg_;
                int speed = startVelocity * radToMilliDeg_;
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
    const int minPos = -100;
    const int maxSpeed = 120000;
    const int maxAcc = 500000;
    const int maxCur = 3900;
    const float startPosition = 1.7;
    const float startVelocity = 1.2;
    u_int8_t robotDefaultState_ = REFERENCED_BREAKON;
    u_int8_t robotDefaultError_ = 0x00;

    EventLogger logger_;
    std::shared_ptr<CANSocketMock> canSocketMock_;
    std::unique_ptr<SchunkGripper> sut_;
    const float radToMilliDeg_ = 180.0 / M_PI *1000;
    const float millidegToRad_ = M_PI / 180 /1000;
    const float toleranceForFloatEquality = 0.001;
};

TEST_F(SchunkGripperShould, returnFalseWhenDoubleInitializedDeinitialized) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(
            canSocketMock_)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, initializeSetsCANDefaultValues) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(
            canSocketMock_)));
    EXPECT_CALL(*canSocketMock_, write(_)).WillRepeatedly(DoDefault());
    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, SET_TARGET_VEL))).
                Times(AtLeast(1)).WillRepeatedly(DoDefault());
    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, SET_TARGET_ACC))).
                Times(AtLeast(1)).WillRepeatedly(DoDefault());
    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, SET_TARGET_CUR))).
                Times(AtLeast(1)).WillRepeatedly(DoDefault());
    EXPECT_CALL(*canSocketMock_, write(
        AllOf(FrameDataEquals(1, GET_STATE), FrameIntValueEquals(2, 40)))).WillOnce(Return(16));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, deinitializeSilencesCAN) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(
            canSocketMock_)));
    ASSERT_TRUE(sut_->initialize());

    EXPECT_CALL(*canSocketMock_, write(
            AllOf(FrameDataEquals(1, GET_STATE), FrameIntValueEquals(2, 0)))).WillOnce(Return(16));
    ASSERT_TRUE(sut_->deinitialize());
}
TEST_F(SchunkGripperShould, updateStateReturnsFalseIfNotRightCANID) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(
            canSocketMock_)));
    can_frame frame{};
    frame.can_id = MESSAGE_FROM_DEVICE + JOINT4;
    frame.can_dlc = 0x07;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = MOVE_VEL;
    frame.data[2] = ASCII_O;
    frame.data[3] = ASCII_K;

    ASSERT_FALSE(sut_->updateState(frame));  // not initialized
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->updateState(frame));  // wrong canid

    frame.can_id = MESSAGE_FROM_DEVICE + GRIPPER;
    ASSERT_TRUE(sut_->updateState(frame));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, updateStateWorksOnPositionVelocityAndGrasping) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(
            canSocketMock_)));
    ASSERT_TRUE(sut_->initialize());

    can_frame frame{};
    frame.can_id = MESSAGE_FROM_DEVICE + GRIPPER;
    frame.can_dlc = 0x07;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = FRAG_START;
    frame.data[2] = GET_STATE;
    float positionPercentage = 70;
    int value = maxPos * (100 - positionPercentage) / 100;
    int speed = 0;
    std::memcpy(&frame.data[3], &value, sizeof(int));
    std::memcpy(&frame.data[7], &speed, sizeof(char));

    ASSERT_TRUE(sut_->updateState(frame));  // update position
    ASSERT_NEAR(sut_->getPosition().get(), positionPercentage, toleranceForFloatEquality);

    std::memcpy(&frame.data[1], &speed, sizeof(int));
    frame.data[1] = FRAG_END;
    frame.data[5] = MOTION_BLOCKED;

    ASSERT_TRUE(sut_->updateState(frame));  // update velocity and state
    ASSERT_TRUE(sut_->isGrasping());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, updateStateHandlesError) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(
            canSocketMock_)));
    ASSERT_TRUE(sut_->initialize());


    can_frame frame{};
    frame.can_id = MESSAGE_FROM_DEVICE + GRIPPER;
    frame.can_dlc = 0x07;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = CMD_ERROR;
    frame.data[2] = ERROR_SOFT_LOW;

    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, CMD_ACK))).WillOnce(Return(16));
    ASSERT_TRUE(sut_->updateState(frame));  // update error

    EXPECT_CALL(*canSocketMock_, write(_)).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, getPositionReturnsTheDefaultValue) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(
            canSocketMock_)));
    ASSERT_FALSE(sut_->getPosition());  // not initialized
    ASSERT_TRUE(sut_->initialize());

    boost::optional<float> pos = sut_->getPosition();
    float defaultPosInPercentage = startPosition * radToMilliDeg_ / maxPos * 100;
    ASSERT_NEAR(pos.get(), 100 - defaultPosInPercentage, toleranceForFloatEquality);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, isGraspingReturnsTrue) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(canSocketMock_)));
    ASSERT_FALSE(sut_->isGrasping());  // not initialized
    ASSERT_TRUE(sut_->initialize());

    // it is initialized with not grasping
    ASSERT_FALSE(sut_->isGrasping());

    can_frame frame{};
    frame.can_id = MESSAGE_FROM_DEVICE + GRIPPER;
    frame.can_dlc = 0x07;
    frame.data[0] = frame.can_dlc - 1;
    int speed = 0;
    std::memcpy(&frame.data[1], &speed, sizeof(int));
    frame.data[1] = FRAG_END;
    frame.data[5] = MOTION_BLOCKED;
    ASSERT_TRUE(sut_->updateState(frame));  // update velocity and state
    ASSERT_TRUE(sut_->isGrasping());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, isGraspingReturnsFalseWhenClosed) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(canSocketMock_)));
    ASSERT_TRUE(sut_->initialize());

    can_frame frame{};
    frame.can_id = MESSAGE_FROM_DEVICE + GRIPPER;
    frame.can_dlc = 0x07;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = FRAG_START;
    frame.data[2] = GET_STATE;
    float positionPercentage = 0;
    int value = maxPos * positionPercentage / 100;
    int speed = 0;
    std::memcpy(&frame.data[3], &value, sizeof(int));
    std::memcpy(&frame.data[7], &speed, sizeof(char));

    ASSERT_TRUE(sut_->updateState(frame));  // update position

    std::memcpy(&frame.data[1], &speed, sizeof(int));
    frame.data[1] = FRAG_END;
    frame.data[5] = MOTION_BLOCKED;

    ASSERT_TRUE(sut_->updateState(frame));  // update velocity and state
    ASSERT_FALSE(sut_->isGrasping());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, isGraspingReturnsFalseWhenOpen) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(canSocketMock_)));
    ASSERT_TRUE(sut_->initialize());

    can_frame frame;
    frame.can_id = MESSAGE_FROM_DEVICE + GRIPPER;
    frame.can_dlc = 0x07;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = FRAG_START;
    frame.data[2] = GET_STATE;
    float positionPercentage = 100;
    int value = maxPos * positionPercentage / 100;
    int speed = 0;
    std::memcpy(&frame.data[3], &value, sizeof(int));
    std::memcpy(&frame.data[7], &speed, sizeof(char));

    ASSERT_TRUE(sut_->updateState(frame));  // update position

    std::memcpy(&frame.data[1], &speed, sizeof(int));
    frame.data[1] = FRAG_END;
    frame.data[5] = MOTION_BLOCKED;

    ASSERT_TRUE(sut_->updateState(frame));  // update velocity and state
    ASSERT_FALSE(sut_->isGrasping());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, stopGripperStopsDevice) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(
            canSocketMock_)));
    ASSERT_TRUE(sut_->initialize());

    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, CMD_STOP))).WillOnce(Return(16));
    ASSERT_TRUE(sut_->stopGripper());

    EXPECT_CALL(*canSocketMock_, write(_)).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, setGraspingForceCheckInput) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(
            canSocketMock_)));
    ASSERT_FALSE(sut_->setGraspingForce(50));  // not initialized

    ASSERT_TRUE(sut_->initialize());

    ASSERT_FALSE(sut_->setGraspingForce(102));
    ASSERT_FALSE(sut_->setGraspingForce(-3));

    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, SET_TARGET_CUR))).WillOnce(Return(16));
    ASSERT_TRUE(sut_->setGraspingForce(30));

    EXPECT_CALL(*canSocketMock_, write(_)).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, setPositionReturnsFalse) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(
            canSocketMock_)));
    ASSERT_FALSE(sut_->setPosition(50));  // not initialized

    ASSERT_TRUE(sut_->initialize());

    // wrong input
    ASSERT_FALSE(sut_->setPosition(102));
    ASSERT_FALSE(sut_->setPosition(-3));

    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, MOVE_POS))).WillOnce(Return(16));
    ASSERT_TRUE(sut_->setPosition(30));

    EXPECT_CALL(*canSocketMock_, write(_)).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, setPositionOpensCloses) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(
            canSocketMock_)));
    ASSERT_TRUE(sut_->initialize());

    // Opens
    int open = maxPos;
    EXPECT_CALL(*canSocketMock_, write(
            AllOf(FrameDataEquals(1, MOVE_POS), FrameIntValueEquals(2, open)))).
            Times(2).WillRepeatedly(Return(16));
    ASSERT_TRUE(sut_->setPosition(0));
    ASSERT_TRUE(sut_->setPosition(IGripper::Gripper_Open));

    // Closes
    int closed = 0;
    EXPECT_CALL(*canSocketMock_, write(
            AllOf(FrameDataEquals(1, MOVE_POS), FrameIntValueEquals(2, closed)))).
            Times(2).WillRepeatedly(Return(16));
    ASSERT_TRUE(sut_->setPosition(100));
    ASSERT_TRUE(sut_->setPosition(IGripper::Gripper_Closed));

    EXPECT_CALL(*canSocketMock_, write(_)).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, setVelocityReturnsFalse) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(
            canSocketMock_)));
    ASSERT_FALSE(sut_->setVelocity(50));  // not initialized

    ASSERT_TRUE(sut_->initialize());

    // wrong input
    ASSERT_FALSE(sut_->setVelocity(102));
    ASSERT_FALSE(sut_->setVelocity(-103));

    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, MOVE_POS))).WillOnce(Return(16));
    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, SET_TARGET_VEL))).
        WillOnce(DoDefault());
    ASSERT_TRUE(sut_->setVelocity(30));

    EXPECT_CALL(*canSocketMock_, write(_)).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, setVelocityNegativeValueOpens) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(
            canSocketMock_)));
    ASSERT_TRUE(sut_->initialize());

    // Opens
    float speedPercentage = -25;
    int open = maxPos;
    EXPECT_CALL(*canSocketMock_, write(
            AllOf(FrameDataEquals(1, MOVE_POS), FrameIntValueEquals(2, open)))).
            WillOnce(Return(16));
    int speed = maxSpeed * std::fabs(speedPercentage) / 100;
    EXPECT_CALL(*canSocketMock_, write(
            AllOf(FrameDataEquals(1, SET_TARGET_VEL), FrameIntValueEquals(2, speed)))).
            WillOnce(Return(16));
    ASSERT_TRUE(sut_->setVelocity(speedPercentage));

    EXPECT_CALL(*canSocketMock_, write(_)).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkGripperShould, setVelocityPositiveValueCloses) {
    ASSERT_NO_THROW(sut_.reset(new SchunkGripper(
            canSocketMock_)));
    ASSERT_TRUE(sut_->initialize());

    // closes
    float speedPercentage = 50;
    int closed = 0;
    EXPECT_CALL(*canSocketMock_, write(
            AllOf(FrameDataEquals(1, MOVE_POS), FrameIntValueEquals(2, closed)))).
            WillOnce(Return(16));
    int speed = maxSpeed * std::fabs(speedPercentage) / 100;
    EXPECT_CALL(*canSocketMock_, write(
            AllOf(FrameDataEquals(1, SET_TARGET_VEL), FrameIntValueEquals(2, speed)))).
            WillOnce(Return(16));
    ASSERT_TRUE(sut_->setVelocity(speedPercentage));

    EXPECT_CALL(*canSocketMock_, write(_)).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->deinitialize());
}
