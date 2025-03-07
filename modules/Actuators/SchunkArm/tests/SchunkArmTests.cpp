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
#include <string>
#include <vector>
#include <fstream>

#include "EventLogger/EventLogger.hpp"
#include "SchunkArm/SchunkCommands.hpp"
#include "SchunkArm/SchunkArm.hpp"
#include "SchunkArm/SchunkGripper.hpp"
#include "CANSocket/CANSocketMock.hpp"

using crf::utility::logger::EventLogger;
using crf::actuators::schunkarm::SchunkArm;
using crf::actuators::schunkarm::SchunkGripper;
using crf::communication::cansocket::CANSocketMock;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;
using testing::AtLeast;
using testing::Ge;
using testing::AllOf;

MATCHER_P2(FrameDataEquals, index, value, "") {
    return (arg->data[index] == value);
}

MATCHER_P2(FrameIntValueEquals, firstByte, value, "") {
    int readValue;
    std::memcpy(&readValue, &arg->data[firstByte], sizeof(int));
    return (readValue == value);
}

MATCHER_P3(CanMessageAcceleratesTowardsGoal, currentPos, goalPos, currentVelocity, "") {
    int velocity;
    std::memcpy(&velocity, &arg->data[2], sizeof(int));
    // we dont care about the scaling, it depends on the controller
    float velRadian = velocity / 180000.0 * M_PI;
    bool moveTowardsGoal = (velRadian - currentVelocity) * (goalPos - currentPos) > 0;
    // if we are at the goalPos we have to stop
    bool weAreAtGoalPos = ((goalPos - currentPos) == 0);
    return (moveTowardsGoal || weAreAtGoalPos);
}

class SchunkArmShould: public ::testing::Test {
 protected:
    SchunkArmShould():
    tempCanFrame_{},
    logger_("SchunkArmShould") {
        logger_->info("{} BEGIN",
                      testing::UnitTest::GetInstance()->current_test_info()->name());
        canSocketMock_.reset(new NiceMock<CANSocketMock>);

        ON_CALL(*canSocketMock_, initialize()).WillByDefault(Return(true));
        ON_CALL(*canSocketMock_, deinitialize()).WillByDefault(Return(true));
        ON_CALL(*canSocketMock_, write(_)).WillByDefault(Invoke(
                [this](can_frame* frame) {
                    return putAcknowledmentIntoQueue(frame);}));
        ON_CALL(*canSocketMock_, read(_)).WillByDefault(Invoke(
                [this](can_frame* frame) {
                    return feedTheCanbus(frame);}));

        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0,
            testDirName_.find("SchunkArmTests.cpp"));
        testDirName_ += "configuration/";
    }
    ~SchunkArmShould() {
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
        tempCanFrame_.data[5] = REFERENCED_BRAKEOFF;
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
                ackCanFrame.data[2] = REFERENCED_BRAKEOFF;
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
            std::chrono::milliseconds duration(sut_->getConfiguration()->getRTLoopTime());
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

    std::queue<can_frame> initializer;
    std::vector<int> sendStatePeriodic_;
    can_frame tempCanFrame_;
    const int maxPos = 170000;
    const int minPos = -170000;
    const int maxSpeed = 120000;
    const int maxAcc = 500000;
    const int maxCur = 3900;
    float startPosition = M_PI/2;
    float startVelocity = M_PI/20;
    u_int8_t robotDefaultState_ = REFERENCED_BRAKEOFF;
    u_int8_t robotDefaultError_ = 0x00;
    const float radToMilliDeg_ = 180.0 / M_PI *1000;
    const int numberOfJoints_ = 6;

    EventLogger logger_;
    std::shared_ptr<CANSocketMock> canSocketMock_;
    std::shared_ptr<SchunkGripper> gripper_ = nullptr;
    const float toleranceForFloatEquality = 0.001;

    std::unique_ptr<SchunkArm> sut_;
    std::string testDirName_;
};

TEST_F(SchunkArmShould, returnFalseWhenDoubleInitializedDeinitialized) {
    std::ifstream robotData(testDirName_ + "SchunkLWA4P.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new SchunkArm(canSocketMock_, robotJSON, gripper_)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(SchunkArmShould, initializeReturnFalseIfNoCan) {
    std::ifstream robotData(testDirName_ + "SchunkLWA4P.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new SchunkArm(canSocketMock_, robotJSON, gripper_)));

    EXPECT_CALL(*canSocketMock_, write(_)).WillOnce(Return(-1)).WillRepeatedly(DoDefault());
    ASSERT_FALSE(sut_->initialize());

    EXPECT_CALL(*canSocketMock_, read(_)).WillRepeatedly(Return(-1));
    ASSERT_FALSE(sut_->initialize());

    EXPECT_CALL(*canSocketMock_, read(_)).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkArmShould, deinitializeStopsArm) {
    std::ifstream robotData(testDirName_ + "SchunkLWA4P.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new SchunkArm(canSocketMock_, robotJSON, gripper_)));

    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, CMD_STOP))).
                Times(6).WillRepeatedly(Return(16));
    EXPECT_CALL(*canSocketMock_, write(
            AllOf(FrameDataEquals(1, GET_STATE), FrameIntValueEquals(2, 0)))).
            Times(6).WillRepeatedly(Return(16));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkArmShould, getJointsPosAndVelReturnFalseIfNotInitialized) {
    std::ifstream robotData(testDirName_ + "SchunkLWA4P.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new SchunkArm(canSocketMock_, robotJSON, gripper_)));

    ASSERT_FALSE(sut_->getJointPositions());
    ASSERT_FALSE(sut_->getJointVelocities());

    ASSERT_TRUE(sut_->initialize());
    for (int i=0; i < numberOfJoints_; i++) {
        ASSERT_NEAR(sut_->getJointPositions().get()[i], startPosition, toleranceForFloatEquality);
        ASSERT_NEAR(sut_->getJointVelocities().get()[i], startVelocity, toleranceForFloatEquality);
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkArmShould, testStateReadingLoop) {
    std::ifstream robotData(testDirName_ + "SchunkLWA4P.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new SchunkArm(canSocketMock_, robotJSON, gripper_)));

    ASSERT_TRUE(sut_->initialize());
    JointPositions pos = sut_->getJointPositions().get();
    JointVelocities vel = sut_->getJointVelocities().get();
    for (int i=0; i < numberOfJoints_; i++) {
        ASSERT_NEAR(pos[i], startPosition, toleranceForFloatEquality);
        ASSERT_NEAR(vel[i], startVelocity, toleranceForFloatEquality);
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkArmShould, stopArmPutOnBreaksAndCANSilent) {
    std::ifstream robotData(testDirName_ + "SchunkLWA4P.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new SchunkArm(canSocketMock_, robotJSON, gripper_)));

    ASSERT_FALSE(sut_->stopArm());
    JointPositions startPos({startPosition, startPosition, startPosition,
        startPosition, startPosition, startPosition});
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setJointPositions(startPos));
    EXPECT_CALL(*canSocketMock_, write(FrameDataEquals(1, CMD_STOP))).
                Times(6).WillRepeatedly(Return(16));
    ASSERT_TRUE(sut_->stopArm());
    EXPECT_CALL(*canSocketMock_, write(_)).WillRepeatedly(Return(16));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkArmShould, setJointVelocitiesReturnFalseIfNotInitialized) {
    std::ifstream robotData(testDirName_ + "SchunkLWA4P.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new SchunkArm(canSocketMock_, robotJSON, gripper_)));

    ASSERT_FALSE(sut_->setJointVelocities(JointVelocities(numberOfJoints_)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setJointVelocities(JointVelocities(numberOfJoints_)));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkArmShould, setJointVelocitiesReturnFalseIfWrongInput) {
    std::ifstream robotData(testDirName_ + "SchunkLWA4P.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new SchunkArm(canSocketMock_, robotJSON, gripper_)));

    JointVelocities bigVelocity1({M_PI*10, M_PI*10, M_PI*10, M_PI*10, M_PI*10, M_PI*10});
    JointVelocities bigVelocity2(bigVelocity1.raw()*(-1.0));
    JointVelocities goodVelocity1({M_PI/10, M_PI/10, M_PI/10, M_PI/10, M_PI/10, M_PI/10});
    JointVelocities goodVelocity2(goodVelocity1.raw()*(-1.0));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setJointVelocities(bigVelocity1));
    ASSERT_FALSE(sut_->setJointVelocities(bigVelocity2));
    ASSERT_TRUE(sut_->setJointVelocities(goodVelocity1));
    ASSERT_TRUE(sut_->setJointVelocities(goodVelocity2));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkArmShould, setJointPositionsReturnFalseIfNotInitialized) {
    std::ifstream robotData(testDirName_ + "SchunkLWA4P.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new SchunkArm(canSocketMock_, robotJSON, gripper_)));

    ASSERT_FALSE(sut_->setJointPositions(JointPositions(numberOfJoints_)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setJointPositions(JointPositions(numberOfJoints_)));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkArmShould, setJointPositionsReturnFalseIfWrongInput) {
    std::ifstream robotData(testDirName_ + "SchunkLWA4P.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new SchunkArm(canSocketMock_, robotJSON, gripper_)));

    ASSERT_TRUE(sut_->initialize());
    JointPositions outOfRange({M_PI, M_PI, M_PI, M_PI, M_PI, M_PI});
    JointPositions inRange({M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2});
    ASSERT_FALSE(sut_->setJointPositions(outOfRange));
    ASSERT_FALSE(sut_->setJointPositions(
        crf::utility::types::JointPositions(outOfRange.raw()*(-1.0))));
    ASSERT_TRUE(sut_->setJointPositions(inRange));
    ASSERT_TRUE(sut_->setJointPositions(
        crf::utility::types::JointPositions(inRange.raw()*(-1.0))));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(SchunkArmShould, notImplementedOperationsReturnNoneOrFalse) {
    std::ifstream robotData(testDirName_ + "SchunkLWA4P.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new SchunkArm(canSocketMock_, robotJSON, gripper_)));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getJointForceTorques());
    ASSERT_FALSE(sut_->getTaskPose());
    ASSERT_FALSE(sut_->getTaskVelocity());
    ASSERT_FALSE(sut_->setJointForceTorques(crf::utility::types::JointForceTorques(6)));
    ASSERT_FALSE(sut_->setTaskPose(crf::utility::types::TaskPose()));
    ASSERT_FALSE(sut_->setTaskVelocity(crf::utility::types::TaskVelocity(), false));
    ASSERT_TRUE(sut_->deinitialize());
}
