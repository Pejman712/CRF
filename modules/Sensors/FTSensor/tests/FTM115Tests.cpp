/* © Copyright CERN 2019.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <vector>
#include <queue>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "FTSensor/FTM115/FTM115.hpp"
#include "CANSocket/CANSocketMock.hpp"
#include "Types/Types.hpp"

#define SET_ACTIVE_CALIBRATION 0x206
#define READ_COEFFICIENT_MATRIX 0x202
#define READ_FT_DATA 0x200

using crf::utility::types::TaskForceTorque;
using crf::utility::types::TaskPose;
using crf::communication::cansocket::CANSocketMock;

using testing::_;
using testing::NiceMock;
using testing::Invoke;
using testing::Return;

class FTM115Should: public ::testing::Test {
 protected:
    FTM115Should():
        logger_("FTM115Should") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());

        std::string filename = __FILE__;
        filename = filename.substr(0, filename.find("tests"));
        filenameZero_ = filename + "tests/config/ftSensorCalibrationZero.json";
        filenameFive_ = filename + "tests/config/ftSensorCalibrationFive.json";

        messageQueue_ = std::queue<can_frame>();
        canSocketMock_.reset(new NiceMock<CANSocketMock>);
        ON_CALL(*canSocketMock_, initialize()).WillByDefault(Return(true));
        ON_CALL(*canSocketMock_, deinitialize()).WillByDefault(Return(true));
        ON_CALL(*canSocketMock_, write(_)).WillByDefault(Invoke([this](can_frame* frame) {
            return putResponseIntoQueue(frame);}));
        ON_CALL(*canSocketMock_, read(_)).WillByDefault(Invoke([this](can_frame* frame) {
            return feedTheCANbus(frame);}));
    }

    ~FTM115Should() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    int putResponseIntoQueue(can_frame* frame) {
        // Code to generate eye(6) as a calibration matrix
        if (frame->can_id == READ_COEFFICIENT_MATRIX) {
            for (int i = 0; i < 3; i++) {
                can_frame ackCANFrame{};

                // Creating zero according to FTM115 format
                float zero = 0;
                char zeroBigEndian[4];
                std::memcpy(&zeroBigEndian, &zero, sizeof(float));
                char zeroLittleEndian[4];
                for (int j = 0; j < 4; j++) {
                    zeroLittleEndian[j] = zeroBigEndian[3 - j];
                }
                std::memcpy(&ackCANFrame.data[0], &zeroLittleEndian, sizeof(float));
                std::memcpy(&ackCANFrame.data[4] , &zeroLittleEndian, sizeof(float));

                // Creating one million according to FTM115 format
                float one = 1e6;
                char oneBigEndian[4];
                std::memcpy(&oneBigEndian, &one, sizeof(float));
                char oneLittleEndian[4];

                for (int j = 0; j < 4; j++) {
                    oneLittleEndian[j] = oneBigEndian[3 - j];
                }

                ackCANFrame.can_id = READ_COEFFICIENT_MATRIX + i;
                // Check if we write 1 here
                if (frame->data[0] / 2 == i) {
                    // Check which position to write it
                    int position = (frame->data[0] % 2) *4;
                    std::memcpy(&ackCANFrame.data[position], &oneLittleEndian, sizeof(float));
                }
                messageQueue_.push(ackCANFrame);
            }
        } else if (frame->can_id == SET_ACTIVE_CALIBRATION) {
            can_frame ackCANFrame{};
            ackCANFrame.can_id = SET_ACTIVE_CALIBRATION;
            messageQueue_.push(ackCANFrame);
        } else if (frame->can_id == READ_FT_DATA) {
            can_frame ackCANFrame{};
            ackCANFrame.can_id = READ_FT_DATA;
            int8_t dataBigEndian[2];
            std::memcpy(&dataBigEndian, &defaultFTVal_, sizeof(int16_t));
            int8_t dataLittleEndian[2];
            std::memcpy(&dataLittleEndian[0], &dataBigEndian[1], sizeof(int8_t));
            std::memcpy(&dataLittleEndian[1], &dataBigEndian[0], sizeof(int8_t));

            for (int i = 0; i < 3; i++) {
                std::memcpy(&ackCANFrame.data[2* (i+1)], &dataLittleEndian, sizeof(int16_t));
            }
            messageQueue_.push(ackCANFrame);

            ackCANFrame.can_id = READ_FT_DATA + 1;
            for (int i = 0; i < 3; i++) {
                std::memcpy(&ackCANFrame.data[2*i], &dataLittleEndian, sizeof(int16_t));
            }
            messageQueue_.push(ackCANFrame);
        } else {
            return -1;
        }
        return 16;
    }

    int feedTheCANbus(can_frame* frame) {
        if (!messageQueue_.empty()) {
            std::memcpy(frame, &messageQueue_.front(), sizeof(can_frame));
            messageQueue_.pop();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return 16;
        }
        can_frame empty_frame{};
        std::memcpy(frame, &empty_frame, sizeof(can_frame));
        return -1;
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<CANSocketMock> canSocketMock_;
    std::queue<can_frame> messageQueue_;
    int16_t defaultFTVal_ = 5;
    const TaskPose rotationAroundZ_ =
        TaskPose({0, 0, 0}, crf::math::rotation::CardanXYZ({0, 0, 2.322}));
    std::string filenameZero_;
    std::string filenameFive_;
    std::unique_ptr<crf::sensors::ftsensor::FTM115> sut_;
};

TEST_F(FTM115Should, initDeinitWorksFine) {
    sut_.reset(new crf::sensors::ftsensor::FTM115(canSocketMock_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(FTM115Should, initFailsIfNoCAN) {
    sut_.reset(new crf::sensors::ftsensor::FTM115(canSocketMock_));
    EXPECT_CALL(*canSocketMock_, write(_)).WillOnce(Return(-1));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(FTM115Should, returnsGoodValue) {
    sut_.reset(new crf::sensors::ftsensor::FTM115(canSocketMock_));
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    TaskForceTorque ct1 = sut_->getRawFT();
    TaskForceTorque ct2 = sut_->getFT();

    Eigen::Matrix3d rotationMatrix =
        rotationAroundZ_.getHomogeneousTransformationMatrix().block<3, 3>(0, 0);

    TaskForceTorque t1{static_cast<double>(defaultFTVal_), static_cast<double>(defaultFTVal_),
        static_cast<double>(defaultFTVal_), static_cast<double>(defaultFTVal_),
        static_cast<double>(defaultFTVal_), static_cast<double>(defaultFTVal_)};
    Eigen::Vector3d forces(t1[0], t1[1], t1[2]);
    Eigen::Vector3d torques(t1[3], t1[4], t1[5]);
    Eigen::Vector3d forces_rotated = rotationMatrix*forces;
    Eigen::Vector3d torques_rotated = rotationMatrix*torques;
    t1 = crf::utility::types::TaskForceTorque(
        {forces_rotated(0), forces_rotated(1), forces_rotated(2),
        torques_rotated(0), torques_rotated(1), torques_rotated(2)});
    ASSERT_TRUE(areAlmostEqual(ct1, t1, 0.1));
    ASSERT_TRUE(areAlmostEqual(ct2, t1, 0.1));

    // Change the value
    defaultFTVal_ = 20;
    TaskForceTorque t2{static_cast<double>(defaultFTVal_), static_cast<double>(defaultFTVal_),
        static_cast<double>(defaultFTVal_), static_cast<double>(defaultFTVal_),
        static_cast<double>(defaultFTVal_), static_cast<double>(defaultFTVal_)};
    Eigen::Vector3d forces2(t2[0], t2[1], t2[2]);
    Eigen::Vector3d torques2(t2[3], t2[4], t2[5]);
    Eigen::Vector3d forces_rotated2 = rotationMatrix*forces2;
    Eigen::Vector3d torques_rotated2 = rotationMatrix*torques2;
    t2 = crf::utility::types::TaskForceTorque(
        {forces_rotated2(0), forces_rotated2(1), forces_rotated2(2),
        torques_rotated2(0), torques_rotated2(1), torques_rotated2(2)});
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    TaskForceTorque ct3 = sut_->getRawFT();
    ASSERT_TRUE(areAlmostEqual(ct3, t2, 0.1));

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    TaskForceTorque ct4 = sut_->getRawFT();
    ASSERT_TRUE(areAlmostEqual(ct4, t2, 0.1));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(FTM115Should, updateBiasReturnsFalseIfSensorNotInit) {
    sut_.reset(new crf::sensors::ftsensor::FTM115(canSocketMock_));
    ASSERT_FALSE(sut_->updateBias(TaskPose(), ""));
}

TEST_F(FTM115Should, updateBiasZerosOutBias) {
    sut_.reset(new crf::sensors::ftsensor::FTM115(canSocketMock_));
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_TRUE(sut_->updateBias(TaskPose(), filenameZero_));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    TaskForceTorque ct0 = sut_->getFT();

    ct0 = {0, 0, 0, 0, 0, 0};
    ASSERT_TRUE(areAlmostEqual(ct0, TaskForceTorque(), 0.001));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(FTM115Should, updateBiasRemovesWeight) {
    sut_.reset(new crf::sensors::ftsensor::FTM115(canSocketMock_));
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_TRUE(sut_->updateBias(TaskPose(), filenameFive_));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    TaskForceTorque ct5 = sut_->getFT();
    // This is the straight up position
    TaskForceTorque zeroPos{};
    zeroPos[2] = -5;
    ASSERT_TRUE(areAlmostEqual(ct5, zeroPos, 0.001));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(FTM115Should, getFTGravityFreeReturnsZeroIfNotInit) {
    sut_.reset(new crf::sensors::ftsensor::FTM115(canSocketMock_));
    TaskPose cp{};
    TaskForceTorque ct = sut_->getFTGravityFree(TaskPose{});
    ASSERT_TRUE(areAlmostEqual(ct, TaskForceTorque{}, 0.001));

    ASSERT_TRUE(sut_->initialize());
    ct = sut_->getFTGravityFree(TaskPose{});
    ASSERT_TRUE(areAlmostEqual(ct, TaskForceTorque{}, 0.001));

    ASSERT_TRUE(sut_->updateBias(TaskPose(), filenameFive_));
    ct = sut_->getFTGravityFree(TaskPose{});
    ASSERT_FALSE(areAlmostEqual(ct, TaskForceTorque{}, 0.001));
    ASSERT_TRUE(sut_->deinitialize());
}


TEST_F(FTM115Should, getFTGravityFreeReturnsZeroValuesIfCalibrated) {
    sut_.reset(new crf::sensors::ftsensor::FTM115(canSocketMock_));

    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_TRUE(sut_->updateBias(TaskPose(), filenameFive_));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    TaskForceTorque ct = sut_->getFTGravityFree(TaskPose{});
    ASSERT_TRUE(areAlmostEqual(ct, TaskForceTorque{}, 0.001));
    ASSERT_TRUE(sut_->deinitialize());
}
using testing::Return;
