/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <future>
#include <memory>
#include <string>
#include <chrono>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "Controller/PositionCtrlVelocityFF/PositionCtrlVelocityFF.hpp"
#include "Types/Signals.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::control::controller::IController;
using crf::control::controller::PositionCtrlVelocityFF;

class PositionCtrlVelocityFFShould: public ::testing::Test {
 protected:
    PositionCtrlVelocityFFShould() :
        logger_("PositionCtrlVelocityFFShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~PositionCtrlVelocityFFShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() {
        sut_ = std::make_unique<PositionCtrlVelocityFF>(
            std::vector<double>({1, 1, 1, 1}),
            std::vector<double>({0.1, 0.1, 0.1, 0.1}),
            std::vector<double>({0.01, 0.01, 0.01, 0.01}),
            2.0,
            nullptr);
    }

    crf::utility::types::JointSignals inputData_;
    crf::utility::types::TaskSignals taskFeedback_;
    crf::utility::types::JointSignals jointFeedback_;

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<PositionCtrlVelocityFF> sut_;
};

TEST_F(PositionCtrlVelocityFFShould, FailIfPositionFeedBackIsNotGiven) {
    inputData_.positions = crf::utility::types::JointPositions({2, 2, 2, 2});
    inputData_.velocities = crf::utility::types::JointVelocities({0, 0, 0, 0});
    inputData_.accelerations = crf::utility::types::JointAccelerations({0, 0, 0, 0});
    inputData_.forceTorques = crf::utility::types::JointForceTorques({0, 0, 0, 0});

    // No Feed-Back given

    ASSERT_THROW(sut_->calculate(inputData_, jointFeedback_, taskFeedback_), std::runtime_error);
}

TEST_F(PositionCtrlVelocityFFShould, ReturnVelocityOutputAndOthersEmpty) {
    // Input data has dimension 4
    inputData_.positions = crf::utility::types::JointPositions({2, 2, 2, 2});
    inputData_.velocities = crf::utility::types::JointVelocities({0, 0, 0, 0});
    inputData_.accelerations = crf::utility::types::JointAccelerations({0, 0, 0, 0});
    inputData_.forceTorques = crf::utility::types::JointForceTorques({0, 0, 0, 0});

    // Feed-Back data has dimension 4
    jointFeedback_.positions = crf::utility::types::JointPositions({1, 1, 1, 1});

    crf::utility::types::Signals result =
        sut_->calculate(inputData_, jointFeedback_, taskFeedback_);

    ASSERT_FALSE(result.joints.positions);
    ASSERT_TRUE(result.joints.velocities);
    ASSERT_FALSE(result.joints.accelerations);
    ASSERT_FALSE(result.joints.forceTorques);
    ASSERT_FALSE(result.task.pose);
    ASSERT_FALSE(result.task.velocity);
    ASSERT_FALSE(result.task.acceleration);
    ASSERT_FALSE(result.task.forceTorque);
}
