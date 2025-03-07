/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/TaskTypes/TaskSignals.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::TaskSignals;

class TaskSignalsShould : public ::testing::Test {
 protected:
    TaskSignalsShould() :
        sut_(std::make_unique<TaskSignals>()) {
    }
    std::unique_ptr<TaskSignals> sut_;

    crf::expected<TaskPose> taskPoseOutput_;
    crf::expected<TaskVelocity> taskVelocityOutput_;
    crf::expected<TaskAcceleration> taskAccelerationOutput_;
    crf::expected<TaskForceTorque> taskForceTorqueOutput_;
};

TEST_F(TaskSignalsShould, HaveCorrectFields) {
    ASSERT_NO_THROW(taskPoseOutput_ = sut_->pose);
    ASSERT_NO_THROW(taskVelocityOutput_ = sut_->velocity);
    ASSERT_NO_THROW(taskAccelerationOutput_ = sut_->acceleration);
    ASSERT_NO_THROW(taskForceTorqueOutput_ = sut_->forceTorque);
}
