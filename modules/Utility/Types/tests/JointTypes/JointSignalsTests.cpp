/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/JointTypes/JointSignals.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;
using crf::utility::types::JointSignals;

class JointSignalsShould : public ::testing::Test {
 protected:
    JointSignalsShould() :
    sut_(std::make_unique<JointSignals>()) {
    }
    std::unique_ptr<JointSignals> sut_;

    crf::expected<JointPositions> jointPositionsOutput_;
    crf::expected<JointVelocities> jointVelocitiesOutput_;
    crf::expected<JointAccelerations> jointAccelerationsOutput_;
    crf::expected<JointForceTorques> jointForceTorquesOutput_;
};

TEST_F(JointSignalsShould, HaveCorrectFields) {
    ASSERT_NO_THROW(jointPositionsOutput_ = sut_->positions);
    ASSERT_NO_THROW(jointVelocitiesOutput_ = sut_->velocities);
    ASSERT_NO_THROW(jointAccelerationsOutput_ = sut_->accelerations);
    ASSERT_NO_THROW(jointForceTorquesOutput_ = sut_->forceTorques);
}
