/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Signals.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::JointSignals;
using crf::utility::types::TaskSignals;
using crf::utility::types::Signals;

class SignalsShould : public ::testing::Test {
 protected:
    SignalsShould() :
        sut_(std::make_unique<Signals>()) {
    }
    std::unique_ptr<Signals> sut_;

    JointSignals jointSignalsOutput_;
    TaskSignals taskSignalsOutput_;
};

TEST_F(SignalsShould, HaveCorrectFields) {
    ASSERT_NO_THROW(jointSignalsOutput_ = sut_->joints);
    ASSERT_NO_THROW(taskSignalsOutput_ = sut_->task);
}
