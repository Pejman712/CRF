/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <exception>
#include <memory>
#include <vector>
#include <thread>

#include "EventLogger/EventLogger.hpp"
#include "ClosedLoopController/PIDController.hpp"

using crf::control::closedloopcontroller::IClosedLoopController;
using crf::control::closedloopcontroller::PIDController;

using namespace std::chrono_literals;  // NOLINT

bool operator <(const std::vector<double>& lhs, const std::vector<double>& rhs) {
    if (lhs.size() != rhs.size()) {
        throw std::invalid_argument("lhs.size() != rhs.size()");
    }
    for (size_t i = 0; i < lhs.size(); i++) {
        if (lhs[i] >= rhs[i]) {
            return false;
        }
    }
    return true;
}

class PIDControllerShould: public ::testing::Test {
 protected:
    PIDControllerShould(): logger_("PIDControllerShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~PIDControllerShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<IClosedLoopController> sut_;
};

TEST_F(PIDControllerShould, throwIfConstructedWithEmptyVectors) {
    std::vector<double> kp;
    std::vector<double> ki;
    std::vector<double> kd;
    ASSERT_THROW(sut_.reset(new PIDController(kp, ki, kd)), std::invalid_argument);
}

TEST_F(PIDControllerShould, throwIfConstructedWithVectorsOfDifferentSizes) {
    std::vector<double> kp(6);
    std::vector<double> ki(6);
    std::vector<double> kd(5);
    ASSERT_THROW(sut_.reset(new PIDController(kp, ki, kd)), std::invalid_argument);
}

TEST_F(PIDControllerShould, returnNoneIfCalledWithWrongSizeArgs) {
    std::vector<double> kp(3);
    std::vector<double> ki(3);
    std::vector<double> kd(3);
    kp.assign({0.1, 0.2, 0.3});
    ki.assign({0.1, 0.1, 0.1});
    kd.assign({0.0, 0.0, 0.0});
    ASSERT_NO_THROW(sut_.reset(new PIDController(kp, ki, kd)));
    std::vector<double> goal(3);
    std::vector<double> feedback(2);
    ASSERT_FALSE(sut_->calculate(goal, feedback));
    goal.resize(2);
    ASSERT_FALSE(sut_->calculate(goal, feedback));
}

TEST_F(PIDControllerShould, returnIncreasingControlOutputIfSystemStateDoesNotChangeOverTime) {
    std::vector<double> kp(3);
    std::vector<double> ki(3);
    std::vector<double> kd(3);
    kp.assign({0.1, 0.2, 0.3});
    ki.assign({0.1, 0.1, 0.1});
    kd.assign({0.0, 0.0, 0.0});
    ASSERT_NO_THROW(sut_.reset(new PIDController(kp, ki, kd)));
    std::vector<double> goal(3);
    std::vector<double> feedback(3);
    goal.assign({1.0, 1.0, 1.0});
    feedback.assign({0.5, 0.5, 0.5});
    auto result1 = sut_->calculate(goal, feedback);
    std::this_thread::sleep_for(1ms);
    auto result2 = sut_->calculate(goal, feedback);
    std::this_thread::sleep_for(1ms);
    auto result3 = sut_->calculate(goal, feedback);
    std::this_thread::sleep_for(1ms);
    ASSERT_TRUE(result1);
    ASSERT_TRUE(result2);
    ASSERT_TRUE(result3);
    ASSERT_LT(*result1, *result2);
    ASSERT_LT(*result2, *result3);
}
