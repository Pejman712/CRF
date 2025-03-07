/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <vector>
#include <memory>

#include "EventLogger/EventLogger.hpp"
#include "StateEstimator/IStateEstimator.hpp"
#include "StateEstimator/StateEstimator.hpp"

#define OUTPUT 6
#define INPUT 3

using crf::algorithms::stateestimator::StateEstimatorFilterType;
using crf::algorithms::stateestimator::IStateEstimator;
using crf::algorithms::stateestimator::StateEstimator;

class StateEstimatorTestsShould: public ::testing::Test {
 protected:
    StateEstimatorTestsShould():
        logger_("StateEstimatorTestsShould"),
        initialState_{1.0, 1.0, 1.0, .0, .0, .0} {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~StateEstimatorTestsShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::array<float, OUTPUT> initialState_;
    std::unique_ptr<IStateEstimator> sut_;
};

TEST_F(StateEstimatorTestsShould, throwIfFilterModeUndefined) {
    ASSERT_THROW(sut_.reset(new StateEstimator<OUTPUT, INPUT>(initialState_,
        StateEstimatorFilterType::UNDEFINED_KF)), std::exception);
}

TEST_F(StateEstimatorTestsShould, returnZeroIfInitialVectorUndefined) {
    std::array<float, OUTPUT> initialState{};
    sut_.reset(new StateEstimator<OUTPUT, INPUT>(initialState));
    auto initialEstimate = sut_->getEstimate();
    for (int i = 0; i < initialEstimate.size(); i++) {
        ASSERT_FLOAT_EQ(initialEstimate[i], .0);
    }
}

TEST_F(StateEstimatorTestsShould, returnInitialState) {
    sut_.reset(new StateEstimator<OUTPUT, INPUT>(initialState_));
    auto initialEstimate = sut_->getEstimate();
    for (int i = 0; i < initialEstimate.size(); i++) {
        ASSERT_FLOAT_EQ(initialEstimate[i], initialState_[i]);
    }
}

TEST_F(StateEstimatorTestsShould, convergeEstimationToValueIfMeasurementStatic) {
    // SUT recreation with different template parameters
    std::unique_ptr<IStateEstimator> sut;
    ASSERT_NO_THROW(sut.reset(new StateEstimator<OUTPUT, OUTPUT>(initialState_)));

    std::vector<float> measurementVector{1.2, 1.2, 1.2, 0.3, 0.4, 0.4};
    for (int i = 0; i < 50; i++) {
        sut->addMeasurement(measurementVector);
    }

    auto estimatedState = sut->getEstimate();
    for (int i = 0; i < estimatedState.size(); i++) {
        ASSERT_NEAR(estimatedState[i], measurementVector[i], 1e-3);
    }
}

// Commented since it results in desired static assertion fail
/*TEST_F(StateEstimatorTestsShould, throwStaticExceptionOnInputVectorZero) {
    std::array<float, 1> initialState;
    std::unique_ptr<IStateEstimator<1, 0>> sut;
    sut.reset(new StateEstimator<1, 0>(initialState));
}*/
/*TEST_F(StateEstimatorTestsShould, throwStaticExceptionOnInputGreaterStateVector) {
    std::array<float, 1> initialState;
    std::unique_ptr<IStateEstimator<1, 6>> sut;
    sut.reset(new StateEstimator<1, 6>(initialState));
}*/

