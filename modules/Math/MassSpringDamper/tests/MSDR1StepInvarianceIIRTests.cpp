/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *         David Günter Forkel CERN EN/CEM/MRO 2021
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
#include "MassSpringDamper/MSDR1/MSDR1StepInvarianceIIR.hpp"

using crf::math::massspringdamper::MSDR1StepInvarianceIIR;
using crf::math::massspringdamper::SignalR1;

class MSDR1StepInvarianceIIRShould: public ::testing::Test {
 protected:
    MSDR1StepInvarianceIIRShould():
      logger_("MSDR1StepInvarianceIIRShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~MSDR1StepInvarianceIIRShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<MSDR1StepInvarianceIIR> msd_;
};

TEST_F(MSDR1StepInvarianceIIRShould, failIfParametersAreZero) {
    double Tstep = 0.01;
    double m = 0;
    double d = 0;
    double k = 0;
    double force = 1;
    msd_.reset(new MSDR1StepInvarianceIIR(Tstep));
    EXPECT_THROW(msd_->calculate(force, m, d, k), std::invalid_argument);
}

TEST_F(MSDR1StepInvarianceIIRShould, failIfAnyParameterIsNegative) {
    double Tstep = 0.01;
    double m = 1;
    double d = 1;
    double k = -1;
    double force = 1;
    msd_.reset(new MSDR1StepInvarianceIIR(Tstep));
    EXPECT_THROW(msd_->calculate(force, m, d, k), std::invalid_argument);

    m = -0.5;
    d = 1;
    k = 1;
    EXPECT_THROW(msd_->calculate(force, m, d, k), std::invalid_argument);

    m = 1;
    d = -2;
    k = 2;
    EXPECT_THROW(msd_->calculate(force, m, d, k), std::invalid_argument);
}

TEST_F(MSDR1StepInvarianceIIRShould, failIfTimeStepIsZeroOrNegative) {
    double Tstep = 0;
    EXPECT_THROW(msd_.reset(new MSDR1StepInvarianceIIR(Tstep)), std::invalid_argument);

    Tstep = -0.5;
    EXPECT_THROW(msd_.reset(new MSDR1StepInvarianceIIR(Tstep)), std::invalid_argument);
}

TEST_F(MSDR1StepInvarianceIIRShould, normalWorking) {
    double Tstep = 0.01;
    double m = 0.5;
    double d = 2;
    double k = 1;
    double force = 0.1;
    double margin = 0.000001;

    msd_.reset(new MSDR1StepInvarianceIIR(Tstep));
    SignalR1 signal = msd_->calculate(force, m, d, k);
    EXPECT_NEAR(signal.position, 0.05000030604, margin);
    double pos = signal.position;
    EXPECT_NEAR(signal.velocity, pos/Tstep, margin);
    double vel = signal.velocity;
    EXPECT_NEAR(signal.acceleration, vel/Tstep, margin);

    // Change of parameters
    m = 1;
    d = 1.5;
    k = 0.5;
    force = 0.12;

    signal = msd_->calculate(force, m, d, k);
    EXPECT_NEAR(signal.position, 0.05062632556, margin);
    EXPECT_NEAR(signal.velocity, (signal.position-pos)/Tstep, margin);
    EXPECT_NEAR(signal.acceleration, (signal.velocity-vel)/Tstep, margin);
}
