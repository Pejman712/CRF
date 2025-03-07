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
#include "MassSpringDamper/MSDRn/MSDRnDecoupled.hpp"
#include "MassSpringDamper/MSDR1/MSDR1ImpulseInvarianceIIR.hpp"

using crf::math::massspringdamper::MSDRnDecoupled;
using crf::math::massspringdamper::MSDR1ImpulseInvarianceIIR;
using crf::math::massspringdamper::SignalRn;

class MSDRnDecoupledShould: public ::testing::Test {
 protected:
    MSDRnDecoupledShould():
      logger_("MSDRnDecoupledShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~MSDRnDecoupledShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<MSDRnDecoupled<MSDR1ImpulseInvarianceIIR>> msd_;
};

TEST_F(MSDRnDecoupledShould, failIfparametersAreZero) {
    const uint8_t dim = 3;
    float Tstep = 0.01;
    Eigen::MatrixXd m = Eigen::Matrix<double, dim, dim>::Zero();
    Eigen::MatrixXd d = Eigen::Matrix<double, dim, dim>::Zero();
    Eigen::MatrixXd k = Eigen::Matrix<double, dim, dim>::Zero();
    Eigen::MatrixXd force = Eigen::Matrix<double, dim, dim>::Identity();
    msd_.reset(new MSDRnDecoupled<MSDR1ImpulseInvarianceIIR>(dim, Tstep));
    EXPECT_THROW(msd_->calculate(force, m, d, k), std::invalid_argument);
}

TEST_F(MSDRnDecoupledShould, failIfAnyParameterIsNegative) {
    const uint8_t dim = 3;
    float Tstep = 0.01;

    Eigen::MatrixXd m = Eigen::DiagonalMatrix<double, dim>(-1, 0.9, 1.1);
    Eigen::MatrixXd d = Eigen::DiagonalMatrix<double, dim>(1, 0.9, 1.1);
    Eigen::MatrixXd k = Eigen::DiagonalMatrix<double, dim>(1, 0.9, 1.1);
    Eigen::MatrixXd force = Eigen::Matrix<double, dim, dim>::Identity();
    msd_.reset(new MSDRnDecoupled<MSDR1ImpulseInvarianceIIR>(dim, Tstep));
    EXPECT_THROW(msd_->calculate(force, m, d, k), std::invalid_argument);

    m = Eigen::DiagonalMatrix<double, dim>(1, 0.9, 1.1);
    d = Eigen::DiagonalMatrix<double, dim>(1, 0.9, -1.1);
    k = Eigen::DiagonalMatrix<double, dim>(1, 0.9, 1.1);
    EXPECT_THROW(msd_->calculate(force, m, d, k), std::invalid_argument);

    m = Eigen::DiagonalMatrix<double, dim>(1, 0.9, 1.1);
    d = Eigen::DiagonalMatrix<double, dim>(1, 0.9, 1.1);
    k = Eigen::DiagonalMatrix<double, dim>(1, -0.9, 1.1);
    EXPECT_THROW(msd_->calculate(force, m, d, k), std::invalid_argument);
}

TEST_F(MSDRnDecoupledShould, failIfTimeStepIsZeroOrNegative) {
    const uint8_t dim = 3;
    float Tstep = 0;
    EXPECT_THROW(
        msd_.reset(new MSDRnDecoupled<MSDR1ImpulseInvarianceIIR>(dim, Tstep)),
        std::invalid_argument);

    Tstep = -0.5;
    EXPECT_THROW(
        msd_.reset(new MSDRnDecoupled<MSDR1ImpulseInvarianceIIR>(dim, Tstep)),
        std::invalid_argument);
}

TEST_F(MSDRnDecoupledShould, normalWorking3DInput) {
    const uint8_t dim = 3;
    float Tstep = 0.01000;
    float margin = 0.000001;
    Eigen::MatrixXd m = Eigen::DiagonalMatrix<double, dim>(0.5000, 0.4000, 0.6000);
    Eigen::MatrixXd d = Eigen::DiagonalMatrix<double, dim>(2.000, 2.1000, 1.9000);
    Eigen::MatrixXd k = Eigen::DiagonalMatrix<double, dim>(1.000, 0.8000, 1.2000);
    Eigen::MatrixXd force = Eigen::Matrix<double, dim, dim>::Identity()*0.1000;

    msd_.reset(new MSDRnDecoupled<MSDR1ImpulseInvarianceIIR>(dim, Tstep));
    SignalRn signal = msd_->calculate(force, m, d, k);
    std::vector<double> pos = signal.position;
    EXPECT_NEAR(pos[0], 0.04999925274, margin);
    EXPECT_NEAR(pos[1], 0.04999927837, margin);
    EXPECT_NEAR(pos[2], 0.04999911969, margin);
    EXPECT_NEAR(signal.velocity[0], pos[0]/Tstep, margin);
    EXPECT_NEAR(signal.velocity[1], pos[1]/Tstep, margin);
    EXPECT_NEAR(signal.velocity[2], pos[2]/Tstep, margin);
    std::vector<double> vel = signal.velocity;
    EXPECT_NEAR(signal.acceleration[0], vel[0]/Tstep, margin);
    EXPECT_NEAR(signal.acceleration[1], vel[1]/Tstep, margin);
    EXPECT_NEAR(signal.acceleration[2], vel[2]/Tstep, margin);

    // Change of parameters
    m = Eigen::DiagonalMatrix<double, dim>(1.000, 0.9000, 1.1000);
    d = Eigen::DiagonalMatrix<double, dim>(1.5000, 1.7000, 1.45000);
    k = Eigen::DiagonalMatrix<double, dim>(0.5000, 0.55000, 0.45000);
    force = Eigen::DiagonalMatrix<double, dim>(0.12000, 0.11000, 0.13000);

    signal = msd_->calculate(force, m, d, k);
    EXPECT_NEAR(signal.position[0], 0.05062653150, margin);
    EXPECT_NEAR(signal.position[1], 0.05084273778, margin);
    EXPECT_NEAR(signal.position[2], 0.05046297077, margin);
    EXPECT_NEAR(signal.velocity[0], (signal.position[0] - pos[0])/Tstep, margin);
    EXPECT_NEAR(signal.velocity[1], (signal.position[1] - pos[1])/Tstep, margin);
    EXPECT_NEAR(signal.velocity[2], (signal.position[2] - pos[2])/Tstep, margin);
    EXPECT_NEAR(signal.acceleration[0], (signal.velocity[0]-vel[0])/Tstep, margin);
    EXPECT_NEAR(signal.acceleration[1], (signal.velocity[1]-vel[1])/Tstep, margin);
    EXPECT_NEAR(signal.acceleration[2], (signal.velocity[2]-vel[2])/Tstep, margin);
}
