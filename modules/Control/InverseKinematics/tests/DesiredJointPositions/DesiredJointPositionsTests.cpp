/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 * 
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include "InverseKinematics/DesiredJointPositions/DesiredJointPositions.hpp"

using crf::control::inversekinematics::DesiredJointPositions;

class DesiredJointPositionsShould: public ::testing::Test {
 protected:
    DesiredJointPositionsShould(): logger_("DesiredJointPositionsShould"),
        q_(1),
        qAttr_(1),
        transitionTimeRange_(0.0),
        cycleTime_(0.0),
        c_(0.0) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~DesiredJointPositionsShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        transitionTimeRange_ = 2.0;
        cycleTime_ = 0.002;
        c_ = 0.1;
        q_ = crf::utility::types::JointPositions({0.5, 2.5});
        qAttr_ = crf::utility::types::JointPositions({1.3, 1.9});
    }

    crf::utility::logger::EventLogger logger_;
    crf::utility::types::JointPositions q_;
    crf::utility::types::JointPositions qAttr_;
    double transitionTimeRange_;
    double cycleTime_;
    double c_;
    std::unique_ptr<DesiredJointPositions> desiredJointPositions_;
};

TEST_F(DesiredJointPositionsShould, ThrowWrongInputInCTor) {
    double transitionTimeRange = 0.0;
    ASSERT_THROW(desiredJointPositions_.reset(new DesiredJointPositions(transitionTimeRange,
        cycleTime_, c_)), std::invalid_argument);
    transitionTimeRange = -10.0;
    ASSERT_THROW(desiredJointPositions_.reset(new DesiredJointPositions(transitionTimeRange,
        cycleTime_, c_)), std::invalid_argument);

    double cycleTime = 0.0;
    ASSERT_THROW(desiredJointPositions_.reset(new DesiredJointPositions(transitionTimeRange_,
        cycleTime, c_)), std::invalid_argument);
    cycleTime = -0.2;
    ASSERT_THROW(desiredJointPositions_.reset(new DesiredJointPositions(transitionTimeRange_,
        cycleTime, c_)), std::invalid_argument);

    double c = 0.0;
    ASSERT_THROW(desiredJointPositions_.reset(new DesiredJointPositions(transitionTimeRange_,
        cycleTime_, c)), std::invalid_argument);
    c = -10.2;
    ASSERT_THROW(desiredJointPositions_.reset(new DesiredJointPositions(transitionTimeRange_,
        cycleTime_, c)), std::invalid_argument);
}

TEST_F(DesiredJointPositionsShould, ThrowWrongInputsInGetGradient) {
    ASSERT_NO_THROW(desiredJointPositions_.reset(new DesiredJointPositions(transitionTimeRange_,
        cycleTime_, c_)));

    crf::utility::types::JointPositions q({2.0, 0.0});
    crf::utility::types::JointPositions qAttr({1.0, 0.5, 0.8});
    ASSERT_THROW(desiredJointPositions_->getGradient(q, qAttr), std::invalid_argument);
}

TEST_F(DesiredJointPositionsShould, returnFalseTryingToEnable) {
    ASSERT_NO_THROW(desiredJointPositions_.reset(new DesiredJointPositions(transitionTimeRange_,
        cycleTime_, c_)));
    ASSERT_TRUE(desiredJointPositions_->enable(true));
    ASSERT_FALSE(desiredJointPositions_->enable(false));
    ASSERT_FALSE(desiredJointPositions_->enable(true));
}

TEST_F(DesiredJointPositionsShould, returnTrueEnablingAndDisablingAndGetEnable) {
    ASSERT_NO_THROW(desiredJointPositions_.reset(new DesiredJointPositions(transitionTimeRange_,
        cycleTime_, c_)));

    ASSERT_FALSE(desiredJointPositions_->enable());
    double rangeTrajectory = 5.0;
    unsigned int timeNumber = (rangeTrajectory / cycleTime_) + 1;
    for (unsigned int i = 0; i < timeNumber; i++) {
        double time = i * cycleTime_;
        if (time == 0.0) {
            ASSERT_TRUE(desiredJointPositions_->enable(true));
            ASSERT_TRUE(desiredJointPositions_->enable());
        }
        if (time == transitionTimeRange_ + cycleTime_) {
            ASSERT_TRUE(desiredJointPositions_->enable(false));
            ASSERT_FALSE(desiredJointPositions_->enable());
        }
        if (time == 2 * transitionTimeRange_ + cycleTime_) {
            ASSERT_TRUE(desiredJointPositions_->enable(true));
            ASSERT_TRUE(desiredJointPositions_->enable());
        }
        Eigen::MatrixXd gradient = desiredJointPositions_->getGradient(q_, qAttr_);
    }
}

TEST_F(DesiredJointPositionsShould, returnNaNInGetGradientIfQAttractorIsNaN) {
    ASSERT_NO_THROW(desiredJointPositions_.reset(new DesiredJointPositions(transitionTimeRange_,
        cycleTime_, c_)));

    crf::utility::types::JointPositions qAttr({std::nan(""), 1.9});
    Eigen::MatrixXd gradient = desiredJointPositions_->getGradient(q_, qAttr);
    ASSERT_EQ(gradient.rows(), qAttr.size());
    ASSERT_EQ(gradient.cols(), 1);
    ASSERT_TRUE(std::isnan(gradient(0, 0)));
    ASSERT_DOUBLE_EQ(gradient(1, 0), 0.0);
}

TEST_F(DesiredJointPositionsShould, returnCorrectResultsInGetGradientAndCorrectBehaviorOfGoToNextIteration) {  // NOLINT
    unsigned int numberOfJoints = 2;
    double cycleTime = 0.1;

    ASSERT_NO_THROW(desiredJointPositions_.reset(new DesiredJointPositions(transitionTimeRange_,
        cycleTime, c_)));

    ASSERT_EQ(desiredJointPositions_->getTransitionFactor(), 0.0);

    double rangeTrajectory = 5.0;
    unsigned int timeNumber = (rangeTrajectory / cycleTime) + 1;
    for (unsigned int i = 0; i < timeNumber; i++) {
        double time = i * cycleTime;
        if ((time == 0.0) || (time == 2 * transitionTimeRange_ + 2 * cycleTime)) {
            // 1st & 5th -> Turn ON
            ASSERT_TRUE(desiredJointPositions_->enable(true));
            Eigen::MatrixXd gradient = desiredJointPositions_->getGradient(q_, qAttr_);
            ASSERT_EQ(gradient.rows(), numberOfJoints);
            ASSERT_EQ(gradient.cols(), 1);
            ASSERT_DOUBLE_EQ(gradient(0, 0), 0.0);
            ASSERT_DOUBLE_EQ(gradient(1, 0), 0.0);
            desiredJointPositions_->goToNextIteration(false);
            ASSERT_EQ(desiredJointPositions_->getTransitionFactor(), 0.0);
            desiredJointPositions_->goToNextIteration(true);
        } else if ((time == transitionTimeRange_ / 2.0)  ||
            (time == transitionTimeRange_ + cycleTime + (transitionTimeRange_ / 2.0))) {
            // 2nd -> Middle of the Transition from OFF to ON
            // 4th -> Middle of the Transition from ON to OFF
            Eigen::MatrixXd gradient = desiredJointPositions_->getGradient(q_, qAttr_);
            ASSERT_EQ(gradient.rows(), numberOfJoints);
            ASSERT_EQ(gradient.cols(), 1);
            ASSERT_DOUBLE_EQ(gradient(0, 0), 0.04);
            ASSERT_DOUBLE_EQ(gradient(1, 0), -0.03);
            desiredJointPositions_->goToNextIteration(false);
            ASSERT_NEAR(desiredJointPositions_->getTransitionFactor(), 0.5, 1E-15);
            desiredJointPositions_->goToNextIteration(true);
        } else if (time == transitionTimeRange_ + cycleTime) {
            // 3rd -> Turn OFF
            ASSERT_TRUE(desiredJointPositions_->enable(false));
            Eigen::MatrixXd gradient = desiredJointPositions_->getGradient(q_, qAttr_);
            ASSERT_EQ(gradient.rows(), numberOfJoints);
            ASSERT_EQ(gradient.cols(), 1);
            ASSERT_DOUBLE_EQ(gradient(0, 0), 0.08);
            ASSERT_DOUBLE_EQ(gradient(1, 0), -0.06);
            desiredJointPositions_->goToNextIteration(false);
            ASSERT_EQ(desiredJointPositions_->getTransitionFactor(), 1.0);
            desiredJointPositions_->goToNextIteration(true);
        } else {
            // Rest of iterations
            Eigen::MatrixXd gradient = desiredJointPositions_->getGradient(q_, qAttr_);
        }
    }
}

TEST_F(DesiredJointPositionsShould, returnFalseIfArgumentIsWrongInSetParameterFunction) {
    ASSERT_NO_THROW(desiredJointPositions_.reset(new DesiredJointPositions(transitionTimeRange_,
        cycleTime_, c_)));
    ASSERT_FALSE(desiredJointPositions_->setParam("c", "-3.2"));
    ASSERT_FALSE(desiredJointPositions_->setParam("c", "0.0"));
    ASSERT_FALSE(desiredJointPositions_->setParam("pc", "2.0"));
}

TEST_F(DesiredJointPositionsShould, returnCorrectResultsInGetGradientAfterSuccessfullySettingNewParameters) {  // NOLINT
    unsigned int numberOfJoints = 2;
    double cycleTime = 0.1;

    ASSERT_NO_THROW(desiredJointPositions_.reset(new DesiredJointPositions(transitionTimeRange_,
        cycleTime, c_)));

    double rangeTrajectory = 8.0;
    unsigned int timeNumber = (rangeTrajectory / cycleTime) + 1;
    for (unsigned int i = 0; i < timeNumber; i++) {
        double time = i * cycleTime;
        if (time == 0.0) {
            // 1st -> Turn ON
            ASSERT_TRUE(desiredJointPositions_->enable(true));
            ASSERT_FALSE(desiredJointPositions_->setParam("c", "5.2"));
            Eigen::MatrixXd gradient = desiredJointPositions_->getGradient(q_, qAttr_);
            ASSERT_EQ(gradient.rows(), numberOfJoints);
            ASSERT_EQ(gradient.cols(), 1);
            ASSERT_DOUBLE_EQ(gradient(0, 0), 0.0);
            ASSERT_DOUBLE_EQ(gradient(1, 0), 0.0);
            ASSERT_FALSE(desiredJointPositions_->setParam("c", "5.2"));
        } else if (time == transitionTimeRange_ + cycleTime) {
            // 2nd -> Turn OFF
            ASSERT_FALSE(desiredJointPositions_->setParam("c", "5.2"));
            ASSERT_TRUE(desiredJointPositions_->enable(false));
            ASSERT_FALSE(desiredJointPositions_->setParam("c", "5.2"));
            Eigen::MatrixXd gradient = desiredJointPositions_->getGradient(q_, qAttr_);
            ASSERT_EQ(gradient.rows(), numberOfJoints);
            ASSERT_EQ(gradient.cols(), 1);
            ASSERT_DOUBLE_EQ(gradient(0, 0), 0.08);
            ASSERT_DOUBLE_EQ(gradient(1, 0), -0.06);
            ASSERT_FALSE(desiredJointPositions_->setParam("c", "5.2"));
        } else if (time == 2 * transitionTimeRange_ + 2 * cycleTime) {
            // 3rd -> Change parameter & Turn ON
            ASSERT_TRUE(desiredJointPositions_->setParam("c", "5.2"));
            ASSERT_TRUE(desiredJointPositions_->enable(true));
            Eigen::MatrixXd gradient = desiredJointPositions_->getGradient(q_, qAttr_);
            ASSERT_EQ(gradient.rows(), numberOfJoints);
            ASSERT_EQ(gradient.cols(), 1);
            ASSERT_DOUBLE_EQ(gradient(0, 0), 0.0);
            ASSERT_DOUBLE_EQ(gradient(1, 0), 0.0);
        } else if (time == 3 * transitionTimeRange_ + 2 * cycleTime) {
            // 4th -> After finishing the transition
            Eigen::MatrixXd gradient = desiredJointPositions_->getGradient(q_, qAttr_);
            ASSERT_EQ(gradient.rows(), numberOfJoints);
            ASSERT_EQ(gradient.cols(), 1);
            ASSERT_DOUBLE_EQ(gradient(0, 0), 4.16);
            ASSERT_DOUBLE_EQ(gradient(1, 0), -3.12);
        } else {
            // Rest of iterations
            Eigen::MatrixXd gradient = desiredJointPositions_->getGradient(q_, qAttr_);
        }
    }
}

TEST_F(DesiredJointPositionsShould, functionGetTimeDerivativeStillNotImplemented) {
    ASSERT_NO_THROW(desiredJointPositions_.reset(new DesiredJointPositions(transitionTimeRange_,
        cycleTime_, c_)));

    ASSERT_EQ(desiredJointPositions_->getTimeDerivative(
        q_, crf::utility::types::JointVelocities(1), qAttr_).size(), 0);
}
