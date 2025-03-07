/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 * 
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include "InverseKinematics/JointLimits/JointLimits.hpp"

using crf::control::inversekinematics::JointLimits;

class JointLimitsShould: public ::testing::Test {
 protected:
    JointLimitsShould(): logger_("JointLimitsShould"),
        minLimit_(1),
        maxLimit_(1),
        q_(1),
        transitionTimeRange_(0.0),
        cycleTime_(0.0),
        c_(0.0),
        p_(0.0) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~JointLimitsShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        transitionTimeRange_ = 2.0;
        cycleTime_ = 0.002;
        c_ = 0.1;
        p_ = 10.2;
        minLimit_ = crf::utility::types::JointPositions({0.0, 0.0});
        maxLimit_ = crf::utility::types::JointPositions({2.0, 6.0});
        q_ = crf::utility::types::JointPositions({0.5, 2.5});
    }

    crf::utility::logger::EventLogger logger_;
    crf::utility::types::JointPositions minLimit_;
    crf::utility::types::JointPositions maxLimit_;
    crf::utility::types::JointPositions q_;
    double transitionTimeRange_;
    double cycleTime_;
    double c_;
    double p_;
    std::unique_ptr<JointLimits> jointLimits_;
};

TEST_F(JointLimitsShould, ThrowWrongInputsInCTor) {
    double transitionTimeRange = 0.0;
    ASSERT_THROW(jointLimits_.reset(new JointLimits(transitionTimeRange, cycleTime_, c_, p_,
        minLimit_, maxLimit_)), std::invalid_argument);

    double cycleTime = -0.2;
    ASSERT_THROW(jointLimits_.reset(new JointLimits(transitionTimeRange_, cycleTime, c_, p_,
        minLimit_, maxLimit_)), std::invalid_argument);

    double c = 0.0;
    ASSERT_THROW(jointLimits_.reset(new JointLimits(transitionTimeRange_, cycleTime_, c, p_,
        minLimit_, maxLimit_)), std::invalid_argument);

    double p = -10.2;
    ASSERT_THROW(jointLimits_.reset(new JointLimits(transitionTimeRange_, cycleTime_, c_, p,
        minLimit_, maxLimit_)), std::invalid_argument);

    crf::utility::types::JointPositions minLimit({10.0, 0.0, 0.0});
    ASSERT_THROW(jointLimits_.reset(new JointLimits(transitionTimeRange_, cycleTime_, c_, p_,
        minLimit, maxLimit_)), std::invalid_argument);

    crf::utility::types::JointPositions minLimit2({0.0, 0.0});
    crf::utility::types::JointPositions maxLimit2({std::nan(""), 6.0});
    ASSERT_THROW(jointLimits_.reset(new JointLimits(transitionTimeRange_, cycleTime_, c_, p_,
        minLimit2, maxLimit2)), std::invalid_argument);
}

TEST_F(JointLimitsShould, ThrowWrongInputsInGetGradient) {
    ASSERT_NO_THROW(jointLimits_.reset(new JointLimits(transitionTimeRange_, cycleTime_, c_, p_,
        minLimit_, maxLimit_)));

    crf::utility::types::JointPositions q({2.0, 0.0, 0.0});
    ASSERT_THROW(jointLimits_->getGradient(q), std::invalid_argument);
}

TEST_F(JointLimitsShould, returnFalseTryingToEnable) {
    ASSERT_NO_THROW(jointLimits_.reset(new JointLimits(transitionTimeRange_, cycleTime_, c_, p_,
        minLimit_, maxLimit_)));
    ASSERT_TRUE(jointLimits_->enable(true));
    ASSERT_FALSE(jointLimits_->enable(false));
    ASSERT_FALSE(jointLimits_->enable(true));
}

TEST_F(JointLimitsShould, returnTrueEnablingAndDisablingAndGetEnable) {
    ASSERT_NO_THROW(jointLimits_.reset(new JointLimits(transitionTimeRange_, cycleTime_, c_, p_,
        minLimit_, maxLimit_)));

    ASSERT_FALSE(jointLimits_->enable());
    double rangeTrajectory = 5.0;
    unsigned int timeNumber = (rangeTrajectory / cycleTime_) + 1;
    for (unsigned int i = 0; i < timeNumber; i++) {
        double time = i * cycleTime_;
        if (time == 0.0) {
            ASSERT_TRUE(jointLimits_->enable(true));
            ASSERT_TRUE(jointLimits_->enable());
        }
        if (time == transitionTimeRange_ + cycleTime_) {
            ASSERT_TRUE(jointLimits_->enable(false));
            ASSERT_FALSE(jointLimits_->enable());
        }
        if (time == 2 * transitionTimeRange_ + cycleTime_) {
            ASSERT_TRUE(jointLimits_->enable(true));
            ASSERT_TRUE(jointLimits_->enable());
        }
        Eigen::MatrixXd gradient = jointLimits_->getGradient(q_);
    }
}

TEST_F(JointLimitsShould, returnNaNInGetGradientIfLimitsAreNaN) {
    crf::utility::types::JointPositions minLimit({std::nan(""), 0.0});
    crf::utility::types::JointPositions maxLimit({std::nan(""), 6.0});
    ASSERT_NO_THROW(jointLimits_.reset(new JointLimits(transitionTimeRange_, cycleTime_, c_, p_,
        minLimit, maxLimit)));

    Eigen::MatrixXd gradient = jointLimits_->getGradient(q_);
    ASSERT_EQ(gradient.rows(), q_.size());
    ASSERT_EQ(gradient.cols(), 1);
    ASSERT_TRUE(std::isnan(gradient(0, 0)));
    ASSERT_DOUBLE_EQ(gradient(1, 0), 0.0);
}

TEST_F(JointLimitsShould, returnCorrectResultsInGetGradientAndCorrectBehaviorOfGoToNextIteration) {  // NOLINT
    unsigned int numberOfJoints = 2;
    double cycleTime = 0.1;

    ASSERT_NO_THROW(jointLimits_.reset(new JointLimits(transitionTimeRange_, cycleTime, c_, p_,
        minLimit_, maxLimit_)));

    ASSERT_EQ(jointLimits_->getTransitionFactor(), 0.0);

    double rangeTrajectory = 5.0;
    unsigned int timeNumber = (rangeTrajectory / cycleTime) + 1;
    for (unsigned int i = 0; i < timeNumber; i++) {
        double time = i * cycleTime;
        if ((time == 0.0) || (time == 2 * transitionTimeRange_ + 2 * cycleTime)) {
            // 1st & 5th -> Turn ON
            ASSERT_TRUE(jointLimits_->enable(true));
            Eigen::MatrixXd gradient = jointLimits_->getGradient(q_);
            ASSERT_EQ(gradient.rows(), numberOfJoints);
            ASSERT_EQ(gradient.cols(), 1);
            ASSERT_DOUBLE_EQ(gradient(0, 0), 0.0);
            ASSERT_DOUBLE_EQ(gradient(1, 0), 0.0);
            jointLimits_->goToNextIteration(false);
            ASSERT_EQ(jointLimits_->getTransitionFactor(), 0.0);
            jointLimits_->goToNextIteration(true);
        } else if ((time == transitionTimeRange_ / 2.0)  ||
            (time == transitionTimeRange_ + cycleTime + (transitionTimeRange_ / 2.0))) {
            // 2nd -> Middle of the Transition from OFF to ON
            // 4th -> Middle of the Transition from ON to OFF
            Eigen::MatrixXd gradient = jointLimits_->getGradient(q_);
            ASSERT_EQ(gradient.rows(), numberOfJoints);
            ASSERT_EQ(gradient.cols(), 1);
            ASSERT_DOUBLE_EQ(gradient(0, 0), 0.046165938518584686);
            ASSERT_DOUBLE_EQ(gradient(1, 0), 0.037797473609872616);
            jointLimits_->goToNextIteration(false);
            ASSERT_NEAR(jointLimits_->getTransitionFactor(), 0.5, 1E-15);
            jointLimits_->goToNextIteration(true);
        } else if (time == transitionTimeRange_ + cycleTime) {
            // 3rd -> Turn OFF
            ASSERT_TRUE(jointLimits_->enable(false));
            Eigen::MatrixXd gradient = jointLimits_->getGradient(q_);
            ASSERT_EQ(gradient.rows(), numberOfJoints);
            ASSERT_EQ(gradient.cols(), 1);
            ASSERT_DOUBLE_EQ(gradient(0, 0), 0.092331877037169385);
            ASSERT_DOUBLE_EQ(gradient(1, 0), 0.0755949472197452456);
            jointLimits_->goToNextIteration(false);
            ASSERT_EQ(jointLimits_->getTransitionFactor(), 1.0);
            jointLimits_->goToNextIteration(true);
        } else {
            // Rest of iterations
            Eigen::MatrixXd gradient = jointLimits_->getGradient(q_);
        }
    }
}

TEST_F(JointLimitsShould, returnFalseIfArgumentIsWrongInSetParameterFunction) {
    ASSERT_NO_THROW(jointLimits_.reset(new JointLimits(transitionTimeRange_, cycleTime_, c_, p_,
        minLimit_, maxLimit_)));
    ASSERT_FALSE(jointLimits_->setParam("c", "-3.2"));
    ASSERT_FALSE(jointLimits_->setParam("p", "0.0"));
    ASSERT_FALSE(jointLimits_->setParam("pc", "2.0"));
}

TEST_F(JointLimitsShould, returnCorrectResultsInGetGradientAfterSuccessfullySettingNewParameters) {  // NOLINT
    unsigned int numberOfJoints = 2;
    double cycleTime = 0.1;

    ASSERT_NO_THROW(jointLimits_.reset(new JointLimits(transitionTimeRange_, cycleTime, c_, p_,
        minLimit_, maxLimit_)));

    double rangeTrajectory = 8.0;
    unsigned int timeNumber = (rangeTrajectory / cycleTime) + 1;
    for (unsigned int i = 0; i < timeNumber; i++) {
        double time = i * cycleTime;
        if (time == 0.0) {
            // 1st -> Turn ON
            ASSERT_TRUE(jointLimits_->enable(true));
            ASSERT_FALSE(jointLimits_->setParam("c", "5.2"));
            ASSERT_FALSE(jointLimits_->setParam("p", "30.5"));
            Eigen::MatrixXd gradient = jointLimits_->getGradient(q_);
            ASSERT_EQ(gradient.rows(), numberOfJoints);
            ASSERT_EQ(gradient.cols(), 1);
            ASSERT_DOUBLE_EQ(gradient(0, 0), 0.0);
            ASSERT_DOUBLE_EQ(gradient(1, 0), 0.0);
            ASSERT_FALSE(jointLimits_->setParam("c", "5.2"));
            ASSERT_FALSE(jointLimits_->setParam("p", "30.5"));
        } else if (time == transitionTimeRange_ + cycleTime) {
            // 2nd -> Turn OFF
            ASSERT_FALSE(jointLimits_->setParam("c", "5.2"));
            ASSERT_FALSE(jointLimits_->setParam("p", "30.5"));
            ASSERT_TRUE(jointLimits_->enable(false));
            ASSERT_FALSE(jointLimits_->setParam("c", "5.2"));
            ASSERT_FALSE(jointLimits_->setParam("p", "30.5"));
            Eigen::MatrixXd gradient = jointLimits_->getGradient(q_);
            ASSERT_EQ(gradient.rows(), numberOfJoints);
            ASSERT_EQ(gradient.cols(), 1);
            ASSERT_DOUBLE_EQ(gradient(0, 0), 0.092331877037169385);
            ASSERT_DOUBLE_EQ(gradient(1, 0), 0.075594947219745245);
            ASSERT_FALSE(jointLimits_->setParam("c", "5.2"));
            ASSERT_FALSE(jointLimits_->setParam("p", "30.5"));
        } else if (time == 2 * transitionTimeRange_ + 2 * cycleTime) {
            // 3rd -> Change parameter & Turn ON
            ASSERT_TRUE(jointLimits_->setParam("c", "5.2"));
            ASSERT_TRUE(jointLimits_->setParam("p", "30.5"));
            ASSERT_TRUE(jointLimits_->enable(true));
            Eigen::MatrixXd gradient = jointLimits_->getGradient(q_);
            ASSERT_EQ(gradient.rows(), numberOfJoints);
            ASSERT_EQ(gradient.cols(), 1);
            ASSERT_DOUBLE_EQ(gradient(0, 0), 0.0);
            ASSERT_DOUBLE_EQ(gradient(1, 0), 0.0);
        } else if (time == 3 * transitionTimeRange_ + 2 * cycleTime) {
            // 4th -> After finishing the transition
            Eigen::MatrixXd gradient = jointLimits_->getGradient(q_);
            ASSERT_EQ(gradient.rows(), numberOfJoints);
            ASSERT_EQ(gradient.cols(), 1);
            ASSERT_DOUBLE_EQ(gradient(0, 0), 11.714805537127159);
            ASSERT_DOUBLE_EQ(gradient(1, 0), 0.00035651062045537511);
        } else {
            // Rest of iterations
            Eigen::MatrixXd gradient = jointLimits_->getGradient(q_);
        }
    }
}

TEST_F(JointLimitsShould, functionGetTimeDerivativeStillNotImplemented) {
    ASSERT_NO_THROW(jointLimits_.reset(new JointLimits(transitionTimeRange_, cycleTime_, c_, p_,
        minLimit_, maxLimit_)));

    ASSERT_EQ(jointLimits_->getTimeDerivative(
        q_, crf::utility::types::JointVelocities(1)).size(), 0);
}
