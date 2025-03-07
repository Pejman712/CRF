/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 * 
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include "InverseKinematics/CollisionAvoidanceSphere/CollisionAvoidanceSphere.hpp"

using crf::control::inversekinematics::CollisionAvoidanceSphere;

class CollisionAvoidanceSphereShould: public ::testing::Test {
 protected:
    CollisionAvoidanceSphereShould(): logger_("CollisionAvoidanceSphereShould"),
        q_({0.5, 2.5, 0.5, 2.5, 0.5, 2.5}),
        transitionTimeRange_(2.0),
        cycleTime_(0.002),
        curveType_(1),  // Exponential
        c_(0.1),
        p_(1.0),
        center_(0.2, 0.0, -1.0),
        radius_(0.4),
        robot_(2) {  // UR10e - 6DOF robot
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~CollisionAvoidanceSphereShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    crf::utility::types::JointPositions q_;
    double transitionTimeRange_;
    double cycleTime_;
    int curveType_;
    double c_;
    double p_;
    Eigen::Vector3d center_;
    double radius_;
    int robot_;
    std::unique_ptr<CollisionAvoidanceSphere> collisionAvoidanceSphere_;
};

TEST_F(CollisionAvoidanceSphereShould, ThrowWrongInputsInCTor) {
    double transitionTimeRange = 0.0;
    ASSERT_THROW(collisionAvoidanceSphere_.reset(new CollisionAvoidanceSphere(transitionTimeRange,
        cycleTime_, curveType_, c_, p_, center_, radius_, robot_)), std::invalid_argument);

    double cycleTime = -0.2;
    ASSERT_THROW(collisionAvoidanceSphere_.reset(new CollisionAvoidanceSphere(transitionTimeRange_,
        cycleTime, curveType_, c_, p_, center_, radius_, robot_)), std::invalid_argument);

    double c = 0.0;
    ASSERT_THROW(collisionAvoidanceSphere_.reset(new CollisionAvoidanceSphere(transitionTimeRange_,
        cycleTime_, curveType_, c, p_, center_, radius_, robot_)), std::invalid_argument);

    double radius = -10.2;
    ASSERT_THROW(collisionAvoidanceSphere_.reset(new CollisionAvoidanceSphere(transitionTimeRange_,
        cycleTime_, curveType_, c_, p_, center_, radius, robot_)), std::invalid_argument);
}

TEST_F(CollisionAvoidanceSphereShould, returnFalseTryingToEnable) {
    ASSERT_NO_THROW(collisionAvoidanceSphere_.reset(new CollisionAvoidanceSphere(
        transitionTimeRange_, cycleTime_, curveType_, c_, p_, center_, radius_, robot_)));
    ASSERT_TRUE(collisionAvoidanceSphere_->enable(true));
    ASSERT_FALSE(collisionAvoidanceSphere_->enable(false));
    ASSERT_FALSE(collisionAvoidanceSphere_->enable(true));
}

TEST_F(CollisionAvoidanceSphereShould, returnTrueEnablingAndDisablingAndGetEnable) {
    ASSERT_NO_THROW(collisionAvoidanceSphere_.reset(new CollisionAvoidanceSphere(
        transitionTimeRange_, cycleTime_, curveType_, c_, p_, center_, radius_, robot_)));

    ASSERT_FALSE(collisionAvoidanceSphere_->enable());
    double rangeTrajectory = 5.0;
    unsigned int timeNumber = (rangeTrajectory / cycleTime_) + 1;
    for (unsigned int i = 0; i < timeNumber; i++) {
        double time = i * cycleTime_;
        if (time == 0.0) {
            ASSERT_TRUE(collisionAvoidanceSphere_->enable(true));
            ASSERT_TRUE(collisionAvoidanceSphere_->enable());
        }
        if (time == transitionTimeRange_ + cycleTime_) {
            ASSERT_TRUE(collisionAvoidanceSphere_->enable(false));
            ASSERT_FALSE(collisionAvoidanceSphere_->enable());
        }
        if (time == 2 * transitionTimeRange_ + cycleTime_) {
            ASSERT_TRUE(collisionAvoidanceSphere_->enable(true));
            ASSERT_TRUE(collisionAvoidanceSphere_->enable());
        }
        // The function getGradient() needs to be called to pass succesfully this test, since the
        // increase of the counter, which establishes the enable/disable transition time where the
        // state can't be changed, is inside this function.
        Eigen::MatrixXd gradient = collisionAvoidanceSphere_->getGradient(q_);
    }
}

TEST_F(CollisionAvoidanceSphereShould, returnFalseIfArgumentIsWrongInSetParameterFunction) {
    ASSERT_NO_THROW(collisionAvoidanceSphere_.reset(new CollisionAvoidanceSphere(
        transitionTimeRange_, cycleTime_, curveType_, c_, p_, center_, radius_, robot_)));
    ASSERT_FALSE(collisionAvoidanceSphere_->setParam("c", "-3.2"));
    ASSERT_FALSE(collisionAvoidanceSphere_->setParam("pc", "2.0"));
}

TEST_F(CollisionAvoidanceSphereShould, functionGetTimeDerivativeStillNotImplemented) {
    ASSERT_NO_THROW(collisionAvoidanceSphere_.reset(new CollisionAvoidanceSphere(
        transitionTimeRange_, cycleTime_, curveType_, c_, p_, center_, radius_, robot_)));

    ASSERT_EQ(collisionAvoidanceSphere_->getTimeDerivative(
        q_, crf::utility::types::JointVelocities(1)).size(), 0);
}
