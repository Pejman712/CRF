/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 * 
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include "InverseKinematics/ResultsIK.hpp"

class ResultsIKShould: public ::testing::Test {
 protected:
    ResultsIKShould(): logger_("ResultsIKShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~ResultsIKShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
};

TEST_F(ResultsIKShould, AssignmentAndGettingTest) {
    crf::control::inversekinematics::ResultsIK extendedResult;

    crf::utility::types::TaskPose z(
        {1.0, 1.0, 1.0}, crf::math::rotation::CardanXYZ({1.0, 1.0, 1.0}));
    crf::utility::types::TaskVelocity zd({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::utility::types::TaskAcceleration zdd({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::utility::types::JointPositions q({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::utility::types::JointVelocities qd({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::utility::types::JointAccelerations qdd({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::utility::types::JointPositions q1({1.0});
    crf::utility::types::JointVelocities qd1({1.0});
    crf::utility::types::JointAccelerations qdd1({1.0});
    std::vector<double> zError({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::control::inversekinematics::ResultFlags flag(
        crf::control::inversekinematics::ResultFlags::success);
    double kinematicManipulability(0.02);
    Eigen::MatrixXd penaltyGradients(q.size(), 1);
    penaltyGradients << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

    ASSERT_FALSE(areAlmostEqual(extendedResult.zDesired(), z, 10e-15));
    ASSERT_FALSE(areAlmostEqual(extendedResult.zdDesired(), zd, 10e-15));
    ASSERT_FALSE(areAlmostEqual(extendedResult.zddDesired(), zdd, 10e-15));
    ASSERT_THROW(areAlmostEqual(extendedResult.qResult(), q, 10e-15), std::invalid_argument);
    ASSERT_THROW(areAlmostEqual(extendedResult.qdResult(), qd, 10e-15), std::invalid_argument);
    ASSERT_THROW(areAlmostEqual(extendedResult.qddResult(), qdd, 10e-15), std::invalid_argument);
    ASSERT_FALSE(areAlmostEqual(extendedResult.qResult(), q1, 10e-15));
    ASSERT_FALSE(areAlmostEqual(extendedResult.qdResult(), qd1, 10e-15));
    ASSERT_FALSE(areAlmostEqual(extendedResult.qddResult(), qdd1, 10e-15));
    ASSERT_NE(extendedResult.zError(), zError);
    ASSERT_NE(extendedResult.flag(), flag);
    ASSERT_NE(extendedResult.kinematicManipulability(), kinematicManipulability);
    // It is not possible to compare Eigen::Matrix of different sizes
    ASSERT_NE(extendedResult.penaltyGradients().size(), penaltyGradients.size());

    extendedResult.zDesired(z);
    extendedResult.zdDesired(zd);
    extendedResult.zddDesired(zdd);
    extendedResult.qResult(q);
    extendedResult.qdResult(qd);
    extendedResult.qddResult(qdd);
    extendedResult.zError(zError);
    extendedResult.flag(flag);
    extendedResult.kinematicManipulability(kinematicManipulability);
    extendedResult.penaltyGradients(penaltyGradients);

    ASSERT_TRUE(areAlmostEqual(extendedResult.zDesired(), z, 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.zdDesired(), zd, 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.zddDesired(), zdd, 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qResult(), q, 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qdResult(), qd, 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qddResult(), qdd, 10e-15));
    ASSERT_EQ(extendedResult.zError(), zError);
    ASSERT_EQ(extendedResult.flag(), flag);
    ASSERT_EQ(extendedResult.kinematicManipulability(), kinematicManipulability);
    ASSERT_EQ(extendedResult.penaltyGradients(), penaltyGradients);
}

TEST_F(ResultsIKShould, CopyCtorTest) {
    crf::control::inversekinematics::ResultsIK extendedResult;

    crf::control::inversekinematics::ResultsIK extendedResultCopy1(extendedResult);

    ASSERT_TRUE(areAlmostEqual(extendedResult.zDesired(), extendedResultCopy1.zDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(
        extendedResult.zdDesired(), extendedResultCopy1.zdDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(
        extendedResult.zddDesired(), extendedResultCopy1.zddDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qResult(), extendedResultCopy1.qResult(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qdResult(), extendedResultCopy1.qdResult(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(
        extendedResult.qddResult(), extendedResultCopy1.qddResult(), 10e-15));
    ASSERT_EQ(extendedResult.zError(), extendedResultCopy1.zError());
    ASSERT_EQ(extendedResult.flag(), extendedResultCopy1.flag());
    ASSERT_EQ(extendedResult.kinematicManipulability(),
        extendedResultCopy1.kinematicManipulability());
    ASSERT_EQ(extendedResult.penaltyGradients(), extendedResultCopy1.penaltyGradients());

    crf::utility::types::TaskPose z(
        {1.0, 1.0, 1.0}, crf::math::rotation::CardanXYZ({1.0, 1.0, 1.0}));
    crf::utility::types::TaskVelocity zd({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::utility::types::TaskAcceleration zdd({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::utility::types::JointPositions q({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::utility::types::JointVelocities qd({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::utility::types::JointAccelerations qdd({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    std::vector<double> zError({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::control::inversekinematics::ResultFlags flag(
        crf::control::inversekinematics::ResultFlags::success);
    double kinematicManipulability(0.02);
    Eigen::MatrixXd penaltyGradients(q.size(), 1);
    penaltyGradients << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

    extendedResult.zDesired(z);
    extendedResult.zdDesired(zd);
    extendedResult.zddDesired(zdd);
    extendedResult.qResult(q);
    extendedResult.qdResult(qd);
    extendedResult.qddResult(qdd);
    extendedResult.zError(zError);
    extendedResult.flag(flag);
    extendedResult.kinematicManipulability(kinematicManipulability);
    extendedResult.penaltyGradients(penaltyGradients);

    crf::control::inversekinematics::ResultsIK extendedResultCopy2(extendedResult);

    ASSERT_TRUE(areAlmostEqual(extendedResult.zDesired(), extendedResultCopy2.zDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(
        extendedResult.zdDesired(), extendedResultCopy2.zdDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(
        extendedResult.zddDesired(), extendedResultCopy2.zddDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qResult(), extendedResultCopy2.qResult(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qdResult(), extendedResultCopy2.qdResult(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(
        extendedResult.qddResult(), extendedResultCopy2.qddResult(), 10e-15));
    ASSERT_EQ(extendedResult.zError(), extendedResultCopy2.zError());
    ASSERT_EQ(extendedResult.flag(), extendedResultCopy2.flag());
    ASSERT_EQ(extendedResult.kinematicManipulability(),
        extendedResultCopy2.kinematicManipulability());
    ASSERT_EQ(extendedResult.penaltyGradients(), extendedResultCopy2.penaltyGradients());

    ASSERT_FALSE(areAlmostEqual(
        extendedResultCopy1.zDesired(), extendedResultCopy2.zDesired(), 10e-15));
    ASSERT_FALSE(areAlmostEqual(
        extendedResultCopy1.zdDesired(), extendedResultCopy2.zdDesired(), 10e-15));
    ASSERT_FALSE(areAlmostEqual(
        extendedResultCopy1.zddDesired(), extendedResultCopy2.zddDesired(), 10e-15));
    ASSERT_THROW(areAlmostEqual(
        extendedResultCopy1.qResult(), extendedResultCopy2.qResult(), 10e-15),
        std::invalid_argument);
    ASSERT_THROW(areAlmostEqual(
        extendedResultCopy1.qdResult(), extendedResultCopy2.qdResult(), 10e-15),
        std::invalid_argument);
    ASSERT_THROW(areAlmostEqual(
        extendedResultCopy1.qddResult(), extendedResultCopy2.qddResult(), 10e-15),
        std::invalid_argument);
    ASSERT_NE(extendedResultCopy1.zError(), extendedResultCopy2.zError());
    ASSERT_NE(extendedResultCopy1.flag(), extendedResultCopy2.flag());
    ASSERT_NE(extendedResultCopy1.kinematicManipulability(),
        extendedResultCopy2.kinematicManipulability());
    ASSERT_NE(extendedResultCopy1.penaltyGradients().size(),
        extendedResultCopy2.penaltyGradients().size());
}

TEST_F(ResultsIKShould, EqualOperatorTest) {
    crf::control::inversekinematics::ResultsIK extendedResult;

    crf::control::inversekinematics::ResultsIK extendedResult2;

    ASSERT_TRUE(areAlmostEqual(extendedResult.zDesired(), extendedResult2.zDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.zdDesired(), extendedResult2.zdDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.zddDesired(), extendedResult2.zddDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qResult(), extendedResult2.qResult(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qdResult(), extendedResult2.qdResult(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qddResult(), extendedResult2.qddResult(), 10e-15));
    ASSERT_EQ(extendedResult.zError(), extendedResult2.zError());
    ASSERT_EQ(extendedResult.flag(), extendedResult2.flag());
    ASSERT_EQ(extendedResult.kinematicManipulability(), extendedResult2.kinematicManipulability());
    ASSERT_EQ(extendedResult.penaltyGradients(), extendedResult2.penaltyGradients());

    extendedResult2 = extendedResult;

    ASSERT_TRUE(areAlmostEqual(extendedResult.zDesired(), extendedResult2.zDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.zdDesired(), extendedResult2.zdDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.zddDesired(), extendedResult2.zddDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qResult(), extendedResult2.qResult(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qdResult(), extendedResult2.qdResult(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qddResult(), extendedResult2.qddResult(), 10e-15));
    ASSERT_EQ(extendedResult.zError(), extendedResult2.zError());
    ASSERT_EQ(extendedResult.flag(), extendedResult2.flag());
    ASSERT_EQ(extendedResult.kinematicManipulability(), extendedResult2.kinematicManipulability());
    ASSERT_EQ(extendedResult.penaltyGradients(), extendedResult2.penaltyGradients());

    crf::utility::types::TaskPose z(
        {1.0, 1.0, 1.0}, crf::math::rotation::CardanXYZ({1.0, 1.0, 1.0}));
    crf::utility::types::TaskVelocity zd({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::utility::types::TaskAcceleration zdd({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::utility::types::JointPositions q({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::utility::types::JointVelocities qd({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::utility::types::JointAccelerations qdd({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    std::vector<double> zError({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::control::inversekinematics::ResultFlags flag(
        crf::control::inversekinematics::ResultFlags::success);
    double kinematicManipulability(0.02);
    Eigen::MatrixXd penaltyGradients(q.size(), 1);
    penaltyGradients << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

    extendedResult.zDesired(z);
    extendedResult.zdDesired(zd);
    extendedResult.zddDesired(zdd);
    extendedResult.qResult(q);
    extendedResult.qdResult(qd);
    extendedResult.qddResult(qdd);
    extendedResult.zError(zError);
    extendedResult.flag(flag);
    extendedResult.kinematicManipulability(kinematicManipulability);
    extendedResult.penaltyGradients(penaltyGradients);

    ASSERT_FALSE(areAlmostEqual(extendedResult.zDesired(), extendedResult2.zDesired(), 10e-15));
    ASSERT_FALSE(areAlmostEqual(extendedResult.zdDesired(), extendedResult2.zdDesired(), 10e-15));
    ASSERT_FALSE(areAlmostEqual(extendedResult.zddDesired(), extendedResult2.zddDesired(), 10e-15));
    ASSERT_THROW(areAlmostEqual(extendedResult.qResult(), extendedResult2.qResult(), 10e-15),
        std::invalid_argument);
    ASSERT_THROW(areAlmostEqual(extendedResult.qdResult(), extendedResult2.qdResult(), 10e-15),
        std::invalid_argument);
    ASSERT_THROW(areAlmostEqual(extendedResult.qddResult(), extendedResult2.qddResult(), 10e-15),
        std::invalid_argument);
    ASSERT_NE(extendedResult.zError(), extendedResult2.zError());
    ASSERT_NE(extendedResult.flag(), extendedResult2.flag());
    ASSERT_NE(extendedResult.kinematicManipulability(), extendedResult2.kinematicManipulability());
    ASSERT_NE(extendedResult.penaltyGradients().size(), extendedResult2.penaltyGradients().size());

    extendedResult2 = extendedResult;

    ASSERT_TRUE(areAlmostEqual(extendedResult.zDesired(), extendedResult2.zDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.zdDesired(), extendedResult2.zdDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.zddDesired(), extendedResult2.zddDesired(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qResult(), extendedResult2.qResult(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qdResult(), extendedResult2.qdResult(), 10e-15));
    ASSERT_TRUE(areAlmostEqual(extendedResult.qddResult(), extendedResult2.qddResult(), 10e-15));
    ASSERT_EQ(extendedResult.zError(), extendedResult2.zError());
    ASSERT_EQ(extendedResult.flag(), extendedResult2.flag());
    ASSERT_EQ(extendedResult.kinematicManipulability(), extendedResult2.kinematicManipulability());
    ASSERT_EQ(extendedResult.penaltyGradients(), extendedResult2.penaltyGradients());
}
