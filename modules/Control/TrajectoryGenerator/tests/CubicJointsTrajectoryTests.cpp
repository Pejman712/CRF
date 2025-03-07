/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include <vector>

#include "TrajectoryGenerator/CubicJointsTrajectory/CubicJointsTrajectory.hpp"

using crf::control::trajectorygenerator::CubicJointsTrajectory;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

class CubicJointsTrajectoryShould: public ::testing::Test {
 protected:
    CubicJointsTrajectoryShould(): logger_("CubicJointsTrajectoryShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~CubicJointsTrajectoryShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<CubicJointsTrajectory> sut_;
};

TEST_F(CubicJointsTrajectoryShould, DeleteConsecutiveDuplicatesAppendedInPath) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({1, 1, 1}),
        JointPositions({1, 1, 1}),
        JointPositions({2, 2, 2})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    sut_->append(points);
}

TEST_F(CubicJointsTrajectoryShould, AppendCorrectlyOnce) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0}),
        JointPositions({2, 2, 2})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);  // First Joint
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 0, 0.000001);  // Second Joint
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 0, 0.000001);  // Third Joint

    ASSERT_NEAR(sut_->getTrajectoryPoint(9).velocities.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(9).velocities.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(9).velocities.value()[2], 0, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(9).positions.value()[0], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(9).positions.value()[1], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(9).positions.value()[2], 2, 0.000001);
}

TEST_F(CubicJointsTrajectoryShould, AppendCorrectlyTwiceWhenLastPointWasTheEnd) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0}),
        JointPositions({2, 2, 2})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 0, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(9).velocities.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(9).velocities.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(9).velocities.value()[2], 0, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(9).positions.value()[0], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(9).positions.value()[1], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(9).positions.value()[2], 2, 0.000001);

    sut_->append(points);  // Append the same points again to the back in point 4

    ASSERT_NEAR(sut_->getTrajectoryPoint(9).velocities.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(9).velocities.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(9).velocities.value()[2], 0, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(9).positions.value()[0], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(9).positions.value()[1], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(9).positions.value()[2], 2, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(22).positions.value()[0], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(22).positions.value()[1], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(22).positions.value()[2], 2, 0.000001);
}

TEST_F(CubicJointsTrajectoryShould, AppendCorrectlyTwiceWhenLastPointWasAfterTheEnd) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0}),
        JointPositions({2, 2, 2})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({3, 3, 3}));
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 3, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 3, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 3, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(11).velocities.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(11).velocities.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(11).velocities.value()[2], 0, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(11).positions.value()[0], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(11).positions.value()[1], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(11).positions.value()[2], 2, 0.000001);

    // Should be static
    ASSERT_NEAR(sut_->getTrajectoryPoint(13).positions.value()[0], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(13).positions.value()[1], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(13).positions.value()[2], 2, 0.000001);

    sut_->append(points);  // Append the same points again to the back in time 10

    ASSERT_NEAR(sut_->getTrajectoryPoint(13).positions.value()[0], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(13).positions.value()[1], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(13).positions.value()[2], 2, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(26).positions.value()[0], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(26).positions.value()[1], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(26).positions.value()[2], 2, 0.000001);
}

TEST_F(CubicJointsTrajectoryShould, AppendCorrectlyTwiceWhenTrajectoryHasNotFinished) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0}),
        JointPositions({2, 2, 2})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({3, 3, 3}));
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 3, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 3, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 3, 0.000001);

    sut_->getTrajectoryPoint(2);

    // We are evaluating the function in time 2, now we append.
    // This append should modify the spline between the last two points (0, 2)
    // Append the same points again to the back in time t=8 that we have not
    // reached yet (last evaluated point is t=2)
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(6.5).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(6.5).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(6.5).positions.value()[2], 0, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(23.5).positions.value()[0], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(23.5).positions.value()[1], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(23.5).positions.value()[2], 2, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(25).positions.value()[0], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(25).positions.value()[1], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(25).positions.value()[2], 2, 0.000001);
}

TEST_F(CubicJointsTrajectoryShould, AppendCorrectlyTwiceAndStopInBetweenIfEvaluatedAtLastPoint) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0}),
        JointPositions({2, 2, 2})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({3, 3, 3}));
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 3, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 3, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 3, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(11).positions.value()[0], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(11).positions.value()[1], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(11).positions.value()[2], 2, 0.000001);

    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(11).velocities.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(11).velocities.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(11).velocities.value()[2], 0, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(24).positions.value()[0], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(24).positions.value()[1], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(24).positions.value()[2], 2, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(26).positions.value()[0], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(26).positions.value()[1], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(26).positions.value()[2], 2, 0.000001);
}

TEST_F(CubicJointsTrajectoryShould, ThrowExceptionIfTrajectoryPOintIsRequestedBeforeAppending) {
    // Three dimensions with four points
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));

    ASSERT_THROW(sut_->getTrajectoryPoint(1), std::runtime_error);
}

TEST_F(CubicJointsTrajectoryShould, ThowAnExceptionIfTimeGoesBackwards) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    sut_->append(points);

    sut_->getTrajectoryPoint(1);

    ASSERT_THROW(sut_->getTrajectoryPoint(0.5), std::runtime_error);
}


TEST_F(CubicJointsTrajectoryShould, CorrectlyClearMemoryThatIsNotBeingUsed) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    sut_->append(points);
    sut_->append(points);
    sut_->append(points);

    // Points: 0, 1, 0, 1, 0, 1, 0
    // Ranges: 0, 2, 4, 6, 8, 10, 12

    sut_->getTrajectoryPoint(10);

    sut_->clearMemory();

    sut_->getTrajectoryPoint(12);

    sut_->clearMemory();
}

TEST_F(CubicJointsTrajectoryShould, ResetAndWorkLikeNewAgain) {
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});
    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    std::vector<JointPositions> points = {
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0}),
        JointPositions({2, 2, 2})};
    {
        sut_->setInitialPosition(JointPositions({3, 3, 3}));
        sut_->append(points);

        ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 3, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 3, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 3, 0.000001);

        ASSERT_NEAR(sut_->getTrajectoryPoint(11).velocities.value()[0], 0, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(11).velocities.value()[1], 0, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(11).velocities.value()[2], 0, 0.000001);

        ASSERT_NEAR(sut_->getTrajectoryPoint(11).positions.value()[0], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(11).positions.value()[1], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(11).positions.value()[2], 2, 0.000001);

        // Should be static
        ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[0], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[1], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[2], 2, 0.000001);

        sut_->append(points);  // Append the same points again to the back in time 10

        ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[0], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[1], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[2], 2, 0.000001);

        ASSERT_NEAR(sut_->getTrajectoryPoint(28).positions.value()[0], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(28).positions.value()[1], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(28).positions.value()[2], 2, 0.000001);
    }

    sut_->reset();

    {
        sut_->setInitialPosition(JointPositions({3, 3, 3}));
        sut_->append(points);

        ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 3, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 3, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 3, 0.000001);

        ASSERT_NEAR(sut_->getTrajectoryPoint(11).velocities.value()[0], 0, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(11).velocities.value()[1], 0, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(11).velocities.value()[2], 0, 0.000001);

        ASSERT_NEAR(sut_->getTrajectoryPoint(11).positions.value()[0], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(11).positions.value()[1], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(11).positions.value()[2], 2, 0.000001);

        // Should be static
        ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[0], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[1], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[2], 2, 0.000001);

        sut_->append(points);  // Append the same points again to the back in time 10

        ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[0], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[1], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[2], 2, 0.000001);

        ASSERT_NEAR(sut_->getTrajectoryPoint(28).positions.value()[0], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(28).positions.value()[1], 2, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(28).positions.value()[2], 2, 0.000001);
    }
}

TEST_F(CubicJointsTrajectoryShould, CorrectlyDoTrajectoriesWithTwoPoints) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({0, 0, 0}),
        JointPositions({1, 1, 1})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    sut_->append(points);
    sut_->append(points);
    sut_->append(points);

    // Points: 0, 1, 0, 1, 0, 1
    // Ranges: 0, 2, 4, 6, 8, 10

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 0, 0.000001);

    // Not clear anything
    sut_->clearMemory();

    ASSERT_NEAR(sut_->getTrajectoryPoint(13).positions.value()[0], 1, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(13).positions.value()[1], 1, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(13).positions.value()[2], 1, 0.000001);

    // Clear two previous trajectories
    sut_->clearMemory();
}

TEST_F(CubicJointsTrajectoryShould, CorrectlyDoTrajectoriesWithThreeRedundantPoints) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    sut_->append(points);
    sut_->append(points);
    sut_->append(points);

    // Points: 0, 1, 0, 1, 0, 1, 0
    // Ranges: 0, 2, 4, 6, 8, 10, 12

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 0, 0.000001);

    sut_->clearMemory();

    ASSERT_NEAR(sut_->getTrajectoryPoint(5).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(5).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(5).positions.value()[2], 0, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(10).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(10).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(10).positions.value()[2], 0, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(15).positions.value()[2], 0, 0.000001);

    sut_->clearMemory();
}

TEST_F(CubicJointsTrajectoryShould, CorrectlyComputeDifferentTrajectoriesForEachJoint) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({1, 12, 6}),
        JointPositions({0, 13, 7})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 13, 5}));
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 13, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 5, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(2.5).positions.value()[0], 1, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.5).positions.value()[1], 12, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.5).positions.value()[2], 6, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(5).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(5).positions.value()[1], 13, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(5).positions.value()[2], 7, 0.000001);
}


TEST_F(CubicJointsTrajectoryShould, TakeTheMostRestrictiveDimensionAndAdaptTheOthers) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({1, 12, 8}),
        JointPositions({2, 11, 12})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 13, 5}));
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 13, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 5, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(13).positions.value()[0], 2, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(13).positions.value()[1], 11, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(13).positions.value()[2], 12, 0.000001);
}

TEST_F(CubicJointsTrajectoryShould, AdjustAllTimeWithDifferentPointsAndMaxVelsAndAccs) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({3, 12, 7}),
        JointPositions({6, 11, 9})};
    JointVelocities maxVel({3, 1, 2});
    JointAccelerations maxAcc({3, 1, 2});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 13, 5}));
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 13, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 5, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(2.5).positions.value()[0], 3, 0.001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.5).positions.value()[1], 12, 0.001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.5).positions.value()[2], 7, 0.001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(5).positions.value()[0], 6, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(5).positions.value()[1], 11, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(5).positions.value()[2], 9, 0.000001);
}

TEST_F(CubicJointsTrajectoryShould, IsTrajectoryRunningShouldReturnTrueAndFalseAccordingly) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({1, 1, 1}),
        JointPositions({2, 2, 2})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({1, 1, 1});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    ASSERT_FALSE(sut_->isTrajectoryRunning());
    sut_->append(points);

    sut_->getTrajectoryPoint(0);

    sut_->getTrajectoryPoint(1);
    ASSERT_TRUE(sut_->isTrajectoryRunning());

    sut_->getTrajectoryPoint(6);
    ASSERT_FALSE(sut_->isTrajectoryRunning());

    sut_->getTrajectoryPoint(10);
    ASSERT_FALSE(sut_->isTrajectoryRunning());

    sut_->append(points);

    sut_->getTrajectoryPoint(11);
    ASSERT_TRUE(sut_->isTrajectoryRunning());

    sut_->getTrajectoryPoint(20);
    ASSERT_FALSE(sut_->isTrajectoryRunning());
}

TEST_F(CubicJointsTrajectoryShould, throwExceptionIfYouTryToAppendBeforeSetInitialPosYouDumDum) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({0, 0, 0}),
        JointPositions({1, 1, 1}),
        JointPositions({2, 2, 2})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({1, 1, 1});

    sut_.reset(new CubicJointsTrajectory(maxVel, maxAcc));
    ASSERT_THROW(sut_->append(points), std::runtime_error);

    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    ASSERT_NO_THROW(sut_->append(points));
}
