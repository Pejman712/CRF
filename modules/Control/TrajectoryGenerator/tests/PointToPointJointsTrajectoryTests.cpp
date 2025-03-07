/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include <vector>

#include "TrajectoryGenerator/PointToPointJointsTrajectory/PointToPointJointsTrajectory.hpp"

using crf::control::trajectorygenerator::PointToPointJointsTrajectory;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

class PointToPointJointsTrajectoryShould: public ::testing::Test {
 protected:
    PointToPointJointsTrajectoryShould(): logger_("PointToPointJointsTrajectoryShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~PointToPointJointsTrajectoryShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<PointToPointJointsTrajectory> sut_;
};

TEST_F(PointToPointJointsTrajectoryShould, WorkWithOneTrajectory) {
    std::vector<JointPositions> points = {
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);  // First Joint
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 0, 0.000001);  // Second Joint
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 0, 0.000001);  // Third Joint

    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[0], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[1], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[2], 0.8125, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(2.625).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.625).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.625).positions.value()[2], 0, 0.000001);
}

TEST_F(PointToPointJointsTrajectoryShould, CorrectlyAppendOnTheBack) {
    std::vector<JointPositions> points = {
        JointPositions({0, 0, 0}),
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    sut_->append(points);
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 0, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[0], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[1], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[2], 0.8125, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(2.25).positions.value()[0], 0.1875, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.25).positions.value()[1], 0.1875, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.25).positions.value()[2], 0.1875, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(3.25).positions.value()[0], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(3.25).positions.value()[1], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(3.25).positions.value()[2], 0.8125, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(4.625).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(4.625).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(4.625).positions.value()[2], 0, 0.000001);
}

TEST_F(PointToPointJointsTrajectoryShould, CorrectlyAppendWhenLastPointWasTheEnd) {
    std::vector<JointPositions> points = {
        JointPositions({0, 0, 0}),
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    sut_->append(points);


    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 0, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[0], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[1], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[2], 0.8125, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(2.7).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.7).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.7).positions.value()[2], 0, 0.000001);

    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(3.95).positions.value()[0], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(3.95).positions.value()[1], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(3.95).positions.value()[2], 0.8125, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(5.35).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(5.35).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(5.35).positions.value()[2], 0, 0.000001);
}

TEST_F(PointToPointJointsTrajectoryShould, ResetAndWorkLikeNewAgain) {
    std::vector<JointPositions> points = {
        JointPositions({0, 0, 0}),
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));
    {
        sut_->setInitialPosition(JointPositions({0, 0, 0}));
        sut_->append(points);

        ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 0, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 0, 0.000001);

        ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[0], 0.8125, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[1], 0.8125, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[2], 0.8125, 0.000001);

        ASSERT_NEAR(sut_->getTrajectoryPoint(2.625).positions.value()[0], 0, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(2.625).positions.value()[1], 0, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(2.625).positions.value()[2], 0, 0.000001);
    }

    sut_->reset();

    {
        sut_->setInitialPosition(JointPositions({0, 0, 0}));
        sut_->append(points);

        ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 0, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 0, 0.000001);

        ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[0], 0.8125, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[1], 0.8125, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[2], 0.8125, 0.000001);

        ASSERT_NEAR(sut_->getTrajectoryPoint(2.625).positions.value()[0], 0, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(2.625).positions.value()[1], 0, 0.000001);
        ASSERT_NEAR(sut_->getTrajectoryPoint(2.625).positions.value()[2], 0, 0.000001);
    }
}

TEST_F(PointToPointJointsTrajectoryShould, WaitUntilTheEndOfThePreviousTrajIfAppendedTooLate) {
    std::vector<JointPositions> points = {
        JointPositions({0, 0, 0}),
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 0, 0.000001);

    sut_->getTrajectoryPoint(2);

    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(2.75).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.75).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.75).positions.value()[2], 0, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(3.875).positions.value()[0], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(3.875).positions.value()[1], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(3.875).positions.value()[2], 0.8125, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(5.25).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(5.25).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(5.25).positions.value()[2], 0, 0.000001);
}

TEST_F(PointToPointJointsTrajectoryShould, CorrectlyComputeDifferentTrajectoriesForEachJoint) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({1, 12, 6}),
        JointPositions({0, 13, 7})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 13, 5}));
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.0001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 13, 0.0001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 5, 0.0001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[0], 0.8125, 0.0001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[1], 12.1875, 0.0001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[2], 5.875, 0.0001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(2.6).positions.value()[0], 0, 0.0001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.6).positions.value()[1], 13, 0.0001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.6).positions.value()[2], 7, 0.0001);
}

TEST_F(PointToPointJointsTrajectoryShould, TakeTheMostRestrictiveDimensionAndAdaptTheOthers) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({0, 13, 5}),
        JointPositions({1, 12, 8}),
        JointPositions({2, 11, 12})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 13, 5}));
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.0001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 13, 0.0001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 5, 0.0001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(3.3).positions.value()[0], 1, 0.01);
    ASSERT_NEAR(sut_->getTrajectoryPoint(3.3).positions.value()[1], 12, 0.01);
    ASSERT_NEAR(sut_->getTrajectoryPoint(3.3).positions.value()[2], 8, 0.1);

    ASSERT_NEAR(sut_->getTrajectoryPoint(7.35).positions.value()[0], 2, 0.0001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(7.35).positions.value()[1], 11, 0.0001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(7.4).positions.value()[2], 12, 0.1);
}

TEST_F(PointToPointJointsTrajectoryShould, AdjustAllTimeWithDifferentPointsAndMaxVelsAndAccs) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({0, 13, 5}),
        JointPositions({3, 12, 7}),
        JointPositions({6, 11, 9})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 13, 5}));
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 13, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 5, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(3.25).positions.value()[0], 2.874, 0.001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(3.25).positions.value()[1], 12, 0.1);
    ASSERT_NEAR(sut_->getTrajectoryPoint(3.25).positions.value()[2], 7, 0.1);

    ASSERT_NEAR(sut_->getTrajectoryPoint(6.5).positions.value()[0], 6, 0.01);
    ASSERT_NEAR(sut_->getTrajectoryPoint(6.5).positions.value()[1], 11, 0.01);
    ASSERT_NEAR(sut_->getTrajectoryPoint(6.5).positions.value()[2], 9, 0.01);
}


TEST_F(PointToPointJointsTrajectoryShould, ThrowExceptionIfTrajectoryPOintIsRequestedBeforeAppending) {  // NOLINT
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));

    ASSERT_THROW(sut_->getTrajectoryPoint(1), std::runtime_error);
}

TEST_F(PointToPointJointsTrajectoryShould, ThowAnExceptionIfTimeGoesBackwards) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({0, 0, 0}),
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 13, 5}));
    sut_->append(points);

    sut_->getTrajectoryPoint(1);

    ASSERT_THROW(sut_->getTrajectoryPoint(0.5), std::runtime_error);
}


TEST_F(PointToPointJointsTrajectoryShould, CorrectlyClearMemoryThatIsNotBeingUsed) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({0, 0, 0}),
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));
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

TEST_F(PointToPointJointsTrajectoryShould, AppendCorrectlyAfterSomeTime) {
    std::vector<JointPositions> points = {
        JointPositions({0, 0, 0}),
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({3, 3, 3});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).positions.value()[2], 0, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[0], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[1], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(1.25).positions.value()[2], 0.8125, 0.000001);

    sut_->getTrajectoryPoint(20);

    sut_->append(points);

    ASSERT_NEAR(sut_->getTrajectoryPoint(20).positions.value()[0], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(20).positions.value()[1], 0, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(20).positions.value()[2], 0, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(21.25).positions.value()[0], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(21.25).positions.value()[1], 0.8125, 0.000001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(21.25).positions.value()[2], 0.8125, 0.000001);

    ASSERT_NEAR(sut_->getTrajectoryPoint(22.6).positions.value()[0], 0, 0.0001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(22.6).positions.value()[1], 0, 0.0001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(22.6).positions.value()[2], 0, 0.0001);
}

TEST_F(PointToPointJointsTrajectoryShould, WorkWhenAccelerationIsNotEnough) {
    std::vector<JointPositions> points = {
        JointPositions({0, 0, 0}),
        JointPositions({1, 1, 1}),
        JointPositions({0, 0, 0})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({1, 1, 1});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    ASSERT_NO_THROW(sut_->append(points));

    ASSERT_NEAR(sut_->getTrajectoryPoint(2.25).positions.value()[0], 0.54380710, 0.0001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.25).positions.value()[1], 0.54380710, 0.0001);
    ASSERT_NEAR(sut_->getTrajectoryPoint(2.25).positions.value()[2], 0.54380710, 0.0001);
}

TEST_F(PointToPointJointsTrajectoryShould, IsTrajectoryRunningShouldReturnTrueAndFalseAccordingly) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({0, 0, 0}),
        JointPositions({1, 1, 1}),
        JointPositions({2, 2, 2})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({1, 1, 1});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));
    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    ASSERT_FALSE(sut_->isTrajectoryRunning());
    sut_->append(points);

    sut_->getTrajectoryPoint(0);

    sut_->getTrajectoryPoint(1);
    ASSERT_TRUE(sut_->isTrajectoryRunning());

    sut_->getTrajectoryPoint(4);
    ASSERT_FALSE(sut_->isTrajectoryRunning());

    sut_->getTrajectoryPoint(10);
    ASSERT_FALSE(sut_->isTrajectoryRunning());

    sut_->append(points);

    sut_->getTrajectoryPoint(11);
    ASSERT_TRUE(sut_->isTrajectoryRunning());

    sut_->getTrajectoryPoint(20);
    ASSERT_FALSE(sut_->isTrajectoryRunning());
}

TEST_F(PointToPointJointsTrajectoryShould, throwExceptionIfYouTryToAppendBeforeSetInitialPos) {
    // Three dimensions with four points
    std::vector<JointPositions> points = {
        JointPositions({0, 0, 0}),
        JointPositions({1, 1, 1}),
        JointPositions({2, 2, 2})};
    JointVelocities maxVel({1, 1, 1});
    JointAccelerations maxAcc({1, 1, 1});

    sut_.reset(new PointToPointJointsTrajectory(maxVel, maxAcc));
    ASSERT_THROW(sut_->append(points), std::runtime_error);

    sut_->setInitialPosition(JointPositions({0, 0, 0}));
    ASSERT_NO_THROW(sut_->append(points));
}
