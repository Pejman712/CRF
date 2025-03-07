/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Chelsea Davidson CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include "TrajectoryGenerator/CubicTaskTrajectory/CubicTaskTrajectory.hpp"

using crf::control::trajectorygenerator::CubicTaskTrajectory;
using crf::math::rotation::areAlmostEqual;

class CubicTaskTrajectoryShould : public ::testing::Test {
 protected:
    CubicTaskTrajectoryShould() :
        logger_("CubicTaskTrajectoryShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~CubicTaskTrajectoryShould() {
        logger_->info(
            "{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<CubicTaskTrajectory> sut_;
    double eps_ = 0.00000000001;  // 1e-11 (defines small difference, epsilon)
};

TEST_F(CubicTaskTrajectoryShould, CorrectlyInformIfTrajectoryIsRunning) {
    // Create a trajectory object
    TaskVelocity maxVel({0.1, 0.1, 0.1, 0.3, 0.3, 0.3});
    TaskAcceleration maxAcc({0.2, 0.2, 0.2, 0.6, 0.6, 0.6});

    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));
    ASSERT_FALSE(sut_->isTrajectoryRunning());

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaternion<double>(1.0, 0, 0, 0));
    sut_->setInitialPose(initialPose);

    ASSERT_FALSE(sut_->isTrajectoryRunning());

    // First append
    std::vector<TaskPose> newPath1 = {
        TaskPose(Eigen::Vector3d(0, 0, 0.5), Eigen::Quaternion<double>(0, 0, 0, 1.0)),
        TaskPose(Eigen::Vector3d(1.0, 0, 0.5), Eigen::Quaternion<double>(1.0, 0, 0, 0))};
    sut_->append(newPath1);

    ASSERT_TRUE(sut_->isTrajectoryRunning());

    // Evaluate outside the range of the trajectory
    TaskSignals evaluationSignal = sut_->getTrajectoryPoint(35.0);
    ASSERT_FALSE(sut_->isTrajectoryRunning());

    // Second append
    std::vector<TaskPose> newPath2 = {
        TaskPose(Eigen::Vector3d(0.5, 0, 0.6), Eigen::Quaternion<double>(1.0, 0, 0, 0)),
        TaskPose(Eigen::Vector3d(0.3, 0, 0.6), Eigen::Quaternion<double>(1.0, 0, 0, 0))};
    sut_->append(newPath2);

    ASSERT_TRUE(sut_->isTrajectoryRunning());

    // Evaluate outside the range of the trajectory
    evaluationSignal = sut_->getTrajectoryPoint(50.0);
    ASSERT_FALSE(sut_->isTrajectoryRunning());
}

TEST_F(CubicTaskTrajectoryShould, CorrectlyPerformAReset) {
    // Create a trajectory object
    TaskVelocity maxVel({0.1, 0.1, 0.1, 0.2, 0.2, 0.2});
    TaskAcceleration maxAcc({0.3, 0.3, 0.3, 0.4, 0.4, 0.4});
    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaternion<double>(1.0, 0, 0, 0));
    sut_->setInitialPose(initialPose);

    // First append
    std::vector<TaskPose> newPath = {
        TaskPose(Eigen::Vector3d(0.0, 0.0, 0.5), Eigen::Quaternion<double>(0.0, 0.0, 0.0, 1.0)),
        TaskPose(Eigen::Vector3d(0.1, 0.0, 0.5), Eigen::Quaternion<double>(0.0, 1.0, 0.0, 0.0))};

    sut_->append(newPath);

    // Expected time instances taken from outputting the time instances used in the
    // CubicTaskTrajectoryGenerator
    std::vector<double> expectedTimeInstances = {24.5619449019234, 49.1238898038469};

    // Store values of the current second last point
    TaskPose pose = sut_->getTrajectoryPoint(expectedTimeInstances[0]).pose.value();
    Eigen::Vector<double, 6> vel =
        sut_->getTrajectoryPoint(expectedTimeInstances[0]).velocity.value().raw();
    Eigen::Vector<double, 6> ac =
        sut_->getTrajectoryPoint(expectedTimeInstances[0]).acceleration.value().raw();

    sut_->reset();

    // Haven't set initial pose yet - appending should throw an error
    ASSERT_ANY_THROW({
        try {
            sut_->append(newPath);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ(
                "Must set an initial pose before defining a trajectory. Please "
                "call setInitialPose() before calling append().", e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    // Haven't set initial pose yet - evaluating should throw an error
    ASSERT_ANY_THROW({
        try {
            sut_->getTrajectoryPoint(0.0);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ(
                "No trajectories have been generated, cannot retrieve trajectory point", e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    // Check values are different after the reset
    ASSERT_NO_THROW(sut_->setInitialPose(initialPose));
    ASSERT_FALSE(areAlmostEqual(
        sut_->getTrajectoryPoint(expectedTimeInstances[0]).pose.value(), pose, eps_));

    ASSERT_FALSE(sut_->
        getTrajectoryPoint(expectedTimeInstances[0]).velocity.value().raw().isApprox(vel, eps_));
    ASSERT_FALSE(sut_->
        getTrajectoryPoint(expectedTimeInstances[0]).acceleration.value().raw().isApprox(ac, eps_));

    // Check it works like new again
    // last evaluated at expectedTimeInstances[0] above and time taken to get from initial pos to
    // newPath[0] is expectedTimeInstances[0] so do expectedTimeInstances[0]*2
    std::vector<double> newTimeInstances = {
        expectedTimeInstances[0] * 2, expectedTimeInstances[0] + expectedTimeInstances[1]};
    sut_->append(newPath);

    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(newTimeInstances[0]).pose.value(), newPath[0], eps_));
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(newTimeInstances[1]).pose.value(), newPath[1], eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(newTimeInstances[1]).velocity.value().raw().isZero(eps_));
}

TEST_F(CubicTaskTrajectoryShould, CorrectlyClearMemoryThatIsNotBeingUsed) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaternion<double>(1.0, 0, 0, 0));
    sut_->setInitialPose(initialPose);

    std::vector<TaskPose> path = {
        TaskPose(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Quaternion<double>(1.0, 0, 0, 0)),
        TaskPose(Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Quaternion<double>(1.0, 0, 0, 0)),
        TaskPose(Eigen::Vector3d(2.0, 2.0, 2.0), Eigen::Quaternion<double>(1.0, 0, 0, 0))};

    // Make 3 trajectories
    sut_->append(path);
    sut_->append(path);
    sut_->append(path);

    // Points:  0, 0, 0                 Ranges: 0
    //          1, 0, 0                         2.5
    // 2nd traj 1, 1, 1                         5
    //          2, 2, 2                         7.5
    //          1, 0, 0                         11.5
    // 3rd traj 1, 1, 1                         14
    //          2, 2, 2                         16.5
    //          1, 0, 0                         20.5
    //          1, 1, 1                         23
    //          2, 2, 2                         25.5

    sut_->getTrajectoryPoint(23);
    sut_->clearMemory();  // Should clear 2 trajectories and leave the one starting at 14
    sut_->getTrajectoryPoint(25.5);
    sut_->clearMemory();  // Shouldn't clear any because there is only one trajectory left

    // Check still works okay
    sut_->append(path);

    // Points:                          Ranges:
    // 1st traj 1, 1, 1                         14
    //          2, 2, 2                         16.5
    //          1, 0, 0                         20.5
    // 2nd traj 1, 1, 1                         23
    //          2, 2, 2                         25.5
    //          1, 0, 0                         29.5
    //          1, 1, 1                         32
    //          2, 2, 2                         34.5
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(25.5).pose.value(), path[2], eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(29.5).pose.value(), path[0], eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(32.0).pose.value(), path[1], eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(34.5).pose.value(), path[2], eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(34.5).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(40.0).pose.value(), path[2], eps_));
}

TEST_F(CubicTaskTrajectoryShould, AppendCorrectlyOnceBeforeEvaluation) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaternion<double>(1.0, 0, 0, 0));
    sut_->setInitialPose(initialPose);

    std::vector<TaskPose> path = {
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(2.0, 0, 2.0), Eigen::Quaternion<double>(0, 0, 1.0, 0)),
        TaskPose(Eigen::Vector3d(3.0, 3.0, 0), Eigen::Quaternion<double>(0, 0, 0, 1.0))};

    std::vector<double> expectedTimes = {8.5, 14.2123889803847, 19.9247779607694};

    sut_->append(path);

    ASSERT_TRUE(sut_->getTrajectoryPoint(0).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(0).pose.value(), initialPose, eps_));

    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[0]).pose.value(), path[0], eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[1]).pose.value(), path[1], eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[2]).pose.value(), path[2], eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTimes[2]).velocity.value().raw().isZero(eps_));

    // Check constant after last point
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(expectedTimes[2] + 5.0).pose.value(), path[2], eps_));
    ASSERT_TRUE(
        sut_->getTrajectoryPoint(expectedTimes[2] + 5.0).velocity.value().raw().isZero(eps_));
}

TEST_F(CubicTaskTrajectoryShould, AppendCorrectlyOnceAfterEvaluation) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaternion<double>(1.0, 0, 0, 0));
    sut_->setInitialPose(initialPose);

    // Evaluate trajectory at start and at 10s
    ASSERT_TRUE(sut_->getTrajectoryPoint(0).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(0).pose.value(), initialPose, eps_));

    ASSERT_TRUE(sut_->getTrajectoryPoint(10.0).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(10.0).pose.value(), initialPose, eps_));

    // Append points after the evaluation - starts trajectory at 10.0s
    std::vector<TaskPose> path = {
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(2.0, 0, 2.0), Eigen::Quaternion<double>(0, 0, 1.0, 0)),
        TaskPose(Eigen::Vector3d(3.0, 3.0, 0), Eigen::Quaternion<double>(0, 0, 0, 1.0))};
    sut_->append(path);

    // Check the start point of this new trajectory didn't change
    ASSERT_TRUE(sut_->getTrajectoryPoint(10.0).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(10.0).pose.value(), initialPose, eps_));

    // Check the points were appended
    std::vector<double> expectedTimes = {18.5, 24.2123889803847, 29.9247779607694};

    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[0]).pose.value(), path[0], eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[1]).pose.value(), path[1], eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[2]).pose.value(), path[2], eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTimes[2]).velocity.value().raw().isZero(eps_));

    // Check constant after last point
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(expectedTimes[2] + 5.0).pose.value(), path[2], eps_));
    ASSERT_TRUE(
        sut_->getTrajectoryPoint(expectedTimes[2] + 5.0).velocity.value().raw().isZero(eps_));
}

TEST_F(CubicTaskTrajectoryShould, AppendCorrectlyTwiceWhenLastPointWasTheEnd) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaternion<double>(1.0, 0, 0, 0));
    sut_->setInitialPose(initialPose);

    std::vector<TaskPose> path = {
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(2.0, 0, 2.0), Eigen::Quaternion<double>(0, 0, 1.0, 0)),
        TaskPose(Eigen::Vector3d(3.0, 3.0, 0), Eigen::Quaternion<double>(0, 0, 0, 1.0))};

    std::vector<double> expectedTimes = {8.5, 14.2123889803847, 19.9247779607694};

    sut_->append(path);
    ASSERT_TRUE(sut_->getTrajectoryPoint(0).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(0).pose.value(), initialPose, eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[0]).pose.value(), path[0], eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[1]).pose.value(), path[1], eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[2]).pose.value(), path[2], eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTimes[2]).velocity.value().raw().isZero(eps_));

    sut_->append(path);  // Append the same points again to the back in point 4
    std::vector<double> newExpectedTimes = {28.4247779607694, 34.1371669411541, 39.8495559215388};

    // Check velocity is 0 at last point in 1st trajectory
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTimes[2]).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[2]).pose.value(), path[2], eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(newExpectedTimes[0]).pose.value(), path[0], eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(newExpectedTimes[1]).pose.value(), path[1], eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(newExpectedTimes[2]).pose.value(), path[2], eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(newExpectedTimes[2]).velocity.value().raw().isZero(eps_));
}

TEST_F(CubicTaskTrajectoryShould, AppendCorrectlyTwiceWhenLastPointWasAfterTheEnd) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaternion<double>(1.0, 0, 0, 0));
    sut_->setInitialPose(initialPose);

    std::vector<TaskPose> path = {
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(2.0, 0, 2.0), Eigen::Quaternion<double>(0, 0, 1.0, 0)),
        TaskPose(Eigen::Vector3d(3.0, 3.0, 0), Eigen::Quaternion<double>(0, 0, 0, 1.0))};

    std::vector<double> expectedTimes = {8.5, 14.2123889803847, 19.9247779607694};

    sut_->append(path);

    ASSERT_TRUE(sut_->getTrajectoryPoint(0).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(0).pose.value(), initialPose, eps_));

    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[2]).pose.value(), path[2], eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTimes[2]).velocity.value().raw().isZero(eps_));

    // Evaluate past the last point Should be static
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[2] + 5).pose.value(), path[2], eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTimes[2] + 5).velocity.value().raw().isZero(eps_));

    // Append the same points again to the back of the time that was just evaluated (last point
    // time instance + 5)
    sut_->append(path);
    std::vector<double> newExpectedTimes = {33.4247779607694, 39.1371669411541, 44.8495559215388};

    // Last evaluated point should still be the same static point with velocity 0
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[2] + 5).pose.value(), path[2], eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTimes[2] + 5).velocity.value().raw().isZero(eps_));

    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(newExpectedTimes[2]).pose.value(), path[2], eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(newExpectedTimes[2]).velocity.value().raw().isZero(eps_));
}

TEST_F(CubicTaskTrajectoryShould, AppendCorrectlyTwiceWhenTrajectoryHasNotFinished) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaternion<double>(1.0, 0, 0, 0));
    sut_->setInitialPose(initialPose);

    std::vector<TaskPose> path = {
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(2.0, 0, 2.0), Eigen::Quaternion<double>(0, 0, 1.0, 0)),
        TaskPose(Eigen::Vector3d(3.0, 3.0, 0), Eigen::Quaternion<double>(0, 0, 0, 1.0))};

    sut_->append(path);
    std::vector<double> expectedTimes = {8.5, 14.2123889803847, 19.9247779607694};

    ASSERT_TRUE(sut_->getTrajectoryPoint(0).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(0).pose.value(), initialPose, eps_));

    // Second last time instance is 14 so we evaluate at a time before that
    sut_->getTrajectoryPoint(10);

    // Last evaluated time was before the second last time instance so this next append will modify
    // the spline between the last two points (14, 19.5).
    sut_->append(path);
    std::vector<double> newExpectedTimes = {28.4247779607694, 34.1371669411541, 39.8495559215388};

    // Check that the previous last two points still go to the correct pose
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[1]).pose.value(), path[1], eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTimes[2]).pose.value(), path[2], eps_));
    // Check the previous last point is no longer zero velocity
    ASSERT_FALSE(sut_->getTrajectoryPoint(expectedTimes[2]).velocity.value().raw().isZero(eps_));

    // Check new last point
    ASSERT_TRUE(sut_->getTrajectoryPoint(newExpectedTimes[2]).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(newExpectedTimes[2]).pose.value(), path[2], eps_));

    // Check stationary beyond new last point
    ASSERT_TRUE(
        sut_->getTrajectoryPoint(newExpectedTimes[2] + 5).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(newExpectedTimes[2] + 5).pose.value(), path[2], eps_));
}

TEST_F(CubicTaskTrajectoryShould, AppendCorrectlyTwiceWhenInitialPoseWasEvaluated) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaternion<double>(1.0, 0, 0, 0));
    sut_->setInitialPose(initialPose);

    // Evaluate the initial pose
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(5.0).pose.value(), initialPose, eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(8.0).pose.value(), initialPose, eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(10.0).pose.value(), initialPose, eps_));

    // 1st append - first trajectory append
    std::vector<TaskPose> point1 = {
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
    };
    ASSERT_NO_THROW(sut_->append(point1));

    // 2nd append - smooth trajectory append since lastEvaluationPoint is second last point
    std::vector<TaskPose> point2 = {
        TaskPose(Eigen::Vector3d(2.0, 0, 2.0), Eigen::Quaternion<double>(0, 0, 1.0, 0)),
    };
    ASSERT_NO_THROW(sut_->append(point2));

    // If range of initial pose (2nd last point) was 0, traj. point at 10s would not be initial pose
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(10.0).pose.value(), initialPose, eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(10.0).velocity.value().raw().isZero(eps_));

    // Check can still evaluate between the last initial pose (10s) and the 1st point (18.5) now
    // that both trajectories are defined as starting at 10
    ASSERT_NO_THROW(sut_->getTrajectoryPoint(15.0).pose.value());
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(18.5).pose.value(), point1[0], eps_));

    // Check can still evaluate between the 1st point (18.5) and the second point (24.21) now
    // that both trajectories are defined as starting at 10
    ASSERT_NO_THROW(sut_->getTrajectoryPoint(20.0).pose.value());
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(24.2123889803847).pose.value(), point2[0], eps_));
}

TEST_F(CubicTaskTrajectoryShould, ThrowExceptionIfTrajectoryPointRequestedBeforeInitialPoseSet) {
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    std::vector<TaskPose> path = {
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(2.0, 0, 2.0), Eigen::Quaternion<double>(0, 0, 1.0, 0)),
        TaskPose(Eigen::Vector3d(3.0, 3.0, 0), Eigen::Quaternion<double>(0, 0, 0, 1.0))};

    ASSERT_ANY_THROW({
        try {
            sut_->append(path);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ(
                "Must set an initial pose before defining a trajectory. Please "
                "call setInitialPose() before calling append().", e.what());
            throw;  // Re-throw to propagate the exception
        }
    });
}

TEST_F(CubicTaskTrajectoryShould, ThrowExceptionIfTimeGoesBackwards) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaternion<double>(1.0, 0, 0, 0));
    sut_->setInitialPose(initialPose);

    sut_->getTrajectoryPoint(1.0);

    ASSERT_ANY_THROW({
        try {
            sut_->getTrajectoryPoint(0.5);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ("Going backwards in a trajectory is not permitted", e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    std::vector<TaskPose> path = {
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(2.0, 0, 2.0), Eigen::Quaternion<double>(0, 0, 1.0, 0)),
        TaskPose(Eigen::Vector3d(3.0, 3.0, 0), Eigen::Quaternion<double>(0, 0, 0, 1.0))};
    sut_->append(path);

    ASSERT_ANY_THROW({
        try {
            sut_->getTrajectoryPoint(0.5);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ("Going backwards in a trajectory is not permitted", e.what());
            throw;  // Re-throw to propagate the exception
        }
    });
}

TEST_F(CubicTaskTrajectoryShould, CorrectlyAppendOnePoint) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(1.0, 0, 0), Eigen::Quaternion<double>(1.0, 0, 0, 0));
    sut_->setInitialPose(initialPose);

    std::vector<TaskPose> point1 = {
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0))};

    // ------- First trajectory append ----------------
    ASSERT_NO_THROW(sut_->append(point1));

    double expectedTime = 8.5;

    // Check points before appended point did not change and that they are constant at initialPose
    ASSERT_TRUE(sut_->getTrajectoryPoint(0).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(0).pose.value(), initialPose, eps_));

    // Check stationary at this appended point since it is the new last point
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTime).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTime).pose.value(), point1[0], eps_));

    // Check stationary beyond appended point
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTime + 5).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTime + 5).pose.value(), point1[0], eps_));

    // ---------- last evaluation point past last point -----------
    std::vector<TaskPose> point2 = {
        TaskPose(Eigen::Vector3d(2.0, 0.0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0))};
    ASSERT_NO_THROW(sut_->append(point2));

    // Check last evaluation point did not change
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTime + 5).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTime + 5).pose.value(), point1[0], eps_));

    expectedTime = 16;

    // Check stationary at this appended point since it is the new last point
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTime).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTime).pose.value(), point2[0], eps_));

    // ---------- Test append when lastEvalPoint is between 2nd last and last point -------
    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));
    sut_->setInitialPose(initialPose);

    std::vector<TaskPose> twoPoints = {
        TaskPose(
            Eigen::Vector3d(0.0, 1.0, 2.0),
            Eigen::Quaternion<double>(0.0, 0.0, 1.0, 0.0)),  // t=5.712
        TaskPose(
            Eigen::Vector3d(5.0, 1.0, 0.0),
            Eigen::Quaternion<double>(0.0, 0.0, 0.0, 1.0))  // t=14.212
    };

    std::vector<double> expectedTimeInstances = {5.71238898038469, 14.2123889803847};

    ASSERT_NO_THROW(sut_->append(twoPoints));
    TaskSignals lastEvalPoint = sut_->getTrajectoryPoint(8.0);
    // ^Time between 5.712 and 14.212 (2nd last and last time instances)

    // Append when lastEvalPoint is between 2nd last and last point
    ASSERT_NO_THROW(sut_->append(point1));

    expectedTime = 23.3005065812692;

    // Check last evaluation point did not change
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(8.0).pose.value(), lastEvalPoint.pose.value(), eps_));

    // Check stationary at this appended point since it is the new last point
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTime).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTime).pose.value(), point1[0], eps_));

    // Check stationary beyond appended point
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTime + 5).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTime + 5).pose.value(), point1[0], eps_));

    // ------ Test append when lastEvalPoint is before 2nd last point ----------
    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));
    sut_->setInitialPose(initialPose);

    ASSERT_NO_THROW(sut_->append(twoPoints));

    // Smooth trajectory append - Last evaluation point is 0 and second last path point is at 5.7
    ASSERT_NO_THROW(sut_->append(point1));

    // Check initial point was unaffected
    ASSERT_TRUE(sut_->getTrajectoryPoint(0).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(0).pose.value(), initialPose, eps_));

    // Check second last path point (start of this new trajectory) was unaffected
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(expectedTimeInstances[0]).pose.value(), twoPoints[0], eps_));

    expectedTime = 22.7123889803847;  // Expected time of second append

    // Check stationary at and beyond the appended point
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTime).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTime).pose.value(), point1[0], eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(expectedTime + 5).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(expectedTime + 5).pose.value(), point1[0], eps_));
}

TEST_F(CubicTaskTrajectoryShould, RemoveDuplicatePoints) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});

    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaternion<double>(1.0, 0, 0, 0));
    sut_->setInitialPose(initialPose);

    // Try to append the initial pose
    ASSERT_ANY_THROW({
        try {
            sut_->append(std::vector<TaskPose>({initialPose}));
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ(
                "Path contained only duplicate points. There are no new points to append.",
                e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    // Append normally - first trajectory append
    std::vector<TaskPose> point = {
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0))  // time 8.5
    };
    ASSERT_NO_THROW(sut_->append(point));

    // Try to append the same point
    ASSERT_ANY_THROW({
        try {
            sut_->append(point);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ(
                "Path contained only duplicate points. There are no new points to append.",
                e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    // Append two points that are duplicates and one new point
    std::vector<TaskPose> points = {
        TaskPose(
            Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),  // would be 9.5
        TaskPose(
            Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),  // would 10.5
        TaskPose(
            Eigen::Vector3d(2.0, 0, 3.0), Eigen::Quaternion<double>(0, 1.0, 0, 0))  // time 12.5
    };

    ASSERT_NO_THROW(sut_->append(points));

    // Testing duplicate points when the append is when the last eval time is before the last point
    // Evaluate between second and last point (between 8.5 and 12.5) and check duplicate points
    // didn't get added
    ASSERT_FALSE(areAlmostEqual(sut_->getTrajectoryPoint(10.0).pose.value(), points[0], eps_));

    // Append only duplicates
    std::vector<TaskPose> points2 = {
        TaskPose(Eigen::Vector3d(2.0, 0, 3.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(2.0, 0, 3.0), Eigen::Quaternion<double>(0, 1.0, 0, 0))};

    ASSERT_ANY_THROW({
        try {
            sut_->append(points2);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ(
                "Path contained only duplicate points. There are no new points to append.",
                e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    // Append 3 points where 1st and second are duplicates - should only append last point
    std::vector<TaskPose> points3 = {
        TaskPose(Eigen::Vector3d(2.0, 0, 3.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0))};
    ASSERT_NO_THROW(sut_->append(points3));

    // Testing duplicate points when the append is when the last eval time is after the last point
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(20.0).pose.value(), points3[2], eps_));
    // Append only duplicates
    std::vector<TaskPose> points4 = {
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0))};
    ASSERT_ANY_THROW({
        try {
            sut_->append(points4);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ(
                "Path contained only duplicate points. There are no new points to append.",
                e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    // Append 5 points where 2nd, 4th, 5th are duplicates - should only append 1st and 3rd points
    std::vector<TaskPose> points5 = {
        TaskPose(Eigen::Vector3d(2.0, 0, 3.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(2.0, 0, 3.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0))};
    ASSERT_NO_THROW(sut_->append(points5));
}

TEST_F(CubicTaskTrajectoryShould, NotRegisterIncorrectGetTrajectoryPointCalls) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});

    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));
    ASSERT_ANY_THROW({
        try {
            sut_->getTrajectoryPoint(10.0);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ(
                "No trajectories have been generated, cannot retrieve trajectory point", e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaternion<double>(1.0, 0, 0, 0));
    sut_->setInitialPose(initialPose);

    // If evaluating at 10s was registered, this would throw an error - going backwards not allowed
    ASSERT_NO_THROW(TaskSignals lastEvaluationPoint = sut_->getTrajectoryPoint(5.0));

    ASSERT_ANY_THROW({
        try {
            sut_->getTrajectoryPoint(1.0);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ("Going backwards in a trajectory is not permitted", e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    // Append points - new trajectory should start at 5.0 not 1.0
    std::vector<TaskPose> path = {
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),
        TaskPose(Eigen::Vector3d(2.0, 0, 2.0), Eigen::Quaternion<double>(0, 0, 1.0, 0)),
        TaskPose(Eigen::Vector3d(3.0, 3.0, 0), Eigen::Quaternion<double>(0, 0, 0, 1.0))};
    sut_->append(path);

    // If it started the trajectory at 1.0, this would not be true
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(5.0).pose.value(), initialPose, eps_));

    // Check append worked
    ASSERT_TRUE(areAlmostEqual(sut_->getTrajectoryPoint(30.0).pose.value(), path[2], eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(30.0).velocity.value().raw().isZero(eps_));
}

TEST_F(CubicTaskTrajectoryShould, ThrowExceptionIfEmptyAppendPath) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});

    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaternion<double>(1.0, 0, 0, 0));
    sut_->setInitialPose(initialPose);

    // Try to append an empty append path
    std::vector<TaskPose> path = {};

    ASSERT_ANY_THROW({
        try {
            sut_->append(path);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ(
                "Require at least 1 point in the append path to continue the trajectory.",
                e.what());
            throw;  // Re-throw to propagate the exception
        }
    });
}

TEST_F(CubicTaskTrajectoryShould, GetTrajectoryPointOfAnUndefinedTrajectoryWithAnInitialPose) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});

    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(1.1, 2.2, 3.3), Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0));
    sut_->setInitialPose(initialPose);

    double evaluationTime = 10.0;
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(evaluationTime).pose.value(), initialPose, eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(evaluationTime).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(evaluationTime).acceleration.value().raw().isZero(eps_));
}

TEST_F(CubicTaskTrajectoryShould, AllowSetInitialPoseToBeCalledMultipleTimesBeforeAppend) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});

    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose1 =
        TaskPose(Eigen::Vector3d(1.1, 2.2, 3.3), Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0));
    sut_->setInitialPose(initialPose1);

    // Set initial pose again
    TaskPose initialPose2 =
        TaskPose(Eigen::Vector3d(2.0, 2.0, 2.0), Eigen::Quaternion<double>(0.0, 0.0, 1.0, 0.0));
    ASSERT_NO_THROW(sut_->setInitialPose(initialPose2));

    // Check initial pose was reset
    double evaluationTime = 0.0;
    ASSERT_FALSE(
        areAlmostEqual(sut_->getTrajectoryPoint(evaluationTime).pose.value(), initialPose1, eps_));

    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(evaluationTime).pose.value(), initialPose2, eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(evaluationTime).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(evaluationTime).acceleration.value().raw().isZero(eps_));

    evaluationTime = 5.0;
    ASSERT_FALSE(
        areAlmostEqual(sut_->getTrajectoryPoint(evaluationTime).pose.value(), initialPose1, eps_));

    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(evaluationTime).pose.value(), initialPose2, eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(evaluationTime).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(evaluationTime).acceleration.value().raw().isZero(eps_));

    // Try to set initial pose again after it has been evaluated but no points have been appended
    TaskPose initialPose3 =
        TaskPose(Eigen::Vector3d(3.0, 3.0, 3.0), Eigen::Quaternion<double>(0.0, 1.0, 0.0, 0.0));
    ASSERT_NO_THROW(sut_->setInitialPose(initialPose3));

    // Check it changed the initial pose
    evaluationTime = 10.0;
    ASSERT_FALSE(
        areAlmostEqual(sut_->getTrajectoryPoint(evaluationTime).pose.value(), initialPose1, eps_));
    ASSERT_FALSE(
        areAlmostEqual(sut_->getTrajectoryPoint(evaluationTime).pose.value(), initialPose2, eps_));

    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(evaluationTime).pose.value(), initialPose3, eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(evaluationTime).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(evaluationTime).acceleration.value().raw().isZero(eps_));

    // Append a point
    std::vector<TaskPose> point = {
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0))};

    ASSERT_NO_THROW(sut_->append(point));

    // Try to set the initial pose now that a trajectory is defined
    ASSERT_ANY_THROW({
        try {
            sut_->setInitialPose(initialPose1);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ(
                "Cannot set initial pose as a trajectory has already been defined."
                " To add points to the existing trajectory, use append(). To start the trajectory "
                "again, call reset() then setInitialPose().",
                e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    ASSERT_ANY_THROW({
        try {
            sut_->setInitialPose(initialPose2);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ(
                "Cannot set initial pose as a trajectory has already been defined."
                " To add points to the existing trajectory, use append(). To start the trajectory "
                "again, call reset() then setInitialPose().",
                e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    ASSERT_ANY_THROW({
        try {
            sut_->setInitialPose(initialPose3);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ(
                "Cannot set initial pose as a trajectory has already been defined."
                " To add points to the existing trajectory, use append(). To start the trajectory "
                "again, call reset() then setInitialPose().",
                e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    // Evaluate new final pose that should be the appended point
    evaluationTime = 20.0;
    ASSERT_FALSE(
        areAlmostEqual(sut_->getTrajectoryPoint(evaluationTime).pose.value(), initialPose1, eps_));
    ASSERT_FALSE(
        areAlmostEqual(sut_->getTrajectoryPoint(evaluationTime).pose.value(), initialPose2, eps_));
    ASSERT_FALSE(
        areAlmostEqual(sut_->getTrajectoryPoint(evaluationTime).pose.value(), initialPose3, eps_));

    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(evaluationTime).pose.value(), point[0], eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(evaluationTime).velocity.value().raw().isZero(eps_));
}

TEST_F(CubicTaskTrajectoryShould, AllowInitialPoseToBeSetAfterAReset) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});

    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(1.1, 2.2, 3.3), Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0));
    sut_->setInitialPose(initialPose);

    // Append a point
    std::vector<TaskPose> point = {
        TaskPose(Eigen::Vector3d(1.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0))};

    ASSERT_NO_THROW(sut_->append(point));

    // Try to set initial pose again once a trajectory has already been defined
    TaskPose newInitialPose =
        TaskPose(Eigen::Vector3d(2.0, 2.0, 2.0), Eigen::Quaternion<double>(0.0, 0.0, 1.0, 0.0));

    ASSERT_ANY_THROW({
        try {
            sut_->setInitialPose(newInitialPose);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ(
                "Cannot set initial pose as a trajectory has already been defined."
                " To add points to the existing trajectory, use append(). To start the trajectory "
                "again, call reset() then setInitialPose().",
                e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    // Do a reset and try and set the initial pose again
    sut_->reset();
    ASSERT_NO_THROW(sut_->setInitialPose(newInitialPose));

    // Check it changed the initial pose
    double evaluationTime = 0.0;
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(evaluationTime).pose.value(), newInitialPose, eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(evaluationTime).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(evaluationTime).acceleration.value().raw().isZero(eps_));

    evaluationTime = 5.0;
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(evaluationTime).pose.value(), newInitialPose, eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(evaluationTime).velocity.value().raw().isZero(eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(evaluationTime).acceleration.value().raw().isZero(eps_));
}

TEST_F(CubicTaskTrajectoryShould, AdjustAllTimeWithDifferentPointsAndMaxVelsAndAccs) {
    // Create a trajectory object
    TaskVelocity maxVel({1.0, 1.5, 2.0, 2.5, 3.0, 3.5});
    TaskAcceleration maxAcc({4.0, 4.5, 5.0, 5.5, 6.0, 6.5});

    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(1.1, 2.2, 3.3), Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0));
    sut_->setInitialPose(initialPose);

    // Append 2 points
    std::vector<TaskPose> path = {
        TaskPose(Eigen::Vector3d(2.0, 0, 5.0), Eigen::Quaternion<double>(0, 1.0, 0, 0)),   // 3.2
        TaskPose(Eigen::Vector3d(2.0, 1.5, 4.0), Eigen::Quaternion<double>(0, 0, 1.0, 0))  // 5.97
    };
    sut_->append(path);

    std::vector<double> times = {3.2, 5.97450175016246};  // Time instances of the appended points

    // Check each component
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).pose.value().getPosition()[0], 1.1, eps_);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).pose.value().getPosition()[1], 2.2, eps_);
    ASSERT_NEAR(sut_->getTrajectoryPoint(0).pose.value().getPosition()[2], 3.3, eps_);
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(0).pose.value().getOrientation(),
        Orientation(Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0)), eps_));

    ASSERT_NEAR(sut_->getTrajectoryPoint(times[0]).pose.value().getPosition()[0], 2.0, eps_);
    ASSERT_NEAR(sut_->getTrajectoryPoint(times[0]).pose.value().getPosition()[1], 0.0, eps_);
    ASSERT_NEAR(sut_->getTrajectoryPoint(times[0]).pose.value().getPosition()[2], 5.0, eps_);
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(times[0]).pose.value().getOrientation(),
        Orientation(Eigen::Quaternion<double>(0.0, 1.0, 0.0, 0.0)), eps_));

    ASSERT_NEAR(sut_->getTrajectoryPoint(times[1]).pose.value().getPosition()[0], 2.0, eps_);
    ASSERT_NEAR(sut_->getTrajectoryPoint(times[1]).pose.value().getPosition()[1], 1.5, eps_);
    ASSERT_NEAR(sut_->getTrajectoryPoint(times[1]).pose.value().getPosition()[2], 4.0, eps_);
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(times[1]).pose.value().getOrientation(),
        Orientation(Eigen::Quaternion<double>(0.0, 0.0, 1.0, 0.0)), eps_));
    ASSERT_TRUE(sut_->getTrajectoryPoint(times[1]).velocity.value().raw().isZero(eps_));
}

// TODO(jplayang): Tests disabled because increasing acceleration limit increases movement time.
// Test can be adapted (see TODO comments in test) and enabled once time calculations are fixed.
TEST_F(CubicTaskTrajectoryShould, DISABLED_SetProfileVelocityCorrectly) {
    // Create a trajectory object
    TaskVelocity maxVel({0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    TaskAcceleration maxAcc({0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0));
    sut_->setInitialPose(initialPose);

    // Append 2 points
    std::vector<TaskPose> path = {
        TaskPose(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0)),
        TaskPose(Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Quaternion<double>(0.0, 1.0, 0.0, 0.0))};
    sut_->append(path);

    sut_->append({initialPose});  // Append initial pose so it is like starting it again

    std::vector<double> originalExpectedTimes = {
        2.82574185835055, 5.10619449019234, 8.46238898038469};
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(originalExpectedTimes[0]).pose.value(), path[0], eps_));
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(originalExpectedTimes[1]).pose.value(), path[1], eps_));
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(originalExpectedTimes[2]).pose.value(), initialPose, eps_));

    // Decrease the max vel
    maxVel = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
    sut_->setProfileVelocity(maxVel);

    // Append the points again - will start trajectory at originalExpectedTimes[2] because
    // lastEvaluationPoint == last point
    sut_->append(path);

    // If the velocity hadn't changed, it would hit the points at 10.2123889803847 and
    // 13.568583470577 (time taken to go from appended initial pose to the first appended point and
    // the second appended point respectively)
    std::vector<double> extendedOriginalExpectedTimes = {
        originalExpectedTimes[2] + originalExpectedTimes[0],
        originalExpectedTimes[2] + originalExpectedTimes[1]};

    // TODO(jplayang): Need the actual times to be larger - not currently the case
    std::vector<double> newExpectedTimes = {10.9623889803847, 16.6747779607694};

    // Check it doesn't match the points at the extendedOriginalExpectedTimes but does match at
    // the new larger expected times
    ASSERT_FALSE(areAlmostEqual(
        sut_->getTrajectoryPoint(extendedOriginalExpectedTimes[0]).pose.value(), path[0], eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(newExpectedTimes[0]).pose.value(), path[0], eps_));

    ASSERT_FALSE(areAlmostEqual(
        sut_->getTrajectoryPoint(extendedOriginalExpectedTimes[1]).pose.value(), path[1], eps_));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTrajectoryPoint(newExpectedTimes[1]).pose.value(), path[1], eps_));
}

// TODO(jplayang): Tests disabled because increasing acceleration limit increases movement time.
// Test can be adapted (see TODO comments in test) and enabled once time calculations are fixed.
TEST_F(CubicTaskTrajectoryShould, DISABLED_SetProfileAccelerationCorrectly) {
    // Create a trajectory object
    TaskVelocity maxVel({20.0, 20.0, 20.0, 20.0, 20.0, 20.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});

    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0));
    sut_->setInitialPose(initialPose);

    // Append 2 points
    std::vector<TaskPose> path = {
        TaskPose(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0)),
        TaskPose(Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Quaternion<double>(0.0, 1.0, 0.0, 0.0))};
    sut_->append(path);
    sut_->append({initialPose});  // Append initial pose so it is like starting it again

    std::vector<double> originalExpectedTimes = {
        1.57735026918963, 3.60067697713611, 5.6240036850826};
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(originalExpectedTimes[0]).pose.value(), path[0], eps_));
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(originalExpectedTimes[1]).pose.value(), path[1], eps_));
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(originalExpectedTimes[2]).pose.value(), initialPose, eps_));

    // Decrease the max acc
    maxAcc = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    sut_->setProfileAcceleration(maxAcc);

    // Append the points again - will start trajectory at originalExpectedTimes[2] because
    // lastEvaluationPoint == last point
    sut_->append(path);

    // If the velocity hadn't changed, it would hit the points at 10.2123889803847 and
    // 13.568583470577 (time taken to go from appended initial pose to the first appended point and
    // the second appended point respectively)
    std::vector<double> extendedOriginalExpectedTimes = {
        originalExpectedTimes[2] + originalExpectedTimes[0],
        originalExpectedTimes[2] + originalExpectedTimes[1]};

    // TODO(jplayang): Need the actual times to be larger - not currently the case
    std::vector<double> newExpectedTimes = {6.91267881967741, 8.42434217365066};

    // Check it doesn't match the points at the extendedOriginalExpectedTimes but does match at
    // the new larger expected times
    ASSERT_FALSE(areAlmostEqual(
        sut_->getTrajectoryPoint(extendedOriginalExpectedTimes[0]).pose.value(), path[0], eps_));
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(newExpectedTimes[0]).pose.value(), path[0], eps_));

    ASSERT_FALSE(areAlmostEqual(
        sut_->getTrajectoryPoint(extendedOriginalExpectedTimes[1]).pose.value(), path[1], eps_));
    ASSERT_TRUE(areAlmostEqual(
        sut_->getTrajectoryPoint(newExpectedTimes[1]).pose.value(), path[1], eps_));
}

TEST_F(CubicTaskTrajectoryShould, ThrowErrorIfEvaluatingNegativeTime) {
    // Create a trajectory object
    TaskVelocity maxVel({20.0, 20.0, 20.0, 20.0, 20.0, 20.0});
    TaskAcceleration maxAcc({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});

    sut_.reset(new CubicTaskTrajectory(maxVel, maxAcc));

    // Set initial pose
    TaskPose initialPose =
        TaskPose(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0));
    sut_->setInitialPose(initialPose);

    // Append 2 points
    std::vector<TaskPose> path = {
        TaskPose(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0)),
        TaskPose(Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Quaternion<double>(0.0, 1.0, 0.0, 0.0))};
    sut_->append(path);

    // Check negative time
    ASSERT_ANY_THROW({
        try {
            sut_->getTrajectoryPoint(-0.000001);
        } catch (const std::runtime_error& e) {
            ASSERT_STREQ("Going backwards in a trajectory is not permitted", e.what());
            throw;  // Re-throw to propagate the exception
        }
    });
}
