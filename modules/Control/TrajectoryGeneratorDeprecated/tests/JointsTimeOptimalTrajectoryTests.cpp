/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <boost/optional.hpp>

#include "EventLogger/EventLogger.hpp"
#include "TrajectoryGeneratorDeprecated/TrajectoryData.hpp"
#include "TrajectoryGeneratorDeprecated/JointsTimeOptimalTrajectory.hpp"

using crf::control::trajectorygeneratordeprecated::JointsTimeOptimalTrajectory;

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

using testing::_;
using testing::Invoke;
using testing::NiceMock;

class JointsTimeOptimalTrajectoryShould: public ::testing::Test {
 protected:
    JointsTimeOptimalTrajectoryShould(): logger_("JointsTimeOptimalTrajectoryShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~JointsTimeOptimalTrajectoryShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<JointsTimeOptimalTrajectory> sut_;
};

TEST_F(JointsTimeOptimalTrajectoryShould, returnFalseIfInputsDimensionAreDifferent) {
    JointAccelerations incorrectMaxAcceleration(2);
    incorrectMaxAcceleration =  { 1.0, 1.0 };
    JointVelocities maxVelocity(3);
    maxVelocity = { 1.0, 1.0, 1.0 };
    float timeStep = 0.01;
    float maxDeviation = 0.1;
    ASSERT_THROW(sut_.reset(new JointsTimeOptimalTrajectory(maxVelocity,
        incorrectMaxAcceleration,
        timeStep,
        maxDeviation)), std::runtime_error);

    JointAccelerations correctMaxAcceleration(3);
    correctMaxAcceleration = { 1.0, 1.0, 1.0 };
    ASSERT_NO_THROW(sut_.reset(new JointsTimeOptimalTrajectory(maxVelocity,
        correctMaxAcceleration,
        timeStep,
        maxDeviation)));

    std::vector<JointPositions> correctPath;
    JointPositions correctPoint(3);
    correctPoint = { 0.0, 0.0, 0.0 };
    correctPath.push_back(correctPoint);
    correctPoint = { 0.0, 0.2, 1.0 };
    correctPath.push_back(correctPoint);
    correctPoint = { 0.0, 3.0, 0.5 };
    correctPath.push_back(correctPoint);
    ASSERT_TRUE(sut_->computeTrajectory(correctPath));

    std::vector<JointPositions> incorrectPath;
    JointPositions incorrectPoint(2);
    incorrectPoint = { 0.0, 0.0 };
    incorrectPath.push_back(incorrectPoint);
    incorrectPoint = { 0.0, 0.2 };
    incorrectPath.push_back(incorrectPoint);
    incorrectPoint = { 0.0, 3.0 };
    incorrectPath.push_back(incorrectPoint);
    ASSERT_FALSE(sut_->computeTrajectory(incorrectPath));
}

TEST_F(JointsTimeOptimalTrajectoryShould, returnEmptyVectorsIfPathWasNotCalculated) {
    JointAccelerations MaxAcceleration(3);
    MaxAcceleration = { 1.0, 1.0, 1.0 };
    JointVelocities maxVelocity(3);
    maxVelocity = { 1.0, 1.0, 1.0 };
    float timeStep = 0.01;
    float maxDeviation = 0.1;
    ASSERT_NO_THROW(sut_.reset(new JointsTimeOptimalTrajectory(maxVelocity,
        MaxAcceleration,
        timeStep,
        maxDeviation)));

    ASSERT_EQ(boost::none, sut_->getDuration());
    ASSERT_EQ(boost::none, sut_->getJointPositions(1));
    ASSERT_EQ(boost::none, sut_->getJointVelocities(1));

    auto trajectoryResult = sut_->getJointsTrajectory();
    ASSERT_EQ(boost::none, trajectoryResult);
}

TEST_F(JointsTimeOptimalTrajectoryShould, returnFalseAndEmptyVectorsIfPathIsTooSmall) {
    JointAccelerations MaxAcceleration(3);
    MaxAcceleration = { 1.0, 1.0, 1.0 };
    JointVelocities maxVelocity(3);
    maxVelocity = { 1.0, 1.0, 1.0 };
    float timeStep = 0.01;
    float maxDeviation = 0.1;
    ASSERT_NO_THROW(sut_.reset(new JointsTimeOptimalTrajectory(maxVelocity,
        MaxAcceleration,
        timeStep,
        maxDeviation)));

    std::vector<JointPositions> path;
    JointPositions point(3);
    point = { 0.0, 0.0, 0.0 };
    path.push_back(point);
    ASSERT_FALSE(sut_->computeTrajectory(path));

    ASSERT_EQ(boost::none, sut_->getDuration());
    ASSERT_EQ(boost::none, sut_->getJointPositions(1));
    ASSERT_EQ(boost::none, sut_->getJointVelocities(1));

    auto trajectoryResult = sut_->getJointsTrajectory();
    ASSERT_EQ(boost::none, trajectoryResult);
}

TEST_F(JointsTimeOptimalTrajectoryShould, returnEmptyVectorIfInputTimeIsNotWithinTheTrajectoryDuration) { // NOLINT
    JointAccelerations MaxAcceleration(3);
    MaxAcceleration = { 1.0, 1.0, 1.0 };
    JointVelocities maxVelocity(3);
    maxVelocity = { 1.0, 1.0, 1.0 };
    float timeStep = 0.01;
    float maxDeviation = 0.1;
    ASSERT_NO_THROW(sut_.reset(new JointsTimeOptimalTrajectory(maxVelocity,
        MaxAcceleration,
        timeStep,
        maxDeviation)));

    std::vector<JointPositions> path;
    JointPositions point(3);
    point = { 0.0, 0.0, 0.0 };
    path.push_back(point);
    point = { 0.0, 0.2, 1.0 };
    path.push_back(point);
    point = { 0.0, 3.0, 0.5 };
    path.push_back(point);
    ASSERT_TRUE(sut_->computeTrajectory(path));

    float incorrectTime = sut_->getDuration().get()+1;
    ASSERT_EQ(boost::none, sut_->getJointPositions(incorrectTime));
    ASSERT_EQ(boost::none, sut_->getJointVelocities(incorrectTime));
    ASSERT_EQ(boost::none, sut_->getJointPositions(-1));
    ASSERT_EQ(boost::none, sut_->getJointVelocities(-1));

    float correctTime = sut_->getDuration().get()*0.5;
    ASSERT_EQ(point.size(), sut_->getJointPositions(correctTime).get().size());
    ASSERT_EQ(point.size(), sut_->getJointVelocities(correctTime).get().size());
}

TEST_F(JointsTimeOptimalTrajectoryShould, returnEmptyVectorIfTheTrajectoryGeneratorAlgorithmFails) {
    JointAccelerations MaxAcceleration(3);
    MaxAcceleration = { 1.0, 1.0, 1.0 };
    JointVelocities maxVelocity(3);
    maxVelocity = { 1.0, 1.0, 1.0 };
    float timeStep = 1;
    float maxDeviation = 0.1;
    ASSERT_NO_THROW(sut_.reset(new JointsTimeOptimalTrajectory(maxVelocity,
        MaxAcceleration,
        timeStep,
        maxDeviation)));

    std::vector<JointPositions> path;
    JointPositions point(3);
    point = { 0.0, 0.0, 0.0 };
    path.push_back(point);
    point = { 0.0, 0.2, 1.0 };
    path.push_back(point);
    point = { 0.0, 3.0, 0.5 };
    path.push_back(point);
    point = { 1.1, 2.0, 0.0 };
    path.push_back(point);
    point = { 1.0, 0.0, 0.0 };
    path.push_back(point);
    point = { 0.0, 1.0, 0.0 };
    path.push_back(point);
    point = { 0.0, 0.0, 1.0 };
    path.push_back(point);
    ASSERT_FALSE(sut_->computeTrajectory(path));

    ASSERT_EQ(boost::none, sut_->getDuration());
    ASSERT_EQ(boost::none, sut_->getJointPositions(1));
    ASSERT_EQ(boost::none, sut_->getJointVelocities(1));

    auto trajectoryResult = sut_->getJointsTrajectory();
    ASSERT_EQ(boost::none, trajectoryResult);
}

TEST_F(JointsTimeOptimalTrajectoryShould, returnEmptyVectorOfAccelerationAndTorque) {
    JointAccelerations MaxAcceleration(3);
    MaxAcceleration = { 1.0, 1.0, 1.0 };
    JointVelocities maxVelocity(3);
    maxVelocity = { 1.0, 1.0, 1.0 };
    float timeStep = 0.01;
    float maxDeviation = 0.1;
    ASSERT_NO_THROW(sut_.reset(new JointsTimeOptimalTrajectory(maxVelocity,
        MaxAcceleration,
        timeStep,
        maxDeviation)));

    std::vector<JointPositions> path;
    JointPositions point(3);
    point = { 0.0, 0.0, 0.0 };
    path.push_back(point);
    point = { 0.0, 0.2, 1.0 };
    path.push_back(point);
    point = { 0.0, 3.0, 0.5 };
    path.push_back(point);
    ASSERT_TRUE(sut_->computeTrajectory(path));

    ASSERT_EQ(boost::none, sut_->getJointAccelerations(1));
    ASSERT_EQ(boost::none, sut_->getJointForceTorques(1));

    auto trajectoryResult = sut_->getJointsTrajectory();
    ASSERT_TRUE(trajectoryResult.get().acceleration.empty());
    ASSERT_TRUE(trajectoryResult.get().torque.empty());
}

TEST_F(JointsTimeOptimalTrajectoryShould, returnDifferentValuesTheSecondTimeTheTrajectoryIsComputed) {  // NOLINT
    JointAccelerations MaxAcceleration(3);
    MaxAcceleration = { 1.0, 1.0, 1.0 };
    JointVelocities maxVelocity(3);
    maxVelocity = { 1.0, 1.0, 1.0 };
    float timeStep = 0.01;
    float maxDeviation = 0.1;
    ASSERT_NO_THROW(sut_.reset(new JointsTimeOptimalTrajectory(maxVelocity,
        MaxAcceleration,
        timeStep,
        maxDeviation)));

    std::vector<JointPositions> longPath;
    JointPositions point(3);
    point = { 0.0, 0.0, 0.0 };
    longPath.push_back(point);
    point = { 0.0, 0.2, 1.0 };
    longPath.push_back(point);
    point = { 0.0, 3.0, 0.5 };
    longPath.push_back(point);
    point = { 1.1, 2.0, 0.0 };
    longPath.push_back(point);
    point = { 1.0, 0.0, 0.0 };
    longPath.push_back(point);
    point = { 0.0, 1.0, 0.0 };
    longPath.push_back(point);
    point = { 0.0, 0.0, 1.0 };
    longPath.push_back(point);
    ASSERT_TRUE(sut_->computeTrajectory(longPath));
    float duration = sut_->getDuration().get();

    std::vector<JointPositions> path;
    point = { 0.0, 0.0, 0.0 };
    path.push_back(point);
    point = { 0.0, 0.2, 1.0 };
    path.push_back(point);
    point = { 0.0, 3.0, 0.5 };
    path.push_back(point);
    ASSERT_TRUE(sut_->computeTrajectory(path));
    ASSERT_NE(duration, sut_->getDuration().get());
}

TEST_F(JointsTimeOptimalTrajectoryShould, returnTrueIfThePathGeneratesASituationInWhichTheNormOfZeroIsCalculated) {   // NOLINT
    JointVelocities maxVel({0.628f, 0.628f, 0.628f, 0.837f, 0.837f, 0.837f});
    JointAccelerations maxAcc({0.6f, 0.6f, 0.6f, 0.8f, 0.8f, 0.8f});
    float timeStep = 0.01;
    float maxDeviation = 0.001;
    ASSERT_NO_THROW(sut_.reset(new JointsTimeOptimalTrajectory(maxVel,
        maxAcc,
        timeStep,
        maxDeviation)));
    std::vector<JointPositions> path1;
    path1.push_back(JointPositions({-0.50352f, -0.50561f, -0.50458f,
        -0.52228f, -0.52235f, -0.52707f}));
    path1.push_back(JointPositions({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}));
    path1.push_back(JointPositions({0.6f, 0.6f, 0.6f, 0.6f, 0.6f, 0.6f}));
    path1.push_back(JointPositions({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}));
    ASSERT_TRUE(sut_->computeTrajectory(path1));
    ASSERT_TRUE(sut_->computeTrajectory(path1));
    ASSERT_TRUE(sut_->computeTrajectory(path1));
    ASSERT_TRUE(sut_->computeTrajectory(path1));
}
