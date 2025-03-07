/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
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
#include "Types/Types.hpp"
#include "TrajectoryGeneratorDeprecated/TrajectoryData.hpp"
#include "TrajectoryGeneratorDeprecated/TaskLinearTrajectory.hpp"

using crf::control::trajectorygeneratordeprecated::TaskLinearTrajectory;

using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;

using testing::_;
using testing::Invoke;
using testing::NiceMock;

class TaskLinearTrajectoryShould: public ::testing::Test {
 protected:
    TaskLinearTrajectoryShould(): logger_("TaskLinearTrajectoryShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~TaskLinearTrajectoryShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<TaskLinearTrajectory> sut_;
};

TEST_F(TaskLinearTrajectoryShould, returnEmptyVectorsIfPathWasNotCalculated) {
    TaskAcceleration MaxAcceleration({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskVelocity maxVelocity({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

    float timeStep = 0.01;
    sut_.reset(new TaskLinearTrajectory(maxVelocity,
        MaxAcceleration,
        timeStep));

    ASSERT_EQ(boost::none, sut_->getDuration());
    ASSERT_EQ(boost::none, sut_->getTaskPose(0));
    ASSERT_EQ(boost::none, sut_->getTaskVelocity(0));
    ASSERT_EQ(boost::none, sut_->getTaskAcceleration(0));
    ASSERT_EQ(boost::none, sut_->getTaskTrajectory());
}

TEST_F(TaskLinearTrajectoryShould, returnFalseAndEmptyVectorsIfPathIsTooSmall) {
    TaskAcceleration MaxAcceleration({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskVelocity maxVelocity({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    float timeStep = 0.01;
    sut_.reset(new TaskLinearTrajectory(maxVelocity,
        MaxAcceleration,
        timeStep));

    std::vector<TaskPose> path;
    TaskPose point;
    point =
        TaskPose({0.0, 0.0, 0.0}, crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0}));
    path.push_back(point);
    ASSERT_FALSE(sut_->computeTrajectory(path));

    ASSERT_EQ(boost::none, sut_->getDuration());
    ASSERT_EQ(boost::none, sut_->getTaskPose(0));
    ASSERT_EQ(boost::none, sut_->getTaskVelocity(0));
    ASSERT_EQ(boost::none, sut_->getTaskAcceleration(0));
    ASSERT_EQ(boost::none, sut_->getTaskTrajectory());
}

TEST_F(TaskLinearTrajectoryShould, returnEmptyVectorIfInputTimeIsNotWithinTheTrajectoryDuration) { // NOLINT
    TaskAcceleration MaxAcceleration({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskVelocity maxVelocity({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    float timeStep = 0.01;
    ASSERT_NO_THROW(sut_.reset(new TaskLinearTrajectory(maxVelocity,
        MaxAcceleration,
        timeStep)));

    std::vector<TaskPose> path;
    TaskPose point;
    point =
        TaskPose({0.0, 0.0, 0.0}, crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0}));
    path.push_back(point);
    point =
        TaskPose({0.1, 0.0, 0.0}, crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0}));
    path.push_back(point);
    ASSERT_TRUE(sut_->computeTrajectory(path));

    float incorrectTime = sut_->getDuration().get()+1;
    ASSERT_EQ(boost::none, sut_->getTaskPose(incorrectTime));
    ASSERT_EQ(boost::none, sut_->getTaskVelocity(incorrectTime));
    ASSERT_EQ(boost::none, sut_->getTaskPose(-1));
    ASSERT_EQ(boost::none, sut_->getTaskVelocity(-1));

    float correctTime = sut_->getDuration().get()*0.5;
    ASSERT_NE(boost::none, sut_->getTaskPose(correctTime));
    ASSERT_NE(boost::none, sut_->getTaskVelocity(correctTime));
}

TEST_F(TaskLinearTrajectoryShould, returnEmptyVectorIfTheTrajectoryGeneratorAlgorithmFails) {
    TaskAcceleration MaxAcceleration({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskVelocity maxVelocity({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    float timeStep = 1;
    ASSERT_NO_THROW(sut_.reset(new TaskLinearTrajectory(maxVelocity,
        MaxAcceleration,
        timeStep)));

    std::vector<TaskPose> path;
    TaskPose point;
    point =
        TaskPose({0.0, 0.0, 0.0}, crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0}));
    path.push_back(point);
    ASSERT_FALSE(sut_->computeTrajectory(path));

    ASSERT_EQ(boost::none, sut_->getTaskPose(0));
    ASSERT_EQ(boost::none, sut_->getTaskVelocity(0));
    ASSERT_EQ(boost::none, sut_->getTaskAcceleration(0));
    ASSERT_EQ(boost::none, sut_->getTaskTrajectory());
}

TEST_F(TaskLinearTrajectoryShould, returnDifferentValuesTheSecondTimeTheTrajectoryIsComputed) {
    TaskAcceleration MaxAcceleration({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskVelocity maxVelocity({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    float timeStep = 0.01;

    ASSERT_NO_THROW(sut_.reset(new TaskLinearTrajectory(maxVelocity,
        MaxAcceleration,
        timeStep)));

    std::vector<TaskPose> longPath;
    TaskPose point;
    point =
        TaskPose({0.0, 0.0, 0.0}, crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0}));
    longPath.push_back(point);
    point =
        TaskPose({0.0, 0.2, 1.0}, crf::math::rotation::CardanXYZ({0.0, 0.2, 1.0}));
    longPath.push_back(point);
    point =
        TaskPose({0.0, 3.0, 0.5}, crf::math::rotation::CardanXYZ({0.0, 3.0, 0.5}));
    longPath.push_back(point);
    point =
        TaskPose({1.1, 2.0, 0.0}, crf::math::rotation::CardanXYZ({1.1, 2.0, 0.0}));
    longPath.push_back(point);
    point =
        TaskPose({1.0, 0.0, 0.0}, crf::math::rotation::CardanXYZ({1.0, 0.0, 0.0}));
    longPath.push_back(point);
    point =
        TaskPose({0.0, 1.0, 0.0}, crf::math::rotation::CardanXYZ({0.0, 1.0, 0.0}));
    longPath.push_back(point);
    point =
        TaskPose({0.0, 0.0, 1.0}, crf::math::rotation::CardanXYZ({0.0, 0.0, 1.0}));
    longPath.push_back(point);
    ASSERT_TRUE(sut_->computeTrajectory(longPath));
    float duration = sut_->getDuration().get();


    std::vector<TaskPose> path;
    point =
        TaskPose({0.0, 0.0, 0.0}, crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0}));
    path.push_back(point);
    point =
        TaskPose({0.0, 0.2, 1.0}, crf::math::rotation::CardanXYZ({0.0, 0.2, 1.0}));
    path.push_back(point);
    point =
        TaskPose({0.0, 3.0, 0.5}, crf::math::rotation::CardanXYZ({0.0, 3.0, 0.5}));
    path.push_back(point);
    ASSERT_TRUE(sut_->computeTrajectory(path));
    ASSERT_GT(duration, sut_->getDuration().get());
}
