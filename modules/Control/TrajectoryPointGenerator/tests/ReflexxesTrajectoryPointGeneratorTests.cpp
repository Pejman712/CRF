/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <vector>
#include <string>
#include <thread>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"
#include "TrajectoryPointGenerator/ITrajectoryPointGenerator.hpp"
#include "TrajectoryPointGenerator/ReflexxesTrajectoryPointGenerator.hpp"

#define TASK_SPACE_SIZE 6

using crf::control::trajectorypointgenerator::ReflexxesTrajectoryPointGenerator;
using crf::control::trajectorypointgenerator::ITrajectoryPointGenerator;
using crf::control::trajectorypointgenerator::ControlMode;

using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskTrajectoryData;
using crf::utility::types::areAlmostEqual;

using testing::_;
using testing::Invoke;
using testing::NiceMock;

class ReflexxesTrajectoryPointGeneratorTestsShould: public ::testing::Test {
 protected:
    ReflexxesTrajectoryPointGeneratorTestsShould():
        logger_("ReflexxesTrajectoryPointGeneratorTestsShould"),
        timeStep_(0.001) {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        for (int i = 0; i < TASK_SPACE_SIZE; i++) {
            selectedDim_.push_back(true);
        }
        sut_.reset(new ReflexxesTrajectoryPointGenerator(ControlMode::VELOCITY, selectedDim_,
                    timeStep_));
        setDefaultMotionConstraints();
    }
    ~ReflexxesTrajectoryPointGeneratorTestsShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    bool setDefaultMotionConstraints() {
        TaskTrajectoryData maximalPoint;
        maximalPoint.velocity = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        maximalPoint.acceleration = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
        return  sut_->updateMotionConstraints(maximalPoint);
    }
    crf::utility::logger::EventLogger logger_;
    std::vector<bool> selectedDim_;
    float timeStep_;
    std::unique_ptr<ITrajectoryPointGenerator> sut_;
};


TEST_F(ReflexxesTrajectoryPointGeneratorTestsShould, throwWithInvalidSpaceSize) {
    std::vector<bool> selectedDim;
    for (int i = 0; i < TASK_SPACE_SIZE+1; i++) {
        selectedDim.push_back(true);
    }
    ASSERT_THROW(sut_.reset(new ReflexxesTrajectoryPointGenerator(
        ControlMode::VELOCITY, selectedDim, timeStep_)), std::runtime_error);
}

TEST_F(ReflexxesTrajectoryPointGeneratorTestsShould,
       throwWithInvalidNumberOfDegreesOfFreedom) {
    std::vector<bool> selectedDim;
    ASSERT_THROW(sut_.reset(new ReflexxesTrajectoryPointGenerator(
        ControlMode::VELOCITY, selectedDim, timeStep_)), std::runtime_error);
}

TEST_F(ReflexxesTrajectoryPointGeneratorTestsShould, returnFalseIfDesiredTargetOverMaximumValue) {
    TaskVelocity invalidVelocity({5.0, 5.0, 5.0, 5.0, 5.0, 5.0});
    ASSERT_TRUE(sut_->updateVelocityTarget(invalidVelocity));
    ASSERT_EQ(sut_->getControlMode(), ControlMode::VELOCITY);
    TaskTrajectoryData maximalPoint;
    maximalPoint.velocity = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
    ASSERT_FALSE(sut_->updateMotionConstraints(maximalPoint));
    ASSERT_FALSE(sut_->getTaskTrajectoryPoint());
    TaskTrajectoryData somePoint = maximalPoint;
    ASSERT_TRUE(sut_->updateCurrentState(somePoint));
    ASSERT_FALSE(sut_->getTaskTrajectoryPoint());
}


TEST_F(ReflexxesTrajectoryPointGeneratorTestsShould, notGetResultsIfInputValuesUndefined) {
    std::vector<bool> selectedDim = {true, false, false, false, false, false};
    sut_.reset(new ReflexxesTrajectoryPointGenerator(ControlMode::POSITION, selectedDim,
                timeStep_));
    ASSERT_FALSE(sut_->getTaskTrajectoryPoint());
}

TEST_F(ReflexxesTrajectoryPointGeneratorTestsShould, notChangeResultsIfNoTargetDefined) {
    TaskTrajectoryData initData;
    initData.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ASSERT_TRUE(sut_->getTaskTrajectoryPoint());
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskTrajectoryPoint().get().velocity, initData.velocity));
}

TEST_F(ReflexxesTrajectoryPointGeneratorTestsShould,
        notChangeResultsIfNoVelocityDefined) {
    TaskTrajectoryData data {};
    data.pose = crf::utility::types::TaskPose(data.pose.getPosition(), data.pose.getCardanXYZ());
    ASSERT_TRUE(sut_->updateVelocityTarget(data.velocity));
    ASSERT_TRUE(sut_->updateCurrentState(sut_->getTaskTrajectoryPoint().get()));
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskTrajectoryPoint().get().pose, data.pose));
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskTrajectoryPoint().get().velocity, data.velocity));
    ASSERT_TRUE(
        areAlmostEqual(sut_->getTaskTrajectoryPoint().get().acceleration, data.acceleration));
}


TEST_F(ReflexxesTrajectoryPointGeneratorTestsShould, returnGoodControlResultOnPerfectControlLoop) {
    TaskVelocity targetVelocity({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    ASSERT_TRUE(sut_->updateVelocityTarget(targetVelocity));
    TaskTrajectoryData result;
    while (std::abs(targetVelocity[0] - result.velocity[0]) > 1e-3) {
        result = sut_->getTaskTrajectoryPoint().get();
        // Here fooling the controller by passing new computed value as current value to controller
        ASSERT_TRUE(sut_->updateCurrentState(result));
    }
}

TEST_F(ReflexxesTrajectoryPointGeneratorTestsShould, keepVelocityIfTargetReachedAndFlagTriggered) {
    TaskTrajectoryData data;
    data.velocity = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
    data.pose = TaskPose({10.0, 1.0, 1.0},
        crf::math::rotation::CardanXYZ({1.0, 1.0, 1.0}));
    TaskTrajectoryData result;

    // Should return false since we are in velocity mode by default
    ASSERT_FALSE(sut_->updatePositionTarget(data.pose));

    ASSERT_TRUE(sut_->updateVelocityTarget(data.velocity));

    while (std::abs(data.velocity[0] - result.velocity[0]) >  1e-3) {
        result = sut_->getTaskTrajectoryPoint().get();
        ASSERT_TRUE(sut_->updateCurrentState(result));
    }
    std::this_thread::sleep_for(std::chrono::duration<float>(timeStep_));

    // Check if velocity is kept (fall back strategy)
    ASSERT_TRUE(std::abs(result.velocity[0] -
        sut_->getTaskTrajectoryPoint().get().velocity[0]) < 1e-3);
}

TEST_F(ReflexxesTrajectoryPointGeneratorTestsShould,
        returnLatestComputedTrajectoryPointWhenNotUpdated) {
    TaskVelocity targetVelocity({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    ASSERT_TRUE(sut_->updateVelocityTarget(targetVelocity));
    TaskTrajectoryData result;
    for (int i = 0; i < 30; i++) {
        if (i > 0) {
            ASSERT_TRUE(sut_->updateCurrentState(result));
        }
        result = sut_->getTaskTrajectoryPoint().get();
    }
    std::this_thread::sleep_for(std::chrono::duration<float>(timeStep_));

    // Check if old value is returned if no update for some time
    auto newResult = sut_->getTaskTrajectoryPoint().get();
    ASSERT_NEAR(result.velocity[0], newResult.velocity[0], 1e-3);
}


TEST_F(ReflexxesTrajectoryPointGeneratorTestsShould,
        notStopIfDestinationReachedIfTargetVelocityDefined) {
    TaskTrajectoryData data;
    data.pose = TaskPose({0.5, 0.5, 0.5},
        crf::math::rotation::CardanXYZ({0.5, 0.5, 0.5}));
    data.velocity = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

    std::vector<bool> selectedDim = {true, false, false, false, false, false};
    sut_.reset(new ReflexxesTrajectoryPointGenerator(ControlMode::POSITION, selectedDim,
                timeStep_));
    ASSERT_EQ(sut_->getControlMode(), ControlMode::POSITION);

    setDefaultMotionConstraints();

    ASSERT_TRUE(sut_->updatePositionTarget(data.pose));
    ASSERT_TRUE(sut_->updateVelocityTarget(data.velocity));

    TaskTrajectoryData result;
    while (std::abs(data.pose.getPosition()(0) - result.pose.getPosition()(0)) > 1e-3) {
        result = sut_->getTaskTrajectoryPoint().get();
        ASSERT_TRUE(sut_->updateCurrentState(result));
        std::this_thread::sleep_for(std::chrono::duration<float>(timeStep_));
    }
    ASSERT_NEAR(data.velocity[0], result.velocity[0], 1e-1);
    ASSERT_NEAR(data.pose.getPosition()(0), result.pose.getPosition()(0), 1e-1);
    // Make sure not selected parts are not computed
    ASSERT_TRUE(std::isnan(result.pose.getPosition()(2)));
}
