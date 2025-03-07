/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <future>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <chrono>

#include "EventLogger/EventLogger.hpp"
#include "Controller/IController.hpp"

// Include here any new controller developed
#include "Controller/PositionCtrlVelocityFF/PositionCtrlVelocityFF.hpp"
#include "Controller/DirectOpenLoopVelocity/DirectOpenLoopVelocity.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::control::controller::IController;
using crf::control::controller::PositionCtrlVelocityFF;
using crf::control::controller::DirectOpenLoopVelocity;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;

class IControllerShould: public ::testing::TestWithParam<std::shared_ptr<IController>> {
 protected:
    IControllerShould() :
        logger_("IControllerShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    void SetUp() override {
        sut_ = GetParam();
    }

    void TearDown() override {
    }

    ~IControllerShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::types::JointSignals jointFeedback_;
    crf::utility::types::TaskSignals taskFeedback_;
    crf::utility::types::JointSignals jointReference_;

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<IController> sut_;

    const std::chrono::milliseconds shortestControlLoop_ = std::chrono::milliseconds(2);
};

// Create any new controller as an instantiate test and add it underneath
INSTANTIATE_TEST_CASE_P(PositionCtrlVelocityFF, IControllerShould,
    ::testing::Values(
        std::make_shared<PositionCtrlVelocityFF>(
            std::vector<double>({1, 1, 1, 1}),
            std::vector<double>({0.1, 0.1, 0.1, 0.1}),
            std::vector<double>({0.01, 0.01, 0.01, 0.01}),
            2.0,
            nullptr)
));
INSTANTIATE_TEST_CASE_P(DirectOpenLoopVelocity, IControllerShould,
    ::testing::Values(std::make_shared<DirectOpenLoopVelocity>(4 , nullptr)
));

TEST_P(IControllerShould, ThrowRuntimeExceptionIfTheDimensionsDoNotMatch) {
    // Input data has dimension 5
    jointFeedback_.positions = JointPositions({2, 2, 2, 2, 2});
    jointFeedback_.velocities = JointVelocities({0.1, 0.1, 0.1, 0.1, 0.1});
    jointFeedback_.accelerations = JointAccelerations({0.1, 0.1, 0.1, 0.1, 0.1});
    jointFeedback_.forceTorques = JointForceTorques({0.1, 0.1, 0.1, 0.1, 0.1});

    taskFeedback_.pose = TaskPose(Eigen::Vector3d({2, 2, 2}), CardanXYZ({2, 2, 2}));
    taskFeedback_.velocity = TaskVelocity({0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    taskFeedback_.acceleration = TaskAcceleration({0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    taskFeedback_.forceTorque = TaskForceTorque({0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

    // Feed-Back data has dimension 4
    jointReference_.positions = JointPositions({1, 1, 1, 1});
    jointReference_.velocities = JointVelocities({1, 1, 1, 1});
    jointReference_.accelerations = JointAccelerations({1, 1, 1, 1});
    jointReference_.forceTorques = JointForceTorques({1, 1, 1, 1});

    // Depending in the controller it might throw in the second run
    try {
        sut_->calculate(jointReference_, jointFeedback_, taskFeedback_);
    } catch (std::exception& e) {}

    ASSERT_THROW(sut_->calculate(jointReference_, jointFeedback_, taskFeedback_),
        std::runtime_error);
}

TEST_P(IControllerShould, RunInLessThanTheMostStrictRobotControlLoop) {
    jointFeedback_.positions = JointPositions({2, 2, 2, 2});
    jointFeedback_.velocities = JointVelocities({0.1, 0.1, 0.1, 0.1});
    jointFeedback_.accelerations = JointAccelerations({0.01, 0.01, 0.01, 0.01});
    jointFeedback_.forceTorques = JointForceTorques({0.001, 0.001, 0.001, 0.001});

    jointReference_.positions = JointPositions({2, 2, 2, 2});
    jointReference_.velocities = JointVelocities({0.1, 0.1, 0.1, 0.1});
    jointReference_.accelerations = JointAccelerations({0.01, 0.01, 0.01, 0.01});
    jointReference_.forceTorques = JointForceTorques({0.001, 0.001, 0.001, 0.001});

    std::chrono::high_resolution_clock::time_point start =
        std::chrono::high_resolution_clock::now();

    sut_->calculate(jointReference_, jointFeedback_, taskFeedback_);

    std::chrono::high_resolution_clock::time_point end =
        std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> elapsed = end - start;
    logger_->info("Loop Duration is {} ms", elapsed.count());
    ASSERT_LT(elapsed.count(), shortestControlLoop_.count());
}

TEST_P(IControllerShould, ThrowExceptionIfAllInputsAreEmptyButExpectedFilled) {
    jointFeedback_.positions = JointPositions({0, 0, 0, 0});
    jointFeedback_.velocities = JointVelocities({0, 0, 0, 0});
    jointFeedback_.accelerations = JointAccelerations({0, 0, 0, 0});
    jointFeedback_.forceTorques = JointForceTorques({0, 0, 0, 0});

    jointReference_.positions = JointPositions(1);
    jointReference_.velocities = JointVelocities(1);
    jointReference_.accelerations = JointAccelerations(1);
    jointReference_.forceTorques = JointForceTorques(1);

    ASSERT_THROW(sut_->calculate(jointReference_, jointFeedback_, taskFeedback_),
        std::runtime_error);
}

TEST_P(IControllerShould, ThrowExceptionIfAllInputsAreEmpty) {
    jointFeedback_.positions = JointPositions({0, 0, 0, 0});
    jointFeedback_.velocities = JointVelocities({0, 0, 0, 0});
    jointFeedback_.accelerations = JointAccelerations({0, 0, 0, 0});
    jointFeedback_.forceTorques = JointForceTorques({0, 0, 0, 0});

    jointReference_.positions = JointPositions(1);
    jointReference_.velocities = JointVelocities(1);
    jointReference_.accelerations = JointAccelerations(1);
    jointReference_.forceTorques = JointForceTorques(1);

    ASSERT_THROW(sut_->calculate(jointReference_, jointFeedback_, taskFeedback_),
        std::runtime_error);
}
