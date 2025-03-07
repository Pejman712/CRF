/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Comparison.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::areAlmostEqual;
using crf::utility::types::isLesser;
using crf::utility::types::isGreater;
using crf::utility::types::isBetween;

using crf::utility::types::TaskPose;
using crf::utility::types::Vector6d;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::TaskSpace;

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;

class ReducedTaskSpaceComparisonShould : public ::testing::Test {
 protected:
    ReducedTaskSpaceComparisonShould() :
        position1_({-2.8, -3.7, 15.12332332343423}),
        position2_({-2.8, -4.814231423423, 15.12332332343423}),
        orientation1_(CardanXYZ({2.7143908348394090, 0.5174722288733832, 1.7853074093465211})),
        orientation2_(CardanXYZ({2.7143908348394090, 2.0, 1.7853074093465211})),
        orientation3_(CardanXYZ({1.4, 2.0, 1.7853074093465211})),
        orientation4_(CardanXYZ({1.4, 2.0, 3.1})),
        taskPose1_(position1_, orientation1_),
        taskPose2_(position2_, orientation2_),
        taskPose3_(position2_, orientation3_),
        taskPose4_(position2_, orientation4_),
        stdArray1_(
            {-2.55417,
             -42.5764243346,
             -std::numeric_limits<double>::infinity(),
             -8.16,
             -std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity()}),
        stdArray2_(
            {15.4,
             std::numeric_limits<double>::quiet_NaN(),
             -std::numeric_limits<double>::infinity(),
             -3.16,
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity()}),
        stdArray3_(
            {21.0,
             -std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::infinity(),
             23.2000001,
             882.24,
             std::numeric_limits<double>::infinity()}),
        stdArray4_(
            {-2.55417,
             42.5764243346,
             -std::numeric_limits<double>::infinity(),
             -8.16,
             8.0,
             -std::numeric_limits<double>::infinity()}),
        vector6d1_(stdArray1_),
        vector6d2_(stdArray2_),
        vector6d3_(stdArray3_),
        vector6d4_(stdArray4_),
        taskVelocity1_(stdArray1_),
        taskVelocity2_(stdArray2_),
        taskVelocity3_(stdArray3_),
        taskVelocity4_(stdArray4_),
        taskAcceleration1_(stdArray1_),
        taskAcceleration2_(stdArray2_),
        taskAcceleration3_(stdArray3_),
        taskAcceleration4_(stdArray4_),
        taskForceTorque1_(stdArray1_),
        taskForceTorque2_(stdArray2_),
        taskForceTorque3_(stdArray3_),
        taskForceTorque4_(stdArray4_),
        taskSpace1_({true, true, true, true, true, true}),
        taskSpace2_({true, false, true, true, true, true}),
        taskSpace3_({true, false, true, true, false, true}),
        taskSpace4_({true, false, true, false, false, true}),
        taskSpace5_({true, false, true, false, false, false}) {
    }

    const Eigen::Vector3d position1_;
    const Eigen::Vector3d position2_;

    const Orientation orientation1_;
    const Orientation orientation2_;
    const Orientation orientation3_;
    const Orientation orientation4_;

    const TaskPose taskPose1_;
    const TaskPose taskPose2_;
    const TaskPose taskPose3_;
    const TaskPose taskPose4_;

    const std::array<double, 6> stdArray1_;
    const std::array<double, 6> stdArray2_;
    const std::array<double, 6> stdArray3_;
    const std::array<double, 6> stdArray4_;

    const Vector6d vector6d1_;
    const Vector6d vector6d2_;
    const Vector6d vector6d3_;
    const Vector6d vector6d4_;

    const TaskVelocity taskVelocity1_;
    const TaskVelocity taskVelocity2_;
    const TaskVelocity taskVelocity3_;
    const TaskVelocity taskVelocity4_;

    const TaskAcceleration taskAcceleration1_;
    const TaskAcceleration taskAcceleration2_;
    const TaskAcceleration taskAcceleration3_;
    const TaskAcceleration taskAcceleration4_;

    const TaskForceTorque taskForceTorque1_;
    const TaskForceTorque taskForceTorque2_;
    const TaskForceTorque taskForceTorque3_;
    const TaskForceTorque taskForceTorque4_;

    const TaskSpace taskSpace1_;
    const TaskSpace taskSpace2_;
    const TaskSpace taskSpace3_;
    const TaskSpace taskSpace4_;
    const TaskSpace taskSpace5_;
};

TEST_F(ReducedTaskSpaceComparisonShould, ReturnAreAlmostEqualTrueForTheSameTaskPoseArguments) {
    ASSERT_TRUE(areAlmostEqual(taskPose1_, taskPose3_, 1e-15, taskSpace4_));
    ASSERT_TRUE(areAlmostEqual(taskPose1_, taskPose4_, 1e-15, taskSpace5_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnAreAlmostEqualFalseForDifferentTaskPoseArguments) {
    ASSERT_FALSE(areAlmostEqual(taskPose1_, taskPose2_, 1e-15, taskSpace3_));
    ASSERT_FALSE(areAlmostEqual(taskPose1_, taskPose3_, 1e-15, taskSpace3_));
    ASSERT_FALSE(areAlmostEqual(taskPose1_, taskPose4_, 1e-15, taskSpace4_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnAreAlmostEqualTrueForTheSameVector6dArguments) {
    ASSERT_TRUE(areAlmostEqual(vector6d1_, vector6d4_, 1e-15, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnAreAlmostEqualFalseForDifferentVector6dArguments) {
    ASSERT_FALSE(areAlmostEqual(vector6d1_, vector6d2_, 1e-15, taskSpace3_));
    ASSERT_FALSE(areAlmostEqual(vector6d2_, vector6d1_, 1e-15, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnAreAlmostEqualTrueForTheSameTaskVelocityArguments) {
    ASSERT_TRUE(areAlmostEqual(taskVelocity1_, taskVelocity4_, 1e-15, taskSpace3_));
}

TEST_F(
    ReducedTaskSpaceComparisonShould,
    ReturnAreAlmostEqualFalseForDifferentTaskVelocityArguments) {
    ASSERT_FALSE(areAlmostEqual(taskVelocity1_, taskVelocity2_, 1e-15, taskSpace3_));
    ASSERT_FALSE(areAlmostEqual(taskVelocity2_, taskVelocity1_, 1e-15, taskSpace3_));
}

TEST_F(
    ReducedTaskSpaceComparisonShould,
    ReturnAreAlmostEqualTrueForTheSameTaskAccelerationArguments) {
    ASSERT_TRUE(areAlmostEqual(taskAcceleration1_, taskAcceleration4_, 1e-15, taskSpace3_));
}

TEST_F(
    ReducedTaskSpaceComparisonShould,
    ReturnAreAlmostEqualFalseForDifferentTaskAccelerationArguments) {
    ASSERT_FALSE(areAlmostEqual(taskAcceleration1_, taskAcceleration2_, 1e-15, taskSpace3_));
    ASSERT_FALSE(areAlmostEqual(taskAcceleration2_, taskAcceleration1_, 1e-15, taskSpace3_));
}

TEST_F(
    ReducedTaskSpaceComparisonShould,
    ReturnAreAlmostEqualTrueForTheSameTaskForceTorqueArguments) {
    ASSERT_TRUE(areAlmostEqual(taskForceTorque1_, taskForceTorque4_, 1e-15, taskSpace3_));
}

TEST_F(
    ReducedTaskSpaceComparisonShould,
    ReturnAreAlmostEqualFalseForDifferentTaskForceTorqueArguments) {
    ASSERT_FALSE(areAlmostEqual(taskForceTorque1_, taskForceTorque2_, 1e-15, taskSpace3_));
    ASSERT_FALSE(areAlmostEqual(taskForceTorque2_, taskForceTorque1_, 1e-15, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsBetweenTrueForOrderedVector6dArguments) {
    ASSERT_TRUE(isBetween(vector6d1_, vector6d3_, vector6d2_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsBetweenFalseForNonOrderedVector6dArguments) {
    ASSERT_FALSE(isBetween(vector6d2_, vector6d3_, vector6d1_, taskSpace3_));
    ASSERT_FALSE(isBetween(vector6d1_, vector6d2_, vector6d3_, taskSpace3_));
    ASSERT_FALSE(isBetween(vector6d3_, vector6d1_, vector6d2_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsBetweenTrueForOrderedTaskVelocityArguments) {
    ASSERT_TRUE(isBetween(taskVelocity1_, taskVelocity3_, taskVelocity2_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsBetweenFalseForNonOrderedTaskVelocityArguments) {
    ASSERT_FALSE(isBetween(taskVelocity2_, taskVelocity3_, taskVelocity1_, taskSpace3_));
    ASSERT_FALSE(isBetween(taskVelocity1_, taskVelocity2_, taskVelocity3_, taskSpace3_));
    ASSERT_FALSE(isBetween(taskVelocity3_, taskVelocity1_, taskVelocity2_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsBetweenTrueForOrderedTaskAccelerationArguments) {
    ASSERT_TRUE(isBetween(taskAcceleration1_, taskAcceleration3_, taskAcceleration2_, taskSpace3_));
}

TEST_F(
    ReducedTaskSpaceComparisonShould,
    ReturnIsBetweenFalseForNonOrderedTaskAccelerationArguments) {
    ASSERT_FALSE(
        isBetween(taskAcceleration2_, taskAcceleration3_, taskAcceleration1_, taskSpace3_));
    ASSERT_FALSE(
        isBetween(taskAcceleration1_, taskAcceleration2_, taskAcceleration3_, taskSpace3_));
    ASSERT_FALSE(
        isBetween(taskAcceleration3_, taskAcceleration1_, taskAcceleration2_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsBetweenTrueForOrderedTaskForceTorqueArguments) {
    ASSERT_TRUE(isBetween(taskForceTorque1_, taskForceTorque3_, taskForceTorque2_, taskSpace3_));
}

TEST_F(
    ReducedTaskSpaceComparisonShould,
    ReturnIsBetweenFalseForNonOrderedTaskForceTorqueArguments) {
    ASSERT_FALSE(isBetween(taskForceTorque2_, taskForceTorque3_, taskForceTorque1_, taskSpace3_));
    ASSERT_FALSE(isBetween(taskForceTorque1_, taskForceTorque2_, taskForceTorque3_, taskSpace3_));
    ASSERT_FALSE(isBetween(taskForceTorque3_, taskForceTorque1_, taskForceTorque2_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsLesserTrueForOrderedVector6dArguments) {
    ASSERT_TRUE(isLesser(vector6d1_, vector6d2_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsLesserFalseForNonOrderedVector6dArguments) {
    ASSERT_FALSE(isLesser(vector6d2_, vector6d1_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsLesserTrueForOrderedTaskVelocityArguments) {
    ASSERT_TRUE(isLesser(taskVelocity1_, taskVelocity2_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsLesserFalseForNonOrderedTaskVelocityArguments) {
    ASSERT_FALSE(isLesser(taskVelocity2_, taskVelocity1_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsLesserTrueForOrderedTaskAccelerationArguments) {
    ASSERT_TRUE(isLesser(taskAcceleration1_, taskAcceleration2_, taskSpace3_));
}

TEST_F(
    ReducedTaskSpaceComparisonShould,
    ReturnIsLesserFalseForNonOrderedTaskAccelerationArguments) {
    ASSERT_FALSE(isLesser(taskAcceleration2_, taskAcceleration1_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsLesserTrueForOrderedTaskForceTorqueArguments) {
    ASSERT_TRUE(isLesser(taskForceTorque1_, taskForceTorque2_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsLesserFalseForNonOrderedTaskForceTorqueArguments) {
    ASSERT_FALSE(isLesser(taskForceTorque2_, taskForceTorque1_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsGreaterTrueForOrderedVector6dArguments) {
    ASSERT_TRUE(isGreater(vector6d2_, vector6d1_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsGreaterFalseForNonOrderedVector6dArguments) {
    ASSERT_FALSE(isGreater(vector6d1_, vector6d2_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsGreaterTrueForOrderedTaskVelocityArguments) {
    ASSERT_TRUE(isGreater(taskVelocity2_, taskVelocity1_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsGreaterFalseForNonOrderedTaskVelocityArguments) {
    ASSERT_FALSE(isGreater(taskVelocity1_, taskVelocity2_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsGreaterTrueForOrderedTaskAccelerationArguments) {
    ASSERT_TRUE(isGreater(taskAcceleration2_, taskAcceleration1_, taskSpace3_));
}

TEST_F(
    ReducedTaskSpaceComparisonShould,
    ReturnIsGreaterFalseForNonOrderedTaskAccelerationArguments) {
    ASSERT_FALSE(isGreater(taskAcceleration1_, taskAcceleration2_, taskSpace3_));
}

TEST_F(ReducedTaskSpaceComparisonShould, ReturnIsGreaterTrueForOrderedTaskForceTorqueArguments) {
    ASSERT_TRUE(isGreater(taskForceTorque2_, taskForceTorque1_, taskSpace3_));
}

TEST_F(
    ReducedTaskSpaceComparisonShould,
    ReturnIsGreaterFalseForNonOrderedTaskForceTorqueArguments) {
    ASSERT_FALSE(isGreater(taskForceTorque1_, taskForceTorque2_, taskSpace3_));
}
