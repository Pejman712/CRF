/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Conversions.hpp"
#include "Types/TaskTypes/TaskVelocity.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::TaskVelocity;
using crf::utility::types::eigenVectorFromStdArray;

class TaskVelocityShould : public ::testing::Test {
 protected:
    TaskVelocityShould() :
        sut_(),
        defaultConstructorVector_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
        taskVelocity1_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        taskVelocity2_({1.2, 3.14, 2.8, 2.2, 5.7, 3.2}),
        taskVelocity3_(
            {-1.2452654789023454,
             3.1487283479344533,
             2.8456356344354765,
             4.0,
             23.134142312313420,
             0.0001231242}),
        taskVelocityArray1_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        taskVelocityArray2_({1.2, 3.14, 2.8, 2.2, 5.7, 3.2}),
        taskVelocityArray3_(
            {-1.2452654789023454,
             3.1487283479344533,
             2.8456356344354765,
             4.0,
             23.134142312313420,
             0.0001231242}),
        taskVelocityInitializerListWrongLength_({2.2, -3.8, 4.7, 5.5}),
        outputDouble_(0),
        outputTaskVelocity_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
        os_() {
    }

    void SetUp() {
        sut_.reset(new TaskVelocity());
        outputDouble_ = 0;
        outputTaskVelocity_ = Eigen::Vector<double, 6>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        os_ = std::stringstream();
    }

    std::unique_ptr<TaskVelocity> sut_;

    const Eigen::Vector<double, 6> defaultConstructorVector_;
    const Eigen::Vector<double, 6> taskVelocity1_;
    const Eigen::Vector<double, 6> taskVelocity2_;
    const Eigen::Vector<double, 6> taskVelocity3_;
    const std::array<double, 6> taskVelocityArray1_;
    const std::array<double, 6> taskVelocityArray2_;
    const std::array<double, 6> taskVelocityArray3_;
    const std::initializer_list<double> taskVelocityInitializerListWrongLength_;

    double outputDouble_;
    Eigen::Vector<double, 6> outputTaskVelocity_;

    std::stringstream os_;
};

TEST_F(TaskVelocityShould, HaveCorrectDefaultConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskVelocity()));
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
}

TEST_F(TaskVelocityShould, HaveCorrectEigenVectorConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskVelocity(taskVelocity1_)));
    ASSERT_EQ(sut_->raw(), taskVelocity1_);
}

TEST_F(TaskVelocityShould, HaveCorrectArrayConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskVelocity(taskVelocityArray1_)));
    ASSERT_EQ(sut_->raw(), taskVelocity1_);
}

TEST_F(TaskVelocityShould, HaveCorrectInitializerListConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskVelocity({2.2, -3.8, 4.7, 5.8, 4.7, 3.9})));
    ASSERT_EQ(sut_->raw(), taskVelocity1_);
    ASSERT_THROW(sut_.reset(new TaskVelocity({2.2, -3.8, 4.7, 5.5})), std::invalid_argument);
}

TEST_F(TaskVelocityShould, HaveCorrectArrayAssingmentoperator) {
    ASSERT_NO_THROW(*sut_ = taskVelocity1_);
    ASSERT_EQ(sut_->raw(), taskVelocity1_);
    ASSERT_NO_THROW(*sut_ = taskVelocity1_);
    ASSERT_NO_THROW(*sut_ = taskVelocity2_);
    ASSERT_EQ(sut_->raw(), taskVelocity2_);
}

TEST_F(TaskVelocityShould, HaveCorrectStdArrayAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = taskVelocityArray1_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdArray<6>(taskVelocityArray1_));
    ASSERT_NO_THROW(*sut_ = taskVelocityArray1_);
    ASSERT_NO_THROW(*sut_ = taskVelocityArray2_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdArray<6>(taskVelocityArray2_));
}

TEST_F(TaskVelocityShould, HaveCorrectInitializerListAssingmentoperator) {
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    ASSERT_EQ(sut_->raw(), taskVelocity1_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_EQ(sut_->raw(), taskVelocity2_);
    ASSERT_THROW(*sut_ = taskVelocityInitializerListWrongLength_, std::invalid_argument);
}

TEST_F(TaskVelocityShould, HaveCorrectSizeOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
    ASSERT_NO_THROW(sut_.reset(new TaskVelocity(taskVelocity1_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
}

TEST_F(TaskVelocityShould, HaveCorrectRawOperator) {
    ASSERT_NO_THROW(outputTaskVelocity_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
    ASSERT_NO_THROW(sut_.reset(new TaskVelocity(taskVelocity1_)));
    ASSERT_NO_THROW(outputTaskVelocity_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), taskVelocity1_);
    ASSERT_NO_THROW(*sut_ = taskVelocity1_);
    ASSERT_NO_THROW(*sut_ = taskVelocity2_);
    ASSERT_NO_THROW(outputTaskVelocity_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), taskVelocity2_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_NO_THROW(outputTaskVelocity_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), taskVelocity2_);
}

TEST_F(TaskVelocityShould, HaveCorrectNonConstIndexOperator) {
    ASSERT_NO_THROW((*sut_)[0] = 2.2);
    ASSERT_NO_THROW((*sut_)[1] = -3.8);
    ASSERT_NO_THROW((*sut_)[2] = 4.7);
    ASSERT_NO_THROW((*sut_)[3] = 5.8);
    ASSERT_NO_THROW((*sut_)[4] = 4.7);
    ASSERT_NO_THROW((*sut_)[5] = 3.9);
    ASSERT_THROW((*sut_)[-1] = 2.2, std::out_of_range);
    ASSERT_THROW((*sut_)[6] = -3.8, std::out_of_range);
    ASSERT_THROW((*sut_)[-8] = 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[10] = 4.7, std::out_of_range);
    ASSERT_EQ(sut_->raw(), taskVelocity1_);
}

TEST_F(TaskVelocityShould, HaveCorrectConstIndexOperator) {
    ASSERT_NO_THROW(sut_.reset(new TaskVelocity(taskVelocity1_)));
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[0]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[1]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[2]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[3]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[4]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[5]);
    ASSERT_EQ((*sut_)[0], 2.2);
    ASSERT_EQ((*sut_)[1], -3.8);
    ASSERT_EQ((*sut_)[2], 4.7);
    ASSERT_EQ((*sut_)[3], 5.8);
    ASSERT_EQ((*sut_)[4], 4.7);
    ASSERT_EQ((*sut_)[5], 3.9);
    ASSERT_THROW((*sut_)[-1] == 2.2, std::out_of_range);
    ASSERT_THROW((*sut_)[6] == -3.8, std::out_of_range);
    ASSERT_THROW((*sut_)[-8] == 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[10] == 4.7, std::out_of_range);
}

TEST_F(TaskVelocityShould, HaveCorrectStreamOperator) {
    ASSERT_NO_THROW(sut_.reset(new TaskVelocity(taskVelocity1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        " 2.200000000000000\n"
        "-3.800000000000000\n"
        " 4.700000000000000\n"
        " 5.800000000000000\n"
        " 4.700000000000000\n"
        " 3.900000000000000");
    os_ = std::stringstream();
    ASSERT_NO_THROW(sut_.reset(new TaskVelocity(taskVelocity3_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "-1.245265478902345\n"
        " 3.148728347934453\n"
        " 2.845635634435476\n"
        " 4.000000000000000\n"
        "23.134142312313418\n"
        " 0.000123124200000");
}
