/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Conversions.hpp"
#include "Types/TaskTypes/TaskAcceleration.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::TaskAcceleration;
using crf::utility::types::eigenVectorFromStdArray;

class TaskAccelerationShould : public ::testing::Test {
 protected:
    TaskAccelerationShould() :
        sut_(),
        defaultConstructorVector_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
        taskAcceleration1_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        taskAcceleration2_({1.2, 3.14, 2.8, 2.2, 5.7, 3.2}),
        taskAcceleration3_(
            {-1.2452654789023454,
             3.1487283479344533,
             2.8456356344354765,
             4.0,
             23.134142312313420,
             0.0001231242}),
        taskAccelerationArray1_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        taskAccelerationArray2_({1.2, 3.14, 2.8, 2.2, 5.7, 3.2}),
        taskAccelerationArray3_(
            {-1.2452654789023454,
             3.1487283479344533,
             2.8456356344354765,
             4.0,
             23.134142312313420,
             0.0001231242}),
        taskAccelerationInitializerListWrongLength_({2.2, -3.8, 4.7, 5.5}),
        outputDouble_(0),
        outputTaskAcceleration_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
        os_() {
    }

    void SetUp() {
        sut_.reset(new TaskAcceleration());
        outputDouble_ = 0;
        outputTaskAcceleration_ = Eigen::Vector<double, 6>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        os_ = std::stringstream();
    }

    std::unique_ptr<TaskAcceleration> sut_;

    const Eigen::Vector<double, 6> defaultConstructorVector_;
    const Eigen::Vector<double, 6> taskAcceleration1_;
    const Eigen::Vector<double, 6> taskAcceleration2_;
    const Eigen::Vector<double, 6> taskAcceleration3_;
    const std::array<double, 6> taskAccelerationArray1_;
    const std::array<double, 6> taskAccelerationArray2_;
    const std::array<double, 6> taskAccelerationArray3_;
    const std::initializer_list<double> taskAccelerationInitializerListWrongLength_;

    double outputDouble_;
    Eigen::Vector<double, 6> outputTaskAcceleration_;

    std::stringstream os_;
};

TEST_F(TaskAccelerationShould, HaveCorrectDefaultConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskAcceleration()));
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
}

TEST_F(TaskAccelerationShould, HaveCorrectEigenVectorConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskAcceleration(taskAcceleration1_)));
    ASSERT_EQ(sut_->raw(), taskAcceleration1_);
}

TEST_F(TaskAccelerationShould, HaveCorrectArrayConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskAcceleration(taskAccelerationArray1_)));
    ASSERT_EQ(sut_->raw(), taskAcceleration1_);
}

TEST_F(TaskAccelerationShould, HaveCorrectInitializerListConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskAcceleration({2.2, -3.8, 4.7, 5.8, 4.7, 3.9})));
    ASSERT_EQ(sut_->raw(), taskAcceleration1_);
    ASSERT_THROW(sut_.reset(new TaskAcceleration({2.2, -3.8, 4.7, 5.5})), std::invalid_argument);
}

TEST_F(TaskAccelerationShould, HaveCorrectArrayAssingmentoperator) {
    ASSERT_NO_THROW(*sut_ = taskAcceleration1_);
    ASSERT_EQ(sut_->raw(), taskAcceleration1_);
    ASSERT_NO_THROW(*sut_ = taskAcceleration1_);
    ASSERT_NO_THROW(*sut_ = taskAcceleration2_);
    ASSERT_EQ(sut_->raw(), taskAcceleration2_);
}

TEST_F(TaskAccelerationShould, HaveCorrectStdArrayAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = taskAccelerationArray1_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdArray<6>(taskAccelerationArray1_));
    ASSERT_NO_THROW(*sut_ = taskAccelerationArray1_);
    ASSERT_NO_THROW(*sut_ = taskAccelerationArray2_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdArray<6>(taskAccelerationArray2_));
}

TEST_F(TaskAccelerationShould, HaveCorrectInitializerListAssingmentoperator) {
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    ASSERT_EQ(sut_->raw(), taskAcceleration1_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_EQ(sut_->raw(), taskAcceleration2_);
    ASSERT_THROW(*sut_ = taskAccelerationInitializerListWrongLength_, std::invalid_argument);
}

TEST_F(TaskAccelerationShould, HaveCorrectSizeOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
    ASSERT_NO_THROW(sut_.reset(new TaskAcceleration(taskAcceleration1_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
}

TEST_F(TaskAccelerationShould, HaveCorrectRawOperator) {
    ASSERT_NO_THROW(outputTaskAcceleration_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
    ASSERT_NO_THROW(sut_.reset(new TaskAcceleration(taskAcceleration1_)));
    ASSERT_NO_THROW(outputTaskAcceleration_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), taskAcceleration1_);
    ASSERT_NO_THROW(*sut_ = taskAcceleration1_);
    ASSERT_NO_THROW(*sut_ = taskAcceleration2_);
    ASSERT_NO_THROW(outputTaskAcceleration_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), taskAcceleration2_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_NO_THROW(outputTaskAcceleration_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), taskAcceleration2_);
}

TEST_F(TaskAccelerationShould, HaveCorrectNonConstIndexOperator) {
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
    ASSERT_EQ(sut_->raw(), taskAcceleration1_);
}

TEST_F(TaskAccelerationShould, HaveCorrectConstIndexOperator) {
    ASSERT_NO_THROW(sut_.reset(new TaskAcceleration(taskAcceleration1_)));
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

TEST_F(TaskAccelerationShould, HaveCorrectStreamOperator) {
    ASSERT_NO_THROW(sut_.reset(new TaskAcceleration(taskAcceleration1_)));
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
    ASSERT_NO_THROW(sut_.reset(new TaskAcceleration(taskAcceleration3_)));
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
