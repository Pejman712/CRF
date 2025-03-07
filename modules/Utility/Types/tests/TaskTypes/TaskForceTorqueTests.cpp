/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Conversions.hpp"
#include "Types/TaskTypes/TaskForceTorque.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::TaskForceTorque;
using crf::utility::types::eigenVectorFromStdArray;

class TaskForceTorqueShould : public ::testing::Test {
 protected:
    TaskForceTorqueShould() :
        sut_(),
        defaultConstructorVector_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
        taskForceTorque1_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        taskForceTorque2_({1.2, 3.14, 2.8, 2.2, 5.7, 3.2}),
        taskForceTorque3_(
            {-1.2452654789023454,
             3.1487283479344533,
             2.8456356344354765,
             4.0,
             23.134142312313420,
             0.0001231242}),
        taskForceTorqueArray1_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        taskForceTorqueArray2_({1.2, 3.14, 2.8, 2.2, 5.7, 3.2}),
        taskForceTorqueArray3_(
            {-1.2452654789023454,
             3.1487283479344533,
             2.8456356344354765,
             4.0,
             23.134142312313420,
             0.0001231242}),
        taskForceTorqueInitializerListWrongLength_({2.2, -3.8, 4.7, 5.5}),
        outputDouble_(0),
        outputTaskForceTorque_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
        os_() {
    }

    void SetUp() {
        sut_.reset(new TaskForceTorque());
        outputDouble_ = 0;
        outputTaskForceTorque_ = Eigen::Vector<double, 6>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        os_ = std::stringstream();
    }

    std::unique_ptr<TaskForceTorque> sut_;

    const Eigen::Vector<double, 6> defaultConstructorVector_;
    const Eigen::Vector<double, 6> taskForceTorque1_;
    const Eigen::Vector<double, 6> taskForceTorque2_;
    const Eigen::Vector<double, 6> taskForceTorque3_;
    const std::array<double, 6> taskForceTorqueArray1_;
    const std::array<double, 6> taskForceTorqueArray2_;
    const std::array<double, 6> taskForceTorqueArray3_;
    const std::initializer_list<double> taskForceTorqueInitializerListWrongLength_;

    double outputDouble_;
    Eigen::Vector<double, 6> outputTaskForceTorque_;

    std::stringstream os_;
};

TEST_F(TaskForceTorqueShould, HaveCorrectDefaultConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskForceTorque()));
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
}

TEST_F(TaskForceTorqueShould, HaveCorrectEigenVectorConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskForceTorque(taskForceTorque1_)));
    ASSERT_EQ(sut_->raw(), taskForceTorque1_);
}

TEST_F(TaskForceTorqueShould, HaveCorrectArrayConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskForceTorque(taskForceTorqueArray1_)));
    ASSERT_EQ(sut_->raw(), taskForceTorque1_);
}

TEST_F(TaskForceTorqueShould, HaveCorrectInitializerListConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskForceTorque({2.2, -3.8, 4.7, 5.8, 4.7, 3.9})));
    ASSERT_EQ(sut_->raw(), taskForceTorque1_);
    ASSERT_THROW(sut_.reset(new TaskForceTorque({2.2, -3.8, 4.7, 5.5})), std::invalid_argument);
}

TEST_F(TaskForceTorqueShould, HaveCorrectArrayAssingmentoperator) {
    ASSERT_NO_THROW(*sut_ = taskForceTorque1_);
    ASSERT_EQ(sut_->raw(), taskForceTorque1_);
    ASSERT_NO_THROW(*sut_ = taskForceTorque1_);
    ASSERT_NO_THROW(*sut_ = taskForceTorque2_);
    ASSERT_EQ(sut_->raw(), taskForceTorque2_);
}

TEST_F(TaskForceTorqueShould, HaveCorrectStdArrayAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = taskForceTorqueArray1_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdArray<6>(taskForceTorqueArray1_));
    ASSERT_NO_THROW(*sut_ = taskForceTorqueArray1_);
    ASSERT_NO_THROW(*sut_ = taskForceTorqueArray2_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdArray<6>(taskForceTorqueArray2_));
}

TEST_F(TaskForceTorqueShould, HaveCorrectInitializerListAssingmentoperator) {
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    ASSERT_EQ(sut_->raw(), taskForceTorque1_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_EQ(sut_->raw(), taskForceTorque2_);
    ASSERT_THROW(*sut_ = taskForceTorqueInitializerListWrongLength_, std::invalid_argument);
}

TEST_F(TaskForceTorqueShould, HaveCorrectSizeOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
    ASSERT_NO_THROW(sut_.reset(new TaskForceTorque(taskForceTorque1_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
}

TEST_F(TaskForceTorqueShould, HaveCorrectRawOperator) {
    ASSERT_NO_THROW(outputTaskForceTorque_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
    ASSERT_NO_THROW(sut_.reset(new TaskForceTorque(taskForceTorque1_)));
    ASSERT_NO_THROW(outputTaskForceTorque_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), taskForceTorque1_);
    ASSERT_NO_THROW(*sut_ = taskForceTorque1_);
    ASSERT_NO_THROW(*sut_ = taskForceTorque2_);
    ASSERT_NO_THROW(outputTaskForceTorque_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), taskForceTorque2_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_NO_THROW(outputTaskForceTorque_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), taskForceTorque2_);
}

TEST_F(TaskForceTorqueShould, HaveCorrectNonConstIndexOperator) {
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
    ASSERT_EQ(sut_->raw(), taskForceTorque1_);
}

TEST_F(TaskForceTorqueShould, HaveCorrectConstIndexOperator) {
    ASSERT_NO_THROW(sut_.reset(new TaskForceTorque(taskForceTorque1_)));
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

TEST_F(TaskForceTorqueShould, HaveCorrectStreamOperator) {
    ASSERT_NO_THROW(sut_.reset(new TaskForceTorque(taskForceTorque1_)));
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
    ASSERT_NO_THROW(sut_.reset(new TaskForceTorque(taskForceTorque3_)));
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
