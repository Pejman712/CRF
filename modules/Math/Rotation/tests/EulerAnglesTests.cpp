/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/EulerAngles.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::math::rotation::EulerAngles;

class EulerAnglesShould : public ::testing::Test {
 protected:
    EulerAnglesShould() :
        sut_(),
        defaultConstructorArray_({0.0, 0.0, 0.0}),
        eulerAnglesArray1_({2.2, 3.8, 4.7}),
        eulerAnglesArray2_({1.2, 3.14, 2.8}),
        eulerAnglesArray3_({-1.2452654789023454, 3.1487283479344533, 152.8456356344354765}),
        eulerAnglesInitializerListWrongLength_({2.2, 3.8, 4.7, 5.5}),
        outputDouble_(0),
        outputDoubleArray_({0.0, 0.0, 0.0}),
        os_() {
    }

    void SetUp() {
        sut_.reset(new EulerAngles());
        outputDouble_ = 0;
        outputDoubleArray_ = std::array<double, 3>({0.0, 0.0, 0.0});
        os_ = std::stringstream();
    }

    std::unique_ptr<EulerAngles> sut_;

    const std::array<double, 3> defaultConstructorArray_;
    const std::array<double, 3> eulerAnglesArray1_;
    const std::array<double, 3> eulerAnglesArray2_;
    const std::array<double, 3> eulerAnglesArray3_;
    const std::initializer_list<double> eulerAnglesInitializerListWrongLength_;

    double outputDouble_;
    std::array<double, 3> outputDoubleArray_;

    std::stringstream os_;
};

TEST_F(EulerAnglesShould, HaveCorrectDefaultConstructor) {
    ASSERT_NO_THROW(sut_.reset(new EulerAngles()));
    ASSERT_EQ(sut_->rawArray(), defaultConstructorArray_);
}

TEST_F(EulerAnglesShould, HaveCorrectArrayConstructor) {
    ASSERT_NO_THROW(sut_.reset(new EulerAngles(eulerAnglesArray1_)));
    ASSERT_EQ(sut_->rawArray(), eulerAnglesArray1_);
}

TEST_F(EulerAnglesShould, HaveCorrectInitializerListConstructor) {
    ASSERT_NO_THROW(sut_.reset(new EulerAngles({2.2, 3.8, 4.7})));
    ASSERT_EQ(sut_->rawArray(), eulerAnglesArray1_);
    ASSERT_THROW(sut_.reset(new EulerAngles({2.2, 3.8, 4.7, 5.5})), std::invalid_argument);
}

TEST_F(EulerAnglesShould, HaveCorrectArrayAssingmentoperator) {
    ASSERT_NO_THROW(*sut_ = eulerAnglesArray1_);
    ASSERT_EQ(sut_->rawArray(), eulerAnglesArray1_);
    ASSERT_NO_THROW(*sut_ = eulerAnglesArray1_);
    ASSERT_NO_THROW(*sut_ = eulerAnglesArray2_);
    ASSERT_EQ(sut_->rawArray(), eulerAnglesArray2_);
}

TEST_F(EulerAnglesShould, HaveCorrectInitializerListAssingmentoperator) {
    *sut_ = {2.2, 3.8, 4.7};
    ASSERT_EQ(sut_->rawArray(), eulerAnglesArray1_);
    *sut_ = {2.2, 3.8, 4.7};
    *sut_ = {1.2, 3.14, 2.8};
    ASSERT_EQ(sut_->rawArray(), eulerAnglesArray2_);
    ASSERT_THROW(*sut_ = eulerAnglesInitializerListWrongLength_, std::invalid_argument);
}

TEST_F(EulerAnglesShould, HaveCorrectNonConstIndexOperator) {
    ASSERT_NO_THROW((*sut_)[0] = 2.2);
    ASSERT_NO_THROW((*sut_)[1] = 3.8);
    ASSERT_NO_THROW((*sut_)[2] = 4.7);
    ASSERT_THROW((*sut_)[-1] = 2.2, std::out_of_range);
    ASSERT_THROW((*sut_)[3] = 3.8, std::out_of_range);
    ASSERT_THROW((*sut_)[-8] = 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[10] = 4.7, std::out_of_range);
    ASSERT_EQ(sut_->rawArray(), eulerAnglesArray1_);
}

TEST_F(EulerAnglesShould, HaveCorrectConstIndexOperator) {
    ASSERT_NO_THROW(sut_.reset(new EulerAngles(eulerAnglesArray1_)));
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[0]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[1]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[2]);
    ASSERT_EQ((*sut_)[0], 2.2);
    ASSERT_EQ((*sut_)[1], 3.8);
    ASSERT_EQ((*sut_)[2], 4.7);
    ASSERT_THROW((*sut_)[-1] == 2.2, std::out_of_range);
    ASSERT_THROW((*sut_)[3] == 3.8, std::out_of_range);
    ASSERT_THROW((*sut_)[-8] == 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[10] == 4.7, std::out_of_range);
}

TEST_F(EulerAnglesShould, HaveCorrectSizeOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 3);
    ASSERT_NO_THROW(sut_.reset(new EulerAngles(eulerAnglesArray1_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 3);
}

TEST_F(EulerAnglesShould, HaveCorrectRawArrayOperator) {
    ASSERT_NO_THROW(outputDoubleArray_ = sut_->rawArray());
    ASSERT_EQ(sut_->rawArray(), defaultConstructorArray_);
    ASSERT_NO_THROW(sut_.reset(new EulerAngles(eulerAnglesArray1_)));
    ASSERT_NO_THROW(outputDoubleArray_ = sut_->rawArray());
    ASSERT_EQ(sut_->rawArray(), eulerAnglesArray1_);
    ASSERT_NO_THROW(*sut_ = eulerAnglesArray1_);
    ASSERT_NO_THROW(*sut_ = eulerAnglesArray2_);
    ASSERT_NO_THROW(outputDoubleArray_ = sut_->rawArray());
    ASSERT_EQ(sut_->rawArray(), eulerAnglesArray2_);
    *sut_ = {2.2, 3.8, 4.7};
    *sut_ = {1.2, 3.14, 2.8};
    ASSERT_NO_THROW(outputDoubleArray_ = sut_->rawArray());
    ASSERT_EQ(sut_->rawArray(), eulerAnglesArray2_);
}

TEST_F(EulerAnglesShould, HaveCorrectStreamOperator) {
    ASSERT_NO_THROW(sut_.reset(new EulerAngles(eulerAnglesArray1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "2.200000000000000\n"
        "3.800000000000000\n"
        "4.700000000000000");
    os_ = std::stringstream();
    ASSERT_NO_THROW(sut_.reset(new EulerAngles(eulerAnglesArray3_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        " -1.245265478902345\n"
        "  3.148728347934453\n"
        "152.845635634435467");
}
