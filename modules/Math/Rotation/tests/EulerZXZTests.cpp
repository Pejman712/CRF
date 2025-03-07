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

using crf::math::rotation::EulerZXZ;

class EulerZXZShould : public ::testing::Test {
 protected:
    EulerZXZShould() :
        sut_(),
        defaultConstructorArray_({0.0, 0.0, 0.0}),
        eulerZXZArray1_({2.2, 3.8, 4.7}),
        eulerZXZArray2_({1.2, 3.14, 2.8}),
        eulerZXZArray3_({-1.2452654789023454, 3.1487283479344533, 152.8456356344354765}),
        eulerZXZInitializerListWrongLength_({2.2, 3.8, 4.7, 5.5}),
        outputDouble_(0),
        outputDoubleArray_({0.0, 0.0, 0.0}),
        os_() {
    }

    void SetUp() {
        sut_.reset(new EulerZXZ());
        outputDouble_ = 0;
        outputDoubleArray_ = std::array<double, 3>({0.0, 0.0, 0.0});
        os_ = std::stringstream();
    }

    std::unique_ptr<EulerZXZ> sut_;

    const std::array<double, 3> defaultConstructorArray_;
    const std::array<double, 3> eulerZXZArray1_;
    const std::array<double, 3> eulerZXZArray2_;
    const std::array<double, 3> eulerZXZArray3_;
    const std::initializer_list<double> eulerZXZInitializerListWrongLength_;

    double outputDouble_;
    std::array<double, 3> outputDoubleArray_;

    std::stringstream os_;
};

TEST_F(EulerZXZShould, HaveCorrectDefaultConstructor) {
    ASSERT_NO_THROW(sut_.reset(new EulerZXZ()));
    ASSERT_EQ(sut_->rawArray(), defaultConstructorArray_);
}

TEST_F(EulerZXZShould, HaveCorrectArrayConstructor) {
    ASSERT_NO_THROW(sut_.reset(new EulerZXZ(eulerZXZArray1_)));
    ASSERT_EQ(sut_->rawArray(), eulerZXZArray1_);
}

TEST_F(EulerZXZShould, HaveCorrectInitializerListConstructor) {
    ASSERT_NO_THROW(sut_.reset(new EulerZXZ({2.2, 3.8, 4.7})));
    ASSERT_EQ(sut_->rawArray(), eulerZXZArray1_);
    ASSERT_THROW(sut_.reset(new EulerZXZ({2.2, 3.8, 4.7, 5.5})), std::invalid_argument);
}

TEST_F(EulerZXZShould, HaveCorrectArrayAssingmentoperator) {
    ASSERT_NO_THROW(*sut_ = eulerZXZArray1_);
    ASSERT_EQ(sut_->rawArray(), eulerZXZArray1_);
    ASSERT_NO_THROW(*sut_ = eulerZXZArray1_);
    ASSERT_NO_THROW(*sut_ = eulerZXZArray2_);
    ASSERT_EQ(sut_->rawArray(), eulerZXZArray2_);
}

TEST_F(EulerZXZShould, HaveCorrectInitializerListAssingmentoperator) {
    *sut_ = {2.2, 3.8, 4.7};
    ASSERT_EQ(sut_->rawArray(), eulerZXZArray1_);
    *sut_ = {2.2, 3.8, 4.7};
    *sut_ = {1.2, 3.14, 2.8};
    ASSERT_EQ(sut_->rawArray(), eulerZXZArray2_);
    ASSERT_THROW(*sut_ = eulerZXZInitializerListWrongLength_, std::invalid_argument);
}

TEST_F(EulerZXZShould, HaveCorrectNonConstIndexOperator) {
    ASSERT_NO_THROW((*sut_)[0] = 2.2);
    ASSERT_NO_THROW((*sut_)[1] = 3.8);
    ASSERT_NO_THROW((*sut_)[2] = 4.7);
    ASSERT_THROW((*sut_)[-1] = 2.2, std::out_of_range);
    ASSERT_THROW((*sut_)[3] = 3.8, std::out_of_range);
    ASSERT_THROW((*sut_)[-8] = 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[10] = 4.7, std::out_of_range);
    ASSERT_EQ(sut_->rawArray(), eulerZXZArray1_);
}

TEST_F(EulerZXZShould, HaveCorrectConstIndexOperator) {
    ASSERT_NO_THROW(sut_.reset(new EulerZXZ(eulerZXZArray1_)));
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

TEST_F(EulerZXZShould, HaveCorrectSizeOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 3);
    ASSERT_NO_THROW(sut_.reset(new EulerZXZ(eulerZXZArray1_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 3);
}

TEST_F(EulerZXZShould, HaveCorrectRawArrayOperator) {
    ASSERT_NO_THROW(outputDoubleArray_ = sut_->rawArray());
    ASSERT_EQ(sut_->rawArray(), defaultConstructorArray_);
    ASSERT_NO_THROW(sut_.reset(new EulerZXZ(eulerZXZArray1_)));
    ASSERT_NO_THROW(outputDoubleArray_ = sut_->rawArray());
    ASSERT_EQ(sut_->rawArray(), eulerZXZArray1_);
    ASSERT_NO_THROW(*sut_ = eulerZXZArray1_);
    ASSERT_NO_THROW(*sut_ = eulerZXZArray2_);
    ASSERT_NO_THROW(outputDoubleArray_ = sut_->rawArray());
    ASSERT_EQ(sut_->rawArray(), eulerZXZArray2_);
    *sut_ = {2.2, 3.8, 4.7};
    *sut_ = {1.2, 3.14, 2.8};
    ASSERT_NO_THROW(outputDoubleArray_ = sut_->rawArray());
    ASSERT_EQ(sut_->rawArray(), eulerZXZArray2_);
}

TEST_F(EulerZXZShould, HaveCorrectStreamOperator) {
    ASSERT_NO_THROW(sut_.reset(new EulerZXZ(eulerZXZArray1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "Z: 2.200000000000000\n"
        "X: 3.800000000000000\n"
        "Z: 4.700000000000000");
    os_ = std::stringstream();
    ASSERT_NO_THROW(sut_.reset(new EulerZXZ(eulerZXZArray3_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "Z:  -1.245265478902345\n"
        "X:   3.148728347934453\n"
        "Z: 152.845635634435467");
}
