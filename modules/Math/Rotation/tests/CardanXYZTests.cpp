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

using crf::math::rotation::CardanXYZ;

class CardanXYZShould : public ::testing::Test {
 protected:
    CardanXYZShould() :
        sut_(),
        defaultConstructorArray_({0.0, 0.0, 0.0}),
        cardanXYZArray1_({2.2, 3.8, 4.7}),
        cardanXYZArray2_({1.2, 3.14, 2.8}),
        cardanXYZArray3_({-1.2452654789023454, 3.1487283479344533, 152.8456356344354765}),
        cardanXYZInitializerListWrongLength_({2.2, 3.8, 4.7, 5.5}),
        outputDouble_(0),
        outputDoubleArray_({0.0, 0.0, 0.0}),
        os_() {
    }

    void SetUp() {
        sut_.reset(new CardanXYZ());
        outputDouble_ = 0;
        outputDoubleArray_ = std::array<double, 3>({0.0, 0.0, 0.0});
        os_ = std::stringstream();
    }

    std::unique_ptr<CardanXYZ> sut_;

    const std::array<double, 3> defaultConstructorArray_;
    const std::array<double, 3> cardanXYZArray1_;
    const std::array<double, 3> cardanXYZArray2_;
    const std::array<double, 3> cardanXYZArray3_;
    const std::initializer_list<double> cardanXYZInitializerListWrongLength_;

    double outputDouble_;
    std::array<double, 3> outputDoubleArray_;

    std::stringstream os_;
};

TEST_F(CardanXYZShould, HaveCorrectDefaultConstructor) {
    ASSERT_NO_THROW(sut_.reset(new CardanXYZ()));
    ASSERT_EQ(sut_->rawArray(), defaultConstructorArray_);
}

TEST_F(CardanXYZShould, HaveCorrectArrayConstructor) {
    ASSERT_NO_THROW(sut_.reset(new CardanXYZ(cardanXYZArray1_)));
    ASSERT_EQ(sut_->rawArray(), cardanXYZArray1_);
}

TEST_F(CardanXYZShould, HaveCorrectInitializerListConstructor) {
    ASSERT_NO_THROW(sut_.reset(new CardanXYZ({2.2, 3.8, 4.7})));
    ASSERT_EQ(sut_->rawArray(), cardanXYZArray1_);
    ASSERT_THROW(sut_.reset(new CardanXYZ({2.2, 3.8, 4.7, 5.5})), std::invalid_argument);
}

TEST_F(CardanXYZShould, HaveCorrectArrayAssingmentoperator) {
    ASSERT_NO_THROW(*sut_ = cardanXYZArray1_);
    ASSERT_EQ(sut_->rawArray(), cardanXYZArray1_);
    ASSERT_NO_THROW(*sut_ = cardanXYZArray1_);
    ASSERT_NO_THROW(*sut_ = cardanXYZArray2_);
    ASSERT_EQ(sut_->rawArray(), cardanXYZArray2_);
}

TEST_F(CardanXYZShould, HaveCorrectInitializerListAssingmentoperator) {
    *sut_ = {2.2, 3.8, 4.7};
    ASSERT_EQ(sut_->rawArray(), cardanXYZArray1_);
    *sut_ = {2.2, 3.8, 4.7};
    *sut_ = {1.2, 3.14, 2.8};
    ASSERT_EQ(sut_->rawArray(), cardanXYZArray2_);
    ASSERT_THROW(*sut_ = cardanXYZInitializerListWrongLength_, std::invalid_argument);
}

TEST_F(CardanXYZShould, HaveCorrectNonConstIndexOperator) {
    ASSERT_NO_THROW((*sut_)[0] = 2.2);
    ASSERT_NO_THROW((*sut_)[1] = 3.8);
    ASSERT_NO_THROW((*sut_)[2] = 4.7);
    ASSERT_THROW((*sut_)[-1] = 2.2, std::out_of_range);
    ASSERT_THROW((*sut_)[3] = 3.8, std::out_of_range);
    ASSERT_THROW((*sut_)[-8] = 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[10] = 4.7, std::out_of_range);
    ASSERT_EQ(sut_->rawArray(), cardanXYZArray1_);
}

TEST_F(CardanXYZShould, HaveCorrectConstIndexOperator) {
    ASSERT_NO_THROW(sut_.reset(new CardanXYZ(cardanXYZArray1_)));
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

TEST_F(CardanXYZShould, HaveCorrectSizeOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 3);
    ASSERT_NO_THROW(sut_.reset(new CardanXYZ(cardanXYZArray1_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 3);
}

TEST_F(CardanXYZShould, HaveCorrectRawArrayOperator) {
    ASSERT_NO_THROW(outputDoubleArray_ = sut_->rawArray());
    ASSERT_EQ(sut_->rawArray(), defaultConstructorArray_);
    ASSERT_NO_THROW(sut_.reset(new CardanXYZ(cardanXYZArray1_)));
    ASSERT_NO_THROW(outputDoubleArray_ = sut_->rawArray());
    ASSERT_EQ(sut_->rawArray(), cardanXYZArray1_);
    ASSERT_NO_THROW(*sut_ = cardanXYZArray1_);
    ASSERT_NO_THROW(*sut_ = cardanXYZArray2_);
    ASSERT_NO_THROW(outputDoubleArray_ = sut_->rawArray());
    ASSERT_EQ(sut_->rawArray(), cardanXYZArray2_);
    *sut_ = {2.2, 3.8, 4.7};
    *sut_ = {1.2, 3.14, 2.8};
    ASSERT_NO_THROW(outputDoubleArray_ = sut_->rawArray());
    ASSERT_EQ(sut_->rawArray(), cardanXYZArray2_);
}

TEST_F(CardanXYZShould, HaveCorrectStreamOperator) {
    ASSERT_NO_THROW(sut_.reset(new CardanXYZ(cardanXYZArray1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "X: 2.200000000000000\n"
        "Y: 3.800000000000000\n"
        "Z: 4.700000000000000");
    os_ = std::stringstream();
    ASSERT_NO_THROW(sut_.reset(new CardanXYZ(cardanXYZArray3_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "X:  -1.245265478902345\n"
        "Y:   3.148728347934453\n"
        "Z: 152.845635634435467");
}
