/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Conversions.hpp"
#include "Types/JointTypes/VectorXd.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::VectorXd;
using crf::utility::types::eigenVectorFromStdVector;

class VectorXdShould : public ::testing::Test {
 protected:
    VectorXdShould() :
        sut_(),
        defaultConstructorVector_(0),
        vectorXd1_(6),
        vectorXd2_(6),
        vectorXd3_(7),
        vectorXdVector1_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        vectorXdVector2_({1.2, 3.14, 2.8, 2.2, 5.7, 3.2}),
        vectorXdVector3_(
            {-1.2452654789023454,
             3.1487283479344533,
             2.8456356344354765,
             4.0,
             23.134142312313420,
             0.0001231242,
             5.22784}),
        vectorXdInitializerListDifferentLength1_({2.2, -3.8, 4.7, 5.5}),
        vectorXdInitializerListDifferentLength2_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        outputDouble_(0),
        outputVectorXd_(),
        os_() {
        vectorXd1_ << 2.2, -3.8, 4.7, 5.8, 4.7, 3.9;
        vectorXd2_ << 1.2, 3.14, 2.8, 2.2, 5.7, 3.2;
        vectorXd3_ << -1.2452654789023454, 3.1487283479344533, 2.8456356344354765, 4.0,
            23.134142312313420, 0.0001231242, 5.22784;
    }

    void SetUp() {
        sut_.reset(new VectorXd());
        outputDouble_ = 0;
        outputVectorXd_ = Eigen::VectorXd(0);
        os_ = std::stringstream();
    }

    std::unique_ptr<VectorXd> sut_;

    const Eigen::VectorXd defaultConstructorVector_;
    Eigen::VectorXd vectorXd1_;
    Eigen::VectorXd vectorXd2_;
    Eigen::VectorXd vectorXd3_;
    const std::vector<double> vectorXdVector1_;
    const std::vector<double> vectorXdVector2_;
    const std::vector<double> vectorXdVector3_;
    const std::initializer_list<double> vectorXdInitializerListDifferentLength1_;
    const std::initializer_list<double> vectorXdInitializerListDifferentLength2_;

    double outputDouble_;
    Eigen::VectorXd outputVectorXd_;

    std::stringstream os_;
};

TEST_F(VectorXdShould, HaveCorrectDefaultConstructor) {
    ASSERT_NO_THROW(sut_.reset(new VectorXd()));
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
}

TEST_F(VectorXdShould, HaveCorrectEigenVectorConstructor) {
    ASSERT_NO_THROW(sut_.reset(new VectorXd(vectorXd1_)));
    ASSERT_EQ(sut_->raw(), vectorXd1_);
}

TEST_F(VectorXdShould, HaveCorrectVectorConstructor) {
    ASSERT_NO_THROW(sut_.reset(new VectorXd(vectorXdVector1_)));
    ASSERT_EQ(sut_->raw(), vectorXd1_);
}

TEST_F(VectorXdShould, HaveCorrectInitializerListConstructor) {
    ASSERT_NO_THROW(sut_.reset(new VectorXd({2.2, -3.8, 4.7, 5.8, 4.7, 3.9})));
    ASSERT_EQ(sut_->raw(), vectorXd1_);
}

TEST_F(VectorXdShould, HaveCorrectVectorAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = vectorXd1_);
    ASSERT_NO_THROW(sut_.reset(new VectorXd(vectorXd2_)));
    ASSERT_NO_THROW(*sut_ = vectorXd1_);
    ASSERT_EQ(sut_->raw(), vectorXd1_);
    ASSERT_NO_THROW(*sut_ = vectorXd1_);
    ASSERT_NO_THROW(*sut_ = vectorXd2_);
    ASSERT_EQ(sut_->raw(), vectorXd2_);
    ASSERT_NO_THROW(*sut_ = vectorXd3_);
}

TEST_F(VectorXdShould, HaveCorrectStdVectorAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = vectorXdVector1_);
    ASSERT_NO_THROW(sut_.reset(new VectorXd(vectorXdVector2_)));
    ASSERT_NO_THROW(*sut_ = vectorXdVector1_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdVector(vectorXdVector1_));
    ASSERT_NO_THROW(*sut_ = vectorXdVector1_);
    ASSERT_NO_THROW(*sut_ = vectorXdVector2_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdVector(vectorXdVector2_));
    ASSERT_NO_THROW(*sut_ = vectorXdVector3_);
}

TEST_F(VectorXdShould, HaveCorrectInitializerListAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = vectorXdInitializerListDifferentLength2_);
    ASSERT_NO_THROW(sut_.reset(new VectorXd(vectorXd2_)));
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    ASSERT_EQ(sut_->raw(), vectorXd1_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_EQ(sut_->raw(), vectorXd2_);
    ASSERT_NO_THROW(*sut_ = vectorXdInitializerListDifferentLength1_);
}

TEST_F(VectorXdShould, HaveCorrectSizeOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 0);
    ASSERT_NO_THROW(sut_.reset(new VectorXd(vectorXd1_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
    ASSERT_NO_THROW(*sut_ = vectorXd2_);
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
    ASSERT_NO_THROW(sut_.reset(new VectorXd(vectorXd3_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 7);
}

TEST_F(VectorXdShould, HaveCorrectRawOperator) {
    ASSERT_NO_THROW(outputVectorXd_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
    ASSERT_NO_THROW(sut_.reset(new VectorXd(vectorXd1_)));
    ASSERT_NO_THROW(outputVectorXd_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), vectorXd1_);
    ASSERT_NO_THROW(*sut_ = vectorXd1_);
    ASSERT_NO_THROW(*sut_ = vectorXd2_);
    ASSERT_NO_THROW(outputVectorXd_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), vectorXd2_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_NO_THROW(outputVectorXd_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), vectorXd2_);
}

TEST_F(VectorXdShould, HaveCorrectNonConstIndexOperator) {
    ASSERT_THROW((*sut_)[0] = 2.2, std::out_of_range);
    ASSERT_THROW((*sut_)[1] = -3.8, std::out_of_range);
    ASSERT_THROW((*sut_)[2] = 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[3] = 5.8, std::out_of_range);
    ASSERT_THROW((*sut_)[4] = 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[5] = 3.9, std::out_of_range);
    ASSERT_NO_THROW(sut_.reset(new VectorXd(vectorXd2_)));
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
    ASSERT_EQ(sut_->raw(), vectorXd1_);
}

TEST_F(VectorXdShould, HaveCorrectConstIndexOperator) {
    ASSERT_THROW(outputDouble_ = (*sut_)[0], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[1], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[2], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[3], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[4], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[5], std::out_of_range);
    ASSERT_NO_THROW(sut_.reset(new VectorXd(vectorXd1_)));
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

TEST_F(VectorXdShould, HaveCorrectStreamOperator) {
    ASSERT_NO_THROW(sut_.reset(new VectorXd(vectorXd1_)));
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
    ASSERT_NO_THROW(sut_.reset(new VectorXd(vectorXd3_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "-1.245265478902345\n"
        " 3.148728347934453\n"
        " 2.845635634435476\n"
        " 4.000000000000000\n"
        "23.134142312313418\n"
        " 0.000123124200000\n"
        " 5.227840000000000");
}
