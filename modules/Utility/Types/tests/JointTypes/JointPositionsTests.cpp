/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Conversions.hpp"
#include "Types/JointTypes/JointPositions.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::JointPositions;
using crf::utility::types::eigenVectorFromStdVector;

class JointPositionsShould : public ::testing::Test {
 protected:
    JointPositionsShould() :
        sut_(),
        defaultConstructorVector_(0),
        jointPositions1_(6),
        jointPositions2_(6),
        jointPositions3_(7),
        jointPositionsVector1_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        jointPositionsVector2_({1.2, 3.14, 2.8, 2.2, 5.7, 3.2}),
        jointPositionsVector3_(
            {-1.2452654789023454,
             3.1487283479344533,
             2.8456356344354765,
             4.0,
             23.134142312313420,
             0.0001231242,
             5.22784}),
        jointPositionsInitializerListDifferentLength1_({2.2, -3.8, 4.7, 5.5}),
        jointPositionsInitializerListDifferentLength2_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        outputDouble_(0),
        outputJointPositions_(),
        os_() {
        jointPositions1_ << 2.2, -3.8, 4.7, 5.8, 4.7, 3.9;
        jointPositions2_ << 1.2, 3.14, 2.8, 2.2, 5.7, 3.2;
        jointPositions3_ << -1.2452654789023454, 3.1487283479344533, 2.8456356344354765, 4.0,
            23.134142312313420, 0.0001231242, 5.22784;
    }

    void SetUp() {
        sut_.reset(new JointPositions());
        outputDouble_ = 0;
        outputJointPositions_ = Eigen::VectorXd(0);
        os_ = std::stringstream();
    }

    std::unique_ptr<JointPositions> sut_;

    const Eigen::VectorXd defaultConstructorVector_;
    Eigen::VectorXd jointPositions1_;
    Eigen::VectorXd jointPositions2_;
    Eigen::VectorXd jointPositions3_;
    const std::vector<double> jointPositionsVector1_;
    const std::vector<double> jointPositionsVector2_;
    const std::vector<double> jointPositionsVector3_;
    const std::initializer_list<double> jointPositionsInitializerListDifferentLength1_;
    const std::initializer_list<double> jointPositionsInitializerListDifferentLength2_;

    double outputDouble_;
    Eigen::VectorXd outputJointPositions_;

    std::stringstream os_;
};

TEST_F(JointPositionsShould, HaveCorrectDefaultConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointPositions()));
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
}

TEST_F(JointPositionsShould, HaveCorrectEigenVectorConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointPositions(jointPositions1_)));
    ASSERT_EQ(sut_->raw(), jointPositions1_);
}

TEST_F(JointPositionsShould, HaveCorrectVectorConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointPositions(jointPositionsVector1_)));
    ASSERT_EQ(sut_->raw(), jointPositions1_);
}

TEST_F(JointPositionsShould, HaveCorrectInitializerListConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointPositions({2.2, -3.8, 4.7, 5.8, 4.7, 3.9})));
    ASSERT_EQ(sut_->raw(), jointPositions1_);
}

TEST_F(JointPositionsShould, HaveCorrectVectorAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = jointPositions1_);
    ASSERT_NO_THROW(sut_.reset(new JointPositions(jointPositions2_)));
    ASSERT_NO_THROW(*sut_ = jointPositions1_);
    ASSERT_EQ(sut_->raw(), jointPositions1_);
    ASSERT_NO_THROW(*sut_ = jointPositions1_);
    ASSERT_NO_THROW(*sut_ = jointPositions2_);
    ASSERT_EQ(sut_->raw(), jointPositions2_);
    ASSERT_NO_THROW(*sut_ = jointPositions3_);
}

TEST_F(JointPositionsShould, HaveCorrectStdVectorAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = jointPositionsVector1_);
    ASSERT_NO_THROW(sut_.reset(new JointPositions(jointPositionsVector2_)));
    ASSERT_NO_THROW(*sut_ = jointPositionsVector1_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdVector(jointPositionsVector1_));
    ASSERT_NO_THROW(*sut_ = jointPositionsVector1_);
    ASSERT_NO_THROW(*sut_ = jointPositionsVector2_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdVector(jointPositionsVector2_));
    ASSERT_NO_THROW(*sut_ = jointPositionsVector3_);
}

TEST_F(JointPositionsShould, HaveCorrectInitializerListAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = jointPositionsInitializerListDifferentLength2_);
    ASSERT_NO_THROW(sut_.reset(new JointPositions(jointPositions2_)));
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    ASSERT_EQ(sut_->raw(), jointPositions1_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_EQ(sut_->raw(), jointPositions2_);
    ASSERT_NO_THROW(*sut_ = jointPositionsInitializerListDifferentLength1_);
}

TEST_F(JointPositionsShould, HaveCorrectSizeOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 0);
    ASSERT_NO_THROW(sut_.reset(new JointPositions(jointPositions1_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
    ASSERT_NO_THROW(*sut_ = jointPositions2_);
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
    ASSERT_NO_THROW(sut_.reset(new JointPositions(jointPositions3_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 7);
}

TEST_F(JointPositionsShould, HaveCorrectRawOperator) {
    ASSERT_NO_THROW(outputJointPositions_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
    ASSERT_NO_THROW(sut_.reset(new JointPositions(jointPositions1_)));
    ASSERT_NO_THROW(outputJointPositions_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), jointPositions1_);
    ASSERT_NO_THROW(*sut_ = jointPositions1_);
    ASSERT_NO_THROW(*sut_ = jointPositions2_);
    ASSERT_NO_THROW(outputJointPositions_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), jointPositions2_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_NO_THROW(outputJointPositions_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), jointPositions2_);
}

TEST_F(JointPositionsShould, HaveCorrectNonConstIndexOperator) {
    ASSERT_THROW((*sut_)[0] = 2.2, std::out_of_range);
    ASSERT_THROW((*sut_)[1] = -3.8, std::out_of_range);
    ASSERT_THROW((*sut_)[2] = 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[3] = 5.8, std::out_of_range);
    ASSERT_THROW((*sut_)[4] = 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[5] = 3.9, std::out_of_range);
    ASSERT_NO_THROW(sut_.reset(new JointPositions(jointPositions2_)));
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
    ASSERT_EQ(sut_->raw(), jointPositions1_);
}

TEST_F(JointPositionsShould, HaveCorrectConstIndexOperator) {
    ASSERT_THROW(outputDouble_ = (*sut_)[0], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[1], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[2], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[3], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[4], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[5], std::out_of_range);
    ASSERT_NO_THROW(sut_.reset(new JointPositions(jointPositions1_)));
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

TEST_F(JointPositionsShould, HaveCorrectStreamOperator) {
    ASSERT_NO_THROW(sut_.reset(new JointPositions(jointPositions1_)));
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
    ASSERT_NO_THROW(sut_.reset(new JointPositions(jointPositions3_)));
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
