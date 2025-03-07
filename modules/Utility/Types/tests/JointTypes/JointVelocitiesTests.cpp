/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Conversions.hpp"
#include "Types/JointTypes/JointVelocities.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::JointVelocities;
using crf::utility::types::eigenVectorFromStdVector;

class JointVelocitiesShould : public ::testing::Test {
 protected:
    JointVelocitiesShould() :
        sut_(),
        defaultConstructorVector_(0),
        jointVelocities1_(6),
        jointVelocities2_(6),
        jointVelocities3_(7),
        jointVelocitiesVector1_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        jointVelocitiesVector2_({1.2, 3.14, 2.8, 2.2, 5.7, 3.2}),
        jointVelocitiesVector3_(
            {-1.2452654789023454,
             3.1487283479344533,
             2.8456356344354765,
             4.0,
             23.134142312313420,
             0.0001231242,
             5.22784}),
        jointVelocitiesInitializerListDifferentLength1_({2.2, -3.8, 4.7, 5.5}),
        jointVelocitiesInitializerListDifferentLength2_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        outputDouble_(0),
        outputJointVelocities_(),
        os_() {
        jointVelocities1_ << 2.2, -3.8, 4.7, 5.8, 4.7, 3.9;
        jointVelocities2_ << 1.2, 3.14, 2.8, 2.2, 5.7, 3.2;
        jointVelocities3_ << -1.2452654789023454, 3.1487283479344533, 2.8456356344354765, 4.0,
            23.134142312313420, 0.0001231242, 5.22784;
    }

    void SetUp() {
        sut_.reset(new JointVelocities());
        outputDouble_ = 0;
        outputJointVelocities_ = Eigen::VectorXd(0);
        os_ = std::stringstream();
    }

    std::unique_ptr<JointVelocities> sut_;

    const Eigen::VectorXd defaultConstructorVector_;
    Eigen::VectorXd jointVelocities1_;
    Eigen::VectorXd jointVelocities2_;
    Eigen::VectorXd jointVelocities3_;
    const std::vector<double> jointVelocitiesVector1_;
    const std::vector<double> jointVelocitiesVector2_;
    const std::vector<double> jointVelocitiesVector3_;
    const std::initializer_list<double> jointVelocitiesInitializerListDifferentLength1_;
    const std::initializer_list<double> jointVelocitiesInitializerListDifferentLength2_;

    double outputDouble_;
    Eigen::VectorXd outputJointVelocities_;

    std::stringstream os_;
};

TEST_F(JointVelocitiesShould, HaveCorrectDefaultConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointVelocities()));
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
}

TEST_F(JointVelocitiesShould, HaveCorrectEigenVectorConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointVelocities(jointVelocities1_)));
    ASSERT_EQ(sut_->raw(), jointVelocities1_);
}

TEST_F(JointVelocitiesShould, HaveCorrectVectorConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointVelocities(jointVelocitiesVector1_)));
    ASSERT_EQ(sut_->raw(), jointVelocities1_);
}

TEST_F(JointVelocitiesShould, HaveCorrectInitializerListConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointVelocities({2.2, -3.8, 4.7, 5.8, 4.7, 3.9})));
    ASSERT_EQ(sut_->raw(), jointVelocities1_);
}

TEST_F(JointVelocitiesShould, HaveCorrectVectorAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = jointVelocities1_);
    ASSERT_NO_THROW(sut_.reset(new JointVelocities(jointVelocities2_)));
    ASSERT_NO_THROW(*sut_ = jointVelocities1_);
    ASSERT_EQ(sut_->raw(), jointVelocities1_);
    ASSERT_NO_THROW(*sut_ = jointVelocities1_);
    ASSERT_NO_THROW(*sut_ = jointVelocities2_);
    ASSERT_EQ(sut_->raw(), jointVelocities2_);
    ASSERT_NO_THROW(*sut_ = jointVelocities3_);
}

TEST_F(JointVelocitiesShould, HaveCorrectStdVectorAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = jointVelocitiesVector1_);
    ASSERT_NO_THROW(sut_.reset(new JointVelocities(jointVelocitiesVector2_)));
    ASSERT_NO_THROW(*sut_ = jointVelocitiesVector1_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdVector(jointVelocitiesVector1_));
    ASSERT_NO_THROW(*sut_ = jointVelocitiesVector1_);
    ASSERT_NO_THROW(*sut_ = jointVelocitiesVector2_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdVector(jointVelocitiesVector2_));
    ASSERT_NO_THROW(*sut_ = jointVelocitiesVector3_);
}

TEST_F(JointVelocitiesShould, HaveCorrectInitializerListAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = jointVelocitiesInitializerListDifferentLength2_);
    ASSERT_NO_THROW(sut_.reset(new JointVelocities(jointVelocities2_)));
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    ASSERT_EQ(sut_->raw(), jointVelocities1_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_EQ(sut_->raw(), jointVelocities2_);
    ASSERT_NO_THROW(*sut_ = jointVelocitiesInitializerListDifferentLength1_);
}

TEST_F(JointVelocitiesShould, HaveCorrectSizeOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 0);
    ASSERT_NO_THROW(sut_.reset(new JointVelocities(jointVelocities1_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
    ASSERT_NO_THROW(*sut_ = jointVelocities2_);
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
    ASSERT_NO_THROW(sut_.reset(new JointVelocities(jointVelocities3_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 7);
}

TEST_F(JointVelocitiesShould, HaveCorrectRawOperator) {
    ASSERT_NO_THROW(outputJointVelocities_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
    ASSERT_NO_THROW(sut_.reset(new JointVelocities(jointVelocities1_)));
    ASSERT_NO_THROW(outputJointVelocities_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), jointVelocities1_);
    ASSERT_NO_THROW(*sut_ = jointVelocities1_);
    ASSERT_NO_THROW(*sut_ = jointVelocities2_);
    ASSERT_NO_THROW(outputJointVelocities_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), jointVelocities2_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_NO_THROW(outputJointVelocities_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), jointVelocities2_);
}

TEST_F(JointVelocitiesShould, HaveCorrectNonConstIndexOperator) {
    ASSERT_THROW((*sut_)[0] = 2.2, std::out_of_range);
    ASSERT_THROW((*sut_)[1] = -3.8, std::out_of_range);
    ASSERT_THROW((*sut_)[2] = 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[3] = 5.8, std::out_of_range);
    ASSERT_THROW((*sut_)[4] = 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[5] = 3.9, std::out_of_range);
    ASSERT_NO_THROW(sut_.reset(new JointVelocities(jointVelocities2_)));
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
    ASSERT_EQ(sut_->raw(), jointVelocities1_);
}

TEST_F(JointVelocitiesShould, HaveCorrectConstIndexOperator) {
    ASSERT_THROW(outputDouble_ = (*sut_)[0], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[1], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[2], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[3], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[4], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[5], std::out_of_range);
    ASSERT_NO_THROW(sut_.reset(new JointVelocities(jointVelocities1_)));
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

TEST_F(JointVelocitiesShould, HaveCorrectStreamOperator) {
    ASSERT_NO_THROW(sut_.reset(new JointVelocities(jointVelocities1_)));
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
    ASSERT_NO_THROW(sut_.reset(new JointVelocities(jointVelocities3_)));
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
