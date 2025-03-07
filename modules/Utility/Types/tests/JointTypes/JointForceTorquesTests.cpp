/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Conversions.hpp"
#include "Types/JointTypes/JointForceTorques.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::JointForceTorques;
using crf::utility::types::eigenVectorFromStdVector;

class JointForceTorquesShould : public ::testing::Test {
 protected:
    JointForceTorquesShould() :
        sut_(),
        defaultConstructorVector_(0),
        jointForceTorques1_(6),
        jointForceTorques2_(6),
        jointForceTorques3_(7),
        jointForceTorquesVector1_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        jointForceTorquesVector2_({1.2, 3.14, 2.8, 2.2, 5.7, 3.2}),
        jointForceTorquesVector3_(
            {-1.2452654789023454,
             3.1487283479344533,
             2.8456356344354765,
             4.0,
             23.134142312313420,
             0.0001231242,
             5.22784}),
        jointForceTorquesInitializerListDifferentLength1_({2.2, -3.8, 4.7, 5.5}),
        jointForceTorquesInitializerListDifferentLength2_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        outputDouble_(0),
        outputJointForceTorques_(),
        os_() {
        jointForceTorques1_ << 2.2, -3.8, 4.7, 5.8, 4.7, 3.9;
        jointForceTorques2_ << 1.2, 3.14, 2.8, 2.2, 5.7, 3.2;
        jointForceTorques3_ << -1.2452654789023454, 3.1487283479344533, 2.8456356344354765, 4.0,
            23.134142312313420, 0.0001231242, 5.22784;
    }

    void SetUp() {
        sut_.reset(new JointForceTorques());
        outputDouble_ = 0;
        outputJointForceTorques_ = Eigen::VectorXd(0);
        os_ = std::stringstream();
    }

    std::unique_ptr<JointForceTorques> sut_;

    const Eigen::VectorXd defaultConstructorVector_;
    Eigen::VectorXd jointForceTorques1_;
    Eigen::VectorXd jointForceTorques2_;
    Eigen::VectorXd jointForceTorques3_;
    const std::vector<double> jointForceTorquesVector1_;
    const std::vector<double> jointForceTorquesVector2_;
    const std::vector<double> jointForceTorquesVector3_;
    const std::initializer_list<double> jointForceTorquesInitializerListDifferentLength1_;
    const std::initializer_list<double> jointForceTorquesInitializerListDifferentLength2_;

    double outputDouble_;
    Eigen::VectorXd outputJointForceTorques_;

    std::stringstream os_;
};

TEST_F(JointForceTorquesShould, HaveCorrectDefaultConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointForceTorques()));
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
}

TEST_F(JointForceTorquesShould, HaveCorrectEigenVectorConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointForceTorques(jointForceTorques1_)));
    ASSERT_EQ(sut_->raw(), jointForceTorques1_);
}

TEST_F(JointForceTorquesShould, HaveCorrectVectorConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointForceTorques(jointForceTorquesVector1_)));
    ASSERT_EQ(sut_->raw(), jointForceTorques1_);
}

TEST_F(JointForceTorquesShould, HaveCorrectInitializerListConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointForceTorques({2.2, -3.8, 4.7, 5.8, 4.7, 3.9})));
    ASSERT_EQ(sut_->raw(), jointForceTorques1_);
}

TEST_F(JointForceTorquesShould, HaveCorrectVectorAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = jointForceTorques1_);
    ASSERT_NO_THROW(sut_.reset(new JointForceTorques(jointForceTorques2_)));
    ASSERT_NO_THROW(*sut_ = jointForceTorques1_);
    ASSERT_EQ(sut_->raw(), jointForceTorques1_);
    ASSERT_NO_THROW(*sut_ = jointForceTorques1_);
    ASSERT_NO_THROW(*sut_ = jointForceTorques2_);
    ASSERT_EQ(sut_->raw(), jointForceTorques2_);
    ASSERT_NO_THROW(*sut_ = jointForceTorques3_);
}

TEST_F(JointForceTorquesShould, HaveCorrectStdVectorAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = jointForceTorquesVector1_);
    ASSERT_NO_THROW(sut_.reset(new JointForceTorques(jointForceTorquesVector2_)));
    ASSERT_NO_THROW(*sut_ = jointForceTorquesVector1_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdVector(jointForceTorquesVector1_));
    ASSERT_NO_THROW(*sut_ = jointForceTorquesVector1_);
    ASSERT_NO_THROW(*sut_ = jointForceTorquesVector2_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdVector(jointForceTorquesVector2_));
    ASSERT_NO_THROW(*sut_ = jointForceTorquesVector3_);
}

TEST_F(JointForceTorquesShould, HaveCorrectInitializerListAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = jointForceTorquesInitializerListDifferentLength2_);
    ASSERT_NO_THROW(sut_.reset(new JointForceTorques(jointForceTorques2_)));
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    ASSERT_EQ(sut_->raw(), jointForceTorques1_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_EQ(sut_->raw(), jointForceTorques2_);
    ASSERT_NO_THROW(*sut_ = jointForceTorquesInitializerListDifferentLength1_);
}

TEST_F(JointForceTorquesShould, HaveCorrectSizeOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 0);
    ASSERT_NO_THROW(sut_.reset(new JointForceTorques(jointForceTorques1_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
    ASSERT_NO_THROW(*sut_ = jointForceTorques2_);
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
    ASSERT_NO_THROW(sut_.reset(new JointForceTorques(jointForceTorques3_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 7);
}

TEST_F(JointForceTorquesShould, HaveCorrectRawOperator) {
    ASSERT_NO_THROW(outputJointForceTorques_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
    ASSERT_NO_THROW(sut_.reset(new JointForceTorques(jointForceTorques1_)));
    ASSERT_NO_THROW(outputJointForceTorques_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), jointForceTorques1_);
    ASSERT_NO_THROW(*sut_ = jointForceTorques1_);
    ASSERT_NO_THROW(*sut_ = jointForceTorques2_);
    ASSERT_NO_THROW(outputJointForceTorques_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), jointForceTorques2_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_NO_THROW(outputJointForceTorques_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), jointForceTorques2_);
}

TEST_F(JointForceTorquesShould, HaveCorrectNonConstIndexOperator) {
    ASSERT_THROW((*sut_)[0] = 2.2, std::out_of_range);
    ASSERT_THROW((*sut_)[1] = -3.8, std::out_of_range);
    ASSERT_THROW((*sut_)[2] = 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[3] = 5.8, std::out_of_range);
    ASSERT_THROW((*sut_)[4] = 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[5] = 3.9, std::out_of_range);
    ASSERT_NO_THROW(sut_.reset(new JointForceTorques(jointForceTorques2_)));
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
    ASSERT_EQ(sut_->raw(), jointForceTorques1_);
}

TEST_F(JointForceTorquesShould, HaveCorrectConstIndexOperator) {
    ASSERT_THROW(outputDouble_ = (*sut_)[0], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[1], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[2], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[3], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[4], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[5], std::out_of_range);
    ASSERT_NO_THROW(sut_.reset(new JointForceTorques(jointForceTorques1_)));
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

TEST_F(JointForceTorquesShould, HaveCorrectStreamOperator) {
    ASSERT_NO_THROW(sut_.reset(new JointForceTorques(jointForceTorques1_)));
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
    ASSERT_NO_THROW(sut_.reset(new JointForceTorques(jointForceTorques3_)));
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
