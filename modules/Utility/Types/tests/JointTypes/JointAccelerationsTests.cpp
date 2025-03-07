/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Conversions.hpp"
#include "Types/JointTypes/JointAccelerations.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::JointAccelerations;
using crf::utility::types::eigenVectorFromStdVector;

class JointAccelerationsShould : public ::testing::Test {
 protected:
    JointAccelerationsShould() :
        sut_(),
        defaultConstructorVector_(0),
        jointAccelerations1_(6),
        jointAccelerations2_(6),
        jointAccelerations3_(7),
        jointAccelerationsVector1_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        jointAccelerationsVector2_({1.2, 3.14, 2.8, 2.2, 5.7, 3.2}),
        jointAccelerationsVector3_(
            {-1.2452654789023454,
             3.1487283479344533,
             2.8456356344354765,
             4.0,
             23.134142312313420,
             0.0001231242,
             5.22784}),
        jointAccelerationsInitializerListDifferentLength1_({2.2, -3.8, 4.7, 5.5}),
        jointAccelerationsInitializerListDifferentLength2_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        outputDouble_(0),
        outputJointAccelerations_(),
        os_() {
        jointAccelerations1_ << 2.2, -3.8, 4.7, 5.8, 4.7, 3.9;
        jointAccelerations2_ << 1.2, 3.14, 2.8, 2.2, 5.7, 3.2;
        jointAccelerations3_ << -1.2452654789023454, 3.1487283479344533, 2.8456356344354765, 4.0,
            23.134142312313420, 0.0001231242, 5.22784;
    }

    void SetUp() {
        sut_.reset(new JointAccelerations());
        outputDouble_ = 0;
        outputJointAccelerations_ = Eigen::VectorXd(0);
        os_ = std::stringstream();
    }

    std::unique_ptr<JointAccelerations> sut_;

    const Eigen::VectorXd defaultConstructorVector_;
    Eigen::VectorXd jointAccelerations1_;
    Eigen::VectorXd jointAccelerations2_;
    Eigen::VectorXd jointAccelerations3_;
    const std::vector<double> jointAccelerationsVector1_;
    const std::vector<double> jointAccelerationsVector2_;
    const std::vector<double> jointAccelerationsVector3_;
    const std::initializer_list<double> jointAccelerationsInitializerListDifferentLength1_;
    const std::initializer_list<double> jointAccelerationsInitializerListDifferentLength2_;

    double outputDouble_;
    Eigen::VectorXd outputJointAccelerations_;

    std::stringstream os_;
};

TEST_F(JointAccelerationsShould, HaveCorrectDefaultConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointAccelerations()));
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
}

TEST_F(JointAccelerationsShould, HaveCorrectEigenVectorConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointAccelerations(jointAccelerations1_)));
    ASSERT_EQ(sut_->raw(), jointAccelerations1_);
}

TEST_F(JointAccelerationsShould, HaveCorrectVectorConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointAccelerations(jointAccelerationsVector1_)));
    ASSERT_EQ(sut_->raw(), jointAccelerations1_);
}

TEST_F(JointAccelerationsShould, HaveCorrectInitializerListConstructor) {
    ASSERT_NO_THROW(sut_.reset(new JointAccelerations({2.2, -3.8, 4.7, 5.8, 4.7, 3.9})));
    ASSERT_EQ(sut_->raw(), jointAccelerations1_);
}

TEST_F(JointAccelerationsShould, HaveCorrectVectorAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = jointAccelerations1_);
    ASSERT_NO_THROW(sut_.reset(new JointAccelerations(jointAccelerations2_)));
    ASSERT_NO_THROW(*sut_ = jointAccelerations1_);
    ASSERT_EQ(sut_->raw(), jointAccelerations1_);
    ASSERT_NO_THROW(*sut_ = jointAccelerations1_);
    ASSERT_NO_THROW(*sut_ = jointAccelerations2_);
    ASSERT_EQ(sut_->raw(), jointAccelerations2_);
    ASSERT_NO_THROW(*sut_ = jointAccelerations3_);
}

TEST_F(JointAccelerationsShould, HaveCorrectStdVectorAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = jointAccelerationsVector1_);
    ASSERT_NO_THROW(sut_.reset(new JointAccelerations(jointAccelerationsVector2_)));
    ASSERT_NO_THROW(*sut_ = jointAccelerationsVector1_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdVector(jointAccelerationsVector1_));
    ASSERT_NO_THROW(*sut_ = jointAccelerationsVector1_);
    ASSERT_NO_THROW(*sut_ = jointAccelerationsVector2_);
    ASSERT_EQ(sut_->raw(), eigenVectorFromStdVector(jointAccelerationsVector2_));
    ASSERT_NO_THROW(*sut_ = jointAccelerationsVector3_);
}

TEST_F(JointAccelerationsShould, HaveCorrectInitializerListAssingmentOperator) {
    ASSERT_NO_THROW(*sut_ = jointAccelerationsInitializerListDifferentLength2_);
    ASSERT_NO_THROW(sut_.reset(new JointAccelerations(jointAccelerations2_)));
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    ASSERT_EQ(sut_->raw(), jointAccelerations1_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_EQ(sut_->raw(), jointAccelerations2_);
    ASSERT_NO_THROW(*sut_ = jointAccelerationsInitializerListDifferentLength1_);
}

TEST_F(JointAccelerationsShould, HaveCorrectSizeOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 0);
    ASSERT_NO_THROW(sut_.reset(new JointAccelerations(jointAccelerations1_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
    ASSERT_NO_THROW(*sut_ = jointAccelerations2_);
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
    ASSERT_NO_THROW(sut_.reset(new JointAccelerations(jointAccelerations3_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 7);
}

TEST_F(JointAccelerationsShould, HaveCorrectRawOperator) {
    ASSERT_NO_THROW(outputJointAccelerations_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), defaultConstructorVector_);
    ASSERT_NO_THROW(sut_.reset(new JointAccelerations(jointAccelerations1_)));
    ASSERT_NO_THROW(outputJointAccelerations_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), jointAccelerations1_);
    ASSERT_NO_THROW(*sut_ = jointAccelerations1_);
    ASSERT_NO_THROW(*sut_ = jointAccelerations2_);
    ASSERT_NO_THROW(outputJointAccelerations_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), jointAccelerations2_);
    *sut_ = {2.2, -3.8, 4.7, 5.8, 4.7, 3.9};
    *sut_ = {1.2, 3.14, 2.8, 2.2, 5.7, 3.2};
    ASSERT_NO_THROW(outputJointAccelerations_ = sut_->raw());
    ASSERT_EQ(sut_->raw(), jointAccelerations2_);
}

TEST_F(JointAccelerationsShould, HaveCorrectNonConstIndexOperator) {
    ASSERT_THROW((*sut_)[0] = 2.2, std::out_of_range);
    ASSERT_THROW((*sut_)[1] = -3.8, std::out_of_range);
    ASSERT_THROW((*sut_)[2] = 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[3] = 5.8, std::out_of_range);
    ASSERT_THROW((*sut_)[4] = 4.7, std::out_of_range);
    ASSERT_THROW((*sut_)[5] = 3.9, std::out_of_range);
    ASSERT_NO_THROW(sut_.reset(new JointAccelerations(jointAccelerations2_)));
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
    ASSERT_EQ(sut_->raw(), jointAccelerations1_);
}

TEST_F(JointAccelerationsShould, HaveCorrectConstIndexOperator) {
    ASSERT_THROW(outputDouble_ = (*sut_)[0], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[1], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[2], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[3], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[4], std::out_of_range);
    ASSERT_THROW(outputDouble_ = (*sut_)[5], std::out_of_range);
    ASSERT_NO_THROW(sut_.reset(new JointAccelerations(jointAccelerations1_)));
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

TEST_F(JointAccelerationsShould, HaveCorrectStreamOperator) {
    ASSERT_NO_THROW(sut_.reset(new JointAccelerations(jointAccelerations1_)));
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
    ASSERT_NO_THROW(sut_.reset(new JointAccelerations(jointAccelerations3_)));
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
