/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/TaskTypes/TaskSpace.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::TaskSpace;
using crf::utility::types::TaskSpaceTangentDimension;

class TaskSpaceShould : public ::testing::Test {
 protected:
    TaskSpaceShould() :
        sut_(),
        defaultConstructorVector_({1, 1, 1, 1, 1, 1}),
        defaultConstructorNoRowsMatrix_(6, 6),
        defaultConstructorZeroRowsMatrix_(
            {{1, 0, 0, 0, 0, 0},
             {0, 1, 0, 0, 0, 0},
             {0, 0, 1, 0, 0, 0},
             {0, 0, 0, 1, 0, 0},
             {0, 0, 0, 0, 1, 0},
             {0, 0, 0, 0, 0, 1}}),
        defaultConstructorNaNRowsMatrix_(
            {{1, 0, 0, 0, 0, 0},
             {0, 1, 0, 0, 0, 0},
             {0, 0, 1, 0, 0, 0},
             {0, 0, 0, 1, 0, 0},
             {0, 0, 0, 0, 1, 0},
             {0, 0, 0, 0, 0, 1}}),
        taskSpaceMap1_(
            {{TaskSpaceTangentDimension::Vx, true},
             {TaskSpaceTangentDimension::Vy, true},
             {TaskSpaceTangentDimension::Vz, true},
             {TaskSpaceTangentDimension::Wx, true},
             {TaskSpaceTangentDimension::Wy, true},
             {TaskSpaceTangentDimension::Wz, true}}),
        taskSpaceMap2_(
            {{TaskSpaceTangentDimension::Vx, true},
             {TaskSpaceTangentDimension::Vy, true},
             {TaskSpaceTangentDimension::Vz, false},
             {TaskSpaceTangentDimension::Wx, true},
             {TaskSpaceTangentDimension::Wy, true},
             {TaskSpaceTangentDimension::Wz, true}}),
        taskSpaceMap3_(
            {{TaskSpaceTangentDimension::Vx, false},
             {TaskSpaceTangentDimension::Vy, true},
             {TaskSpaceTangentDimension::Vz, true},
             {TaskSpaceTangentDimension::Wx, true},
             {TaskSpaceTangentDimension::Wy, false},
             {TaskSpaceTangentDimension::Wz, false}}),
        taskSpaceMap4_(
            {{TaskSpaceTangentDimension::Vx, true},
             {TaskSpaceTangentDimension::Vy, true},
             {TaskSpaceTangentDimension::Vz, true},
             {TaskSpaceTangentDimension::Wx, false},
             {TaskSpaceTangentDimension::Wy, true},
             {TaskSpaceTangentDimension::Wz, true}}),
        taskSpaceArray1_({true, true, true, true, true, true}),
        taskSpaceArray2_({true, true, false, true, true, true}),
        taskSpaceArray3_({false, true, true, true, false, false}),
        taskSpaceArray4_({true, true, true, false, true, true}),
        taskSpaceVector1_({1, 1, 1, 1, 1, 1}),
        taskSpaceVector2_({1, 1, 0, 1, 1, 1}),
        taskSpaceVector3_({0, 1, 1, 1, 0, 0}),
        taskSpaceVector4_({1, 1, 1, 0, 1, 1}),
        taskSpaceNoRowsMatrix1_(6, 6),
        taskSpaceNoRowsMatrix2_(5, 6),
        taskSpaceNoRowsMatrix3_(3, 6),
        taskSpaceNoRowsMatrix4_(5, 6),
        taskSpaceZeroRowsMatrix1_(
            {{1, 0, 0, 0, 0, 0},
             {0, 1, 0, 0, 0, 0},
             {0, 0, 1, 0, 0, 0},
             {0, 0, 0, 1, 0, 0},
             {0, 0, 0, 0, 1, 0},
             {0, 0, 0, 0, 0, 1}}),
        taskSpaceZeroRowsMatrix2_(
            {{1, 0, 0, 0, 0, 0},
             {0, 1, 0, 0, 0, 0},
             {0, 0, 0, 0, 0, 0},
             {0, 0, 0, 1, 0, 0},
             {0, 0, 0, 0, 1, 0},
             {0, 0, 0, 0, 0, 1}}),
        taskSpaceZeroRowsMatrix3_(
            {{0, 0, 0, 0, 0, 0},
             {0, 1, 0, 0, 0, 0},
             {0, 0, 1, 0, 0, 0},
             {0, 0, 0, 1, 0, 0},
             {0, 0, 0, 0, 0, 0},
             {0, 0, 0, 0, 0, 0}}),
        taskSpaceZeroRowsMatrix4_(
            {{1, 0, 0, 0, 0, 0},
             {0, 1, 0, 0, 0, 0},
             {0, 0, 1, 0, 0, 0},
             {0, 0, 0, 0, 0, 0},
             {0, 0, 0, 0, 1, 0},
             {0, 0, 0, 0, 0, 1}}),
        taskSpaceNaNRowsMatrix1_(
            {{1, 0, 0, 0, 0, 0},
             {0, 1, 0, 0, 0, 0},
             {0, 0, 1, 0, 0, 0},
             {0, 0, 0, 1, 0, 0},
             {0, 0, 0, 0, 1, 0},
             {0, 0, 0, 0, 0, 1}}),
        taskSpaceNaNRowsMatrix2_(
            {{1, 0, 0, 0, 0, 0},
             {0, 1, 0, 0, 0, 0},
             {std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN()},
             {0, 0, 0, 1, 0, 0},
             {0, 0, 0, 0, 1, 0},
             {0, 0, 0, 0, 0, 1}}),
        taskSpaceNaNRowsMatrix3_(
            {{std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN()},
             {0, 1, 0, 0, 0, 0},
             {0, 0, 1, 0, 0, 0},
             {0, 0, 0, 1, 0, 0},
             {std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN()},
             {std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN()}}),
        taskSpaceNaNRowsMatrix4_(
            {{1, 0, 0, 0, 0, 0},
             {0, 1, 0, 0, 0, 0},
             {0, 0, 1, 0, 0, 0},
             {std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN()},
             {0, 0, 0, 0, 1, 0},
             {0, 0, 0, 0, 0, 1}}),
        taskSpaceInitializerListWrongLength_({false, true, true, false, false}) {
        defaultConstructorNoRowsMatrix_ << 1, 0, 0, 0, 0, 0,
                                           0, 1, 0, 0, 0, 0,
                                           0, 0, 1, 0, 0, 0,
                                           0, 0, 0, 1, 0, 0,
                                           0, 0, 0, 0, 1, 0,
                                           0, 0, 0, 0, 0, 1;
        taskSpaceNoRowsMatrix1_ << 1, 0, 0, 0, 0, 0,
                                   0, 1, 0, 0, 0, 0,
                                   0, 0, 1, 0, 0, 0,
                                   0, 0, 0, 1, 0, 0,
                                   0, 0, 0, 0, 1, 0,
                                   0, 0, 0, 0, 0, 1;

        taskSpaceNoRowsMatrix2_ << 1, 0, 0, 0, 0, 0,
                                   0, 1, 0, 0, 0, 0,
                                   0, 0, 0, 1, 0, 0,
                                   0, 0, 0, 0, 1, 0,
                                   0, 0, 0, 0, 0, 1;

        taskSpaceNoRowsMatrix3_ << 0, 1, 0, 0, 0, 0,
                                   0, 0, 1, 0, 0, 0,
                                   0, 0, 0, 1, 0, 0;

        taskSpaceNoRowsMatrix4_ << 1, 0, 0, 0, 0, 0,
                                   0, 1, 0, 0, 0, 0,
                                   0, 0, 1, 0, 0, 0,
                                   0, 0, 0, 0, 1, 0,
                                   0, 0, 0, 0, 0, 1;
    }

    void SetUp() {
        sut_.reset(new TaskSpace());
        outputDouble_ = 0;
        outputBool_ = true;
        outputMatrixX6_ = Eigen::Matrix<double, 6, 6>::Identity();
        outputMatrix6x6_ = Eigen::Matrix<double, 6, 6>::Identity();
        outputVector6d_ = Eigen::Vector<double, 6>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        os_ = std::stringstream();
    }

    bool areEqualWithNaNs(
        const Eigen::Matrix<double, 6, 6>& matrix1,
        const Eigen::Matrix<double, 6, 6>& matrix2) {
        for (size_t i = 0; i < 6; i++) {
            for (size_t j = 0; j < 6; j++) {
                if (matrix1(i, j) != matrix2(i, j) &&
                    !(std::isnan(matrix1(i, j)) && std::isnan(matrix2(i, j)))) {
                    return false;
                }
            }
        }
        return true;
    }

    std::unique_ptr<TaskSpace> sut_;

    const Eigen::Vector<double, 6> defaultConstructorVector_;
    Eigen::Matrix<double, Eigen::Dynamic, 6> defaultConstructorNoRowsMatrix_;
    const Eigen::Matrix<double, 6, 6> defaultConstructorZeroRowsMatrix_;
    const Eigen::Matrix<double, 6, 6> defaultConstructorNaNRowsMatrix_;
    const std::map<TaskSpaceTangentDimension, bool> taskSpaceMap1_;
    const std::map<TaskSpaceTangentDimension, bool> taskSpaceMap2_;
    const std::map<TaskSpaceTangentDimension, bool> taskSpaceMap3_;
    const std::map<TaskSpaceTangentDimension, bool> taskSpaceMap4_;
    const std::array<bool, 6> taskSpaceArray1_;
    const std::array<bool, 6> taskSpaceArray2_;
    const std::array<bool, 6> taskSpaceArray3_;
    const std::array<bool, 6> taskSpaceArray4_;
    const Eigen::Vector<double, 6> taskSpaceVector1_;
    const Eigen::Vector<double, 6> taskSpaceVector2_;
    const Eigen::Vector<double, 6> taskSpaceVector3_;
    const Eigen::Vector<double, 6> taskSpaceVector4_;

    Eigen::Matrix<double, Eigen::Dynamic, 6> taskSpaceNoRowsMatrix1_;
    Eigen::Matrix<double, Eigen::Dynamic, 6> taskSpaceNoRowsMatrix2_;
    Eigen::Matrix<double, Eigen::Dynamic, 6> taskSpaceNoRowsMatrix3_;
    Eigen::Matrix<double, Eigen::Dynamic, 6> taskSpaceNoRowsMatrix4_;

    const Eigen::Matrix<double, 6, 6> taskSpaceZeroRowsMatrix1_;
    const Eigen::Matrix<double, 6, 6> taskSpaceZeroRowsMatrix2_;
    const Eigen::Matrix<double, 6, 6> taskSpaceZeroRowsMatrix3_;
    const Eigen::Matrix<double, 6, 6> taskSpaceZeroRowsMatrix4_;

    const Eigen::Matrix<double, 6, 6> taskSpaceNaNRowsMatrix1_;
    const Eigen::Matrix<double, 6, 6> taskSpaceNaNRowsMatrix2_;
    const Eigen::Matrix<double, 6, 6> taskSpaceNaNRowsMatrix3_;
    const Eigen::Matrix<double, 6, 6> taskSpaceNaNRowsMatrix4_;
    const std::initializer_list<bool> taskSpaceInitializerListWrongLength_;

    double outputDouble_;
    bool outputBool_;
    Eigen::Matrix<double, Eigen::Dynamic, 6> outputMatrixX6_;
    Eigen::Matrix<double, 6, 6> outputMatrix6x6_;
    Eigen::Vector<double, 6> outputVector6d_;

    std::stringstream os_;
};

TEST_F(TaskSpaceShould, HaveCorrectDefaultConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskSpace()));
    ASSERT_EQ(sut_->getVector6d(), defaultConstructorVector_);
}

TEST_F(TaskSpaceShould, HaveCorrectMapConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskSpace(taskSpaceMap3_)));
    ASSERT_EQ(sut_->getVector6d(), taskSpaceVector3_);
}

TEST_F(TaskSpaceShould, HaveCorrectArrayConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskSpace(taskSpaceArray3_)));
    ASSERT_EQ(sut_->getVector6d(), taskSpaceVector3_);
}

TEST_F(TaskSpaceShould, HaveCorrectInitializerListConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskSpace({false, true, true, true, false, false})));
    ASSERT_EQ(sut_->getVector6d(), taskSpaceVector3_);
    ASSERT_THROW(
        sut_.reset(new TaskSpace({false, true, true, false, false})), std::invalid_argument);
}

TEST_F(TaskSpaceShould, HaveCorrectMapAssignmentOperator) {
    ASSERT_NO_THROW(*sut_ = taskSpaceMap1_);
    ASSERT_EQ(sut_->getVector6d(), taskSpaceVector1_);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap1_);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap3_);
    ASSERT_EQ(sut_->getVector6d(), taskSpaceVector3_);
}

TEST_F(TaskSpaceShould, HaveCorrectArrayAssignmentOperator) {
    ASSERT_NO_THROW(*sut_ = taskSpaceArray1_);
    ASSERT_EQ(sut_->getVector6d(), taskSpaceVector1_);
    ASSERT_NO_THROW(*sut_ = taskSpaceArray1_);
    ASSERT_NO_THROW(*sut_ = taskSpaceArray3_);
    ASSERT_EQ(sut_->getVector6d(), taskSpaceVector3_);
}

TEST_F(TaskSpaceShould, HaveCorrectInitializerListAssignmentOperator) {
    *sut_ = {true, true, true, true, true, true};
    ASSERT_EQ(sut_->getVector6d(), taskSpaceVector1_);
    *sut_ = {true, true, true, true, true, true};
    *sut_ = {false, true, true, true, false, false};
    ASSERT_EQ(sut_->getVector6d(), taskSpaceVector3_);
    ASSERT_THROW(*sut_ = taskSpaceInitializerListWrongLength_, std::invalid_argument);
}

TEST_F(TaskSpaceShould, HaveCorrectDimensionOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->dimension());
    ASSERT_EQ(sut_->dimension(), 6);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap1_);
    ASSERT_NO_THROW(outputDouble_ = sut_->dimension());
    ASSERT_EQ(sut_->dimension(), 6);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap2_);
    ASSERT_NO_THROW(outputDouble_ = sut_->dimension());
    ASSERT_EQ(sut_->dimension(), 5);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap3_);
    ASSERT_NO_THROW(outputDouble_ = sut_->dimension());
    ASSERT_EQ(sut_->dimension(), 3);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap4_);
    ASSERT_NO_THROW(outputDouble_ = sut_->dimension());
    ASSERT_EQ(sut_->dimension(), 5);
}

TEST_F(TaskSpaceShould, HaveCorrectLinearDimensionOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->linearDimension());
    ASSERT_EQ(sut_->linearDimension(), 3);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap1_);
    ASSERT_NO_THROW(outputDouble_ = sut_->linearDimension());
    ASSERT_EQ(sut_->linearDimension(), 3);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap2_);
    ASSERT_NO_THROW(outputDouble_ = sut_->linearDimension());
    ASSERT_EQ(sut_->linearDimension(), 2);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap3_);
    ASSERT_NO_THROW(outputDouble_ = sut_->linearDimension());
    ASSERT_EQ(sut_->linearDimension(), 2);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap4_);
    ASSERT_NO_THROW(outputDouble_ = sut_->linearDimension());
    ASSERT_EQ(sut_->linearDimension(), 3);
}

TEST_F(TaskSpaceShould, HaveCorrectAngularDimensionOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->angularDimension());
    ASSERT_EQ(sut_->angularDimension(), 3);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap1_);
    ASSERT_NO_THROW(outputDouble_ = sut_->angularDimension());
    ASSERT_EQ(sut_->angularDimension(), 3);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap2_);
    ASSERT_NO_THROW(outputDouble_ = sut_->angularDimension());
    ASSERT_EQ(sut_->angularDimension(), 3);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap3_);
    ASSERT_NO_THROW(outputDouble_ = sut_->angularDimension());
    ASSERT_EQ(sut_->angularDimension(), 1);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap4_);
    ASSERT_NO_THROW(outputDouble_ = sut_->angularDimension());
    ASSERT_EQ(sut_->angularDimension(), 2);
}

TEST_F(TaskSpaceShould, HaveCorrectNoRowsMatrixGetter) {
    ASSERT_NO_THROW(outputMatrixX6_ = sut_->getNoRowsMatrix());
    ASSERT_EQ(sut_->getNoRowsMatrix(), defaultConstructorNoRowsMatrix_);
    ASSERT_NO_THROW(sut_.reset(new TaskSpace(taskSpaceMap1_)));
    ASSERT_NO_THROW(outputMatrixX6_ = sut_->getNoRowsMatrix());
    ASSERT_EQ(sut_->getNoRowsMatrix(), taskSpaceNoRowsMatrix1_);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap1_);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap3_);
    ASSERT_NO_THROW(outputMatrixX6_ = sut_->getNoRowsMatrix());
    ASSERT_EQ(sut_->getNoRowsMatrix(), taskSpaceNoRowsMatrix3_);
}

TEST_F(TaskSpaceShould, HaveCorrectZeroRowsMatrixGetter) {
    ASSERT_NO_THROW(outputMatrix6x6_ = sut_->getZeroRowsMatrix());
    ASSERT_EQ(sut_->getZeroRowsMatrix(), defaultConstructorZeroRowsMatrix_);
    ASSERT_NO_THROW(sut_.reset(new TaskSpace(taskSpaceMap1_)));
    ASSERT_NO_THROW(outputMatrix6x6_ = sut_->getZeroRowsMatrix());
    ASSERT_EQ(sut_->getZeroRowsMatrix(), taskSpaceZeroRowsMatrix1_);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap1_);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap3_);
    ASSERT_NO_THROW(outputMatrix6x6_ = sut_->getZeroRowsMatrix());
    ASSERT_EQ(sut_->getZeroRowsMatrix(), taskSpaceZeroRowsMatrix3_);
}

TEST_F(TaskSpaceShould, HaveCorrectNaNRowsMatrixGetter) {
    ASSERT_NO_THROW(outputMatrix6x6_ = sut_->getNaNRowsMatrix());
    ASSERT_EQ(sut_->getNaNRowsMatrix(), defaultConstructorNaNRowsMatrix_);
    ASSERT_NO_THROW(sut_.reset(new TaskSpace(taskSpaceMap1_)));
    ASSERT_NO_THROW(outputMatrix6x6_ = sut_->getNaNRowsMatrix());
    ASSERT_TRUE(areEqualWithNaNs(sut_->getNaNRowsMatrix(), taskSpaceNaNRowsMatrix1_));
    ASSERT_NO_THROW(*sut_ = taskSpaceMap1_);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap3_);
    ASSERT_NO_THROW(outputMatrix6x6_ = sut_->getNaNRowsMatrix());
    ASSERT_TRUE(areEqualWithNaNs(sut_->getNaNRowsMatrix(), taskSpaceNaNRowsMatrix3_));
}

TEST_F(TaskSpaceShould, HaveCorrectVector6dGetter) {
    ASSERT_NO_THROW(outputVector6d_ = sut_->getVector6d());
    ASSERT_EQ(sut_->getVector6d(), defaultConstructorVector_);
    ASSERT_NO_THROW(sut_.reset(new TaskSpace(taskSpaceMap1_)));
    ASSERT_NO_THROW(outputVector6d_ = sut_->getVector6d());
    ASSERT_EQ(sut_->getVector6d(), taskSpaceVector1_);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap1_);
    ASSERT_NO_THROW(*sut_ = taskSpaceMap3_);
    ASSERT_NO_THROW(outputVector6d_ = sut_->getVector6d());
    ASSERT_EQ(sut_->getVector6d(), taskSpaceVector3_);
}

TEST_F(TaskSpaceShould, HaveCorrectSizeOperator) {
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
    ASSERT_NO_THROW(sut_.reset(new TaskSpace(taskSpaceMap1_)));
    ASSERT_NO_THROW(outputDouble_ = sut_->size());
    ASSERT_EQ(sut_->size(), 6);
}

TEST_F(TaskSpaceShould, HaveCorrectNonConstDimensionIndexOperator) {
    ASSERT_NO_THROW((*sut_)[TaskSpaceTangentDimension::Vx] = false);
    ASSERT_NO_THROW((*sut_)[TaskSpaceTangentDimension::Vy] = true);
    ASSERT_NO_THROW((*sut_)[TaskSpaceTangentDimension::Vz] = true);
    ASSERT_NO_THROW((*sut_)[TaskSpaceTangentDimension::Wx] = true);
    ASSERT_NO_THROW((*sut_)[TaskSpaceTangentDimension::Wy] = false);
    ASSERT_NO_THROW((*sut_)[TaskSpaceTangentDimension::Wz] = false);
    ASSERT_EQ(sut_->getVector6d(), taskSpaceVector3_);
}

TEST_F(TaskSpaceShould, HaveCorrectConstDimensionIndexOperator) {
    ASSERT_NO_THROW(sut_.reset(new TaskSpace(taskSpaceMap3_)));
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[TaskSpaceTangentDimension::Vx]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[TaskSpaceTangentDimension::Vy]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[TaskSpaceTangentDimension::Vz]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[TaskSpaceTangentDimension::Wx]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[TaskSpaceTangentDimension::Wy]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[TaskSpaceTangentDimension::Wz]);
    ASSERT_EQ((*sut_)[TaskSpaceTangentDimension::Vx], false);
    ASSERT_EQ((*sut_)[TaskSpaceTangentDimension::Vy], true);
    ASSERT_EQ((*sut_)[TaskSpaceTangentDimension::Vz], true);
    ASSERT_EQ((*sut_)[TaskSpaceTangentDimension::Wx], true);
    ASSERT_EQ((*sut_)[TaskSpaceTangentDimension::Wy], false);
    ASSERT_EQ((*sut_)[TaskSpaceTangentDimension::Wz], false);
}

TEST_F(TaskSpaceShould, HaveCorrectNonConstSizeTIndexOperator) {
    ASSERT_NO_THROW((*sut_)[0] = false);
    ASSERT_NO_THROW((*sut_)[1] = true);
    ASSERT_NO_THROW((*sut_)[2] = true);
    ASSERT_NO_THROW((*sut_)[3] = true);
    ASSERT_NO_THROW((*sut_)[4] = false);
    ASSERT_NO_THROW((*sut_)[5] = false);
    ASSERT_THROW((*sut_)[-1] = true, std::out_of_range);
    ASSERT_THROW((*sut_)[6] = true, std::out_of_range);
    ASSERT_THROW((*sut_)[-8] = false, std::out_of_range);
    ASSERT_THROW((*sut_)[10] = false, std::out_of_range);
    ASSERT_EQ(sut_->getVector6d(), taskSpaceVector3_);
}

TEST_F(TaskSpaceShould, HaveCorrectConstSizeTIndexOperator) {
    ASSERT_NO_THROW(sut_.reset(new TaskSpace(taskSpaceMap3_)));
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[0]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[1]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[2]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[3]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[4]);
    ASSERT_NO_THROW(outputDouble_ = (*sut_)[5]);
    ASSERT_EQ((*sut_)[0], false);
    ASSERT_EQ((*sut_)[1], true);
    ASSERT_EQ((*sut_)[2], true);
    ASSERT_EQ((*sut_)[3], true);
    ASSERT_EQ((*sut_)[4], false);
    ASSERT_EQ((*sut_)[5], false);
    ASSERT_THROW((*sut_)[-1] == true, std::out_of_range);
    ASSERT_THROW((*sut_)[6] == true, std::out_of_range);
    ASSERT_THROW((*sut_)[-8] == false, std::out_of_range);
    ASSERT_THROW((*sut_)[10] == false, std::out_of_range);
}
TEST_F(TaskSpaceShould, HaveCorrectStreamOperator) {
    ASSERT_NO_THROW(sut_.reset(new TaskSpace(taskSpaceMap1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "Vx: true\n"
        "Vy: true\n"
        "Vz: true\n"
        "Wx: true\n"
        "Wy: true\n"
        "Wz: true");
    os_ = std::stringstream();
    ASSERT_NO_THROW(sut_.reset(new TaskSpace(taskSpaceMap3_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "Vx: false\n"
        "Vy: true\n"
        "Vz: true\n"
        "Wx: true\n"
        "Wy: false\n"
        "Wz: false");
}
