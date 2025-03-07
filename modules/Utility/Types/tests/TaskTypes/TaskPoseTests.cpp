/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Comparison.hpp"
#include "Types/TaskTypes/TaskPose.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::TaskPose;
using crf::math::rotation::Orientation;
using crf::math::rotation::OrientationRepresentation;
using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;
using crf::math::rotation::areAlmostEqual;
using crf::utility::types::areAlmostEqual;

class TaskPoseShould : public ::testing::Test {
 protected:
    TaskPoseShould() :
        sut_(),
        inputHomogeneousTransformationMatrix1_(
            {{0.4157869320692789, -0.9023592869214287, -0.1134413699981963, -2.537896000000000},
             {-0.0173036611331485, -0.1325610914209059, 0.9910237839490472, 3.451273000000000},
             {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917, -23.451267540000000},
             {0.000000000000000, 0.000000000000000, 0.000000000000000, 1.000000000000000}}),
        inputPosition1_({-2.537896, 3.451273, -23.45126754}),
        inputQuaternion1_(
            {0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        inputRotationMatrix1_(
            {{0.4157869320692789, -0.9023592869214287, -0.1134413699981963},
             {-0.0173036611331485, -0.1325610914209059, 0.9910237839490472},
             {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917}}),
        inputAngleAxis1_(
            1.9755068924749106,
            Eigen::Vector3d({-0.7621249848254679, 0.4328991822668132, 0.4814185346426796})),
        inputCardanXYZ1_({1.4, 2.0, 3.1}),
        inputEulerZXZ1_({1.1471123464639490, -1.6415867259093058, 0.1139727950424842}),
        inputHomogeneousTransformationMatrix2_(
            {{-0.1849992186629872, 0.8456391273700271, 0.5006693073825711, -14.530000000000000},
             {0.8491537754599140, 0.3940019571883435, -0.3517105675892164, 22.470000000000000},
             {-0.4946849044758272, 0.3600790524212897, -0.7909677119144169, 2.300000000000000},
             {0.000000000000000, 0.000000000000000, 0.000000000000000, 1.000000000000000}}),
        inputPosition2_({-14.53, 22.47, 2.3}),
        inputQuaternion2_(
            {0.3232781413160111, 0.5504467585659593, 0.7697351635084868, 0.0027179753598398}),
        inputRotationMatrix2_(
            {{-0.1849992186629872, 0.8456391273700271, 0.5006693073825711},
             {0.8491537754599140, 0.3940019571883435, -0.3517105675892164},
             {-0.4946849044758272, 0.3600790524212897, -0.7909677119144169}}),
        inputAngleAxis2_(
            2.4832094549611980,
            Eigen::Vector3d({0.5816806901502302, 0.8134121496309343, 0.0028722010957822})),
        inputCardanXYZ2_({2.7143908348394090, 0.5174722288733832, 1.7853074093465211}),
        inputEulerZXZ2_({-0.9415926535897930, 2.4831853071795869, 0.9584073464102065}),
        inputOrientationQuaternion1_(inputQuaternion1_),
        inputOrientationMatrix1_(inputRotationMatrix1_),
        inputOrientationAngleAxis1_(inputAngleAxis1_),
        inputOrientationCardanXYZ1_(inputCardanXYZ1_),
        inputOrientationEulerZXZ1_(inputEulerZXZ1_),
        inputOrientationQuaternion2_(inputQuaternion2_),
        inputOrientationMatrix2_(inputRotationMatrix2_),
        inputOrientationAngleAxis2_(inputAngleAxis2_),
        inputOrientationCardanXYZ2_(inputCardanXYZ2_),
        inputOrientationEulerZXZ2_(inputEulerZXZ2_),
        nonOrthogonalMatrixPart12_(
            {{0.4157869320692789, -0.9023592869214287, -0.1134413699981963, -2.537896000000000},
             {-0.0173036611331485, -0.1325610914209059, 0.9910237839490472, 3.451273000000000},
             {-0.9092974268266818, -0.4100917877109334, -0.0707312888348917, -23.451267540000000},
             {0.000000000000000, 0.000000000000000, 0.000000000000000, 1.000000000000000}}),
        nonOrthogonalMatrixPart5_(
            {{0.4157869320692789, -0.9023592869214287, -0.1134413699981963, -2.537896000000000},
             {-0.0172036611331485, -0.1325610914209059, 0.9910237839490472, 3.451273000000000},
             {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917, -23.451267540000000},
             {0.000000000000000, 0.000000000000000, 0.000000000000000, 1.000000000000000}}),
        orthogonalNonDet1MatrixPart12_(
            {{-0.4157869320692789, 0.9023592869214287, 0.1134413699981963, -2.537896000000000},
             {0.0173036611331485, 0.1325610914209059, -0.9910237839490472, 3.451273000000000},
             {0.9092974268266818, 0.4100917877109334, 0.0707312888348917, -23.451267540000000},
             {0.000000000000000, 0.000000000000000, 0.000000000000000, 1.000000000000000}}),
        orthogonalNonDet1MatrixPart5_(
            {{-0.4157869320692789, 0.9023592869214287, 0.1134413699981963, -2.537896000000000},
             {0.0172036611331485, 0.1325610914209059, -0.9910237839490472, 3.451273000000000},
             {0.9092974268256818, 0.4100917877109334, 0.0707312888348917, -23.451267540000000},
             {0.000000000000000, 0.000000000000000, 0.000000000000000, 1.000000000000000}}),
        wrongFourthRowMatrix12_(
            {{0.4157869320692789, -0.9023592869214287, -0.1134413699981963, -2.537896000000000},
             {-0.0173036611331485, -0.1325610914209059, 0.9910237839490472, 3.451273000000000},
             {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917, -23.451267540000000},
             {0.000000000000000, 0.000000000000000, 0.000000000000000, 1.000000000010000}}),
        wrongFourthRowMatrix5_(
            {{0.4157869320692789, -0.9023592869214287, -0.1134413699981963, -2.537896000000000},
             {-0.0173036611331485, -0.1325610914209059, 0.9910237839490472, 3.451273000000000},
             {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917, -23.451267540000000},
             {0.000000000000000, 0.000000000000000, 0.000100000000000, 1.000000000000000}}),
        nonUnitaryQuaternion12_(
            {0.5505666516802112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        nonUnitaryQuaternion5_(
            {0.5506666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        nonOrthogonalMatrix12_(
            {{0.4157869320692789, -0.9023592869214287, -0.1134413699981963},
             {-0.0173036611331485, -0.1325610914209059, 0.9910237839490472},
             {-0.9092974268266818, -0.4100917877109334, -0.0707312888348917}}),
        nonOrthogonalMatrix5_(
            {{0.4157869320692789, -0.9023592869214287, -0.1134413699981963},
             {-0.0172036611331485, -0.1325610914209059, 0.9910237839490472},
             {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917}}),
        orthogonalNonDet1Matrix12_(-nonOrthogonalMatrix12_),
        orthogonalNonDet1Matrix5_(-nonOrthogonalMatrix5_),
        os_() {
    }

    void SetUp() override {
        sut_.reset(new TaskPose());
        outputHomogeneousTransformationMatrix_ = Eigen::Matrix4d::Identity();
        outputPosition_ = Eigen::Vector3d::Zero();
        outputOrientation_ = Orientation();
        outputQuaternion_ = Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0});
        outputRotationMatrix_ = Eigen::Matrix3d::Identity();
        outputAngleAxis_ = Eigen::AngleAxisd(0.0, Eigen::Vector3d({1.0, 0.0, 0.0}));
        outputCardanXYZ_ = CardanXYZ();
        outputEulerZXZ_ = EulerZXZ();
        os_ = std::stringstream();
    }

    bool areAlmostEqualEigenMatrix4d(
        const Eigen::Matrix4d& matrix1,
        const Eigen::Matrix4d& matrix2,
        const double& accuracy = 1e-12) {
        for (std::size_t i = 0; i < 4; i++) {
            for (std::size_t j = 0; j < 4; j++) {
                if (!(abs(matrix1(i, j) - matrix2(i, j)) < accuracy)) {
                    return false;
                }
            }
        }
        return true;
    }

    bool areAlmostEqualEigenVector3d(
        const Eigen::Vector3d& vector1,
        const Eigen::Vector3d& vector2,
        const double& accuracy = 1e-12) {
        for (std::size_t i = 0; i < 3; i++) {
            if (!(abs(vector1(i) - vector2(i)) < accuracy)) {
                return false;
            }
        }
        return true;
    }

    std::unique_ptr<TaskPose> sut_;
    const Eigen::Matrix4d inputHomogeneousTransformationMatrix1_;
    const Eigen::Vector3d inputPosition1_;
    const Eigen::Quaterniond inputQuaternion1_;
    const Eigen::Matrix3d inputRotationMatrix1_;
    const Eigen::AngleAxisd inputAngleAxis1_;
    const CardanXYZ inputCardanXYZ1_;
    const EulerZXZ inputEulerZXZ1_;

    const Eigen::Matrix4d inputHomogeneousTransformationMatrix2_;
    const Eigen::Vector3d inputPosition2_;
    const Eigen::Quaterniond inputQuaternion2_;
    const Eigen::Matrix3d inputRotationMatrix2_;
    const Eigen::AngleAxisd inputAngleAxis2_;
    const CardanXYZ inputCardanXYZ2_;
    const EulerZXZ inputEulerZXZ2_;

    const Orientation inputOrientationQuaternion1_;
    const Orientation inputOrientationMatrix1_;
    const Orientation inputOrientationAngleAxis1_;
    const Orientation inputOrientationCardanXYZ1_;
    const Orientation inputOrientationEulerZXZ1_;

    const Orientation inputOrientationQuaternion2_;
    const Orientation inputOrientationMatrix2_;
    const Orientation inputOrientationAngleAxis2_;
    const Orientation inputOrientationCardanXYZ2_;
    const Orientation inputOrientationEulerZXZ2_;

    const Eigen::Matrix4d nonOrthogonalMatrixPart12_;
    const Eigen::Matrix4d nonOrthogonalMatrixPart5_;
    const Eigen::Matrix4d orthogonalNonDet1MatrixPart12_;
    const Eigen::Matrix4d orthogonalNonDet1MatrixPart5_;
    const Eigen::Matrix4d wrongFourthRowMatrix12_;
    const Eigen::Matrix4d wrongFourthRowMatrix5_;

    const Eigen::Quaterniond nonUnitaryQuaternion12_;
    const Eigen::Quaterniond nonUnitaryQuaternion5_;

    const Eigen::Matrix3d nonOrthogonalMatrix12_;
    const Eigen::Matrix3d nonOrthogonalMatrix5_;
    const Eigen::Matrix3d orthogonalNonDet1Matrix12_;
    const Eigen::Matrix3d orthogonalNonDet1Matrix5_;

    Eigen::Matrix4d outputHomogeneousTransformationMatrix_;
    Eigen::Vector3d outputPosition_;
    Orientation outputOrientation_;
    Eigen::Quaterniond outputQuaternion_;
    Eigen::Matrix3d outputRotationMatrix_;
    Eigen::AngleAxisd outputAngleAxis_;
    CardanXYZ outputCardanXYZ_;
    EulerZXZ outputEulerZXZ_;

    std::stringstream os_;
};

TEST_F(TaskPoseShould, HaveCorrectPositionDefaultConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose()));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(outputPosition_, Eigen::Vector3d::Zero()));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(outputQuaternion_, Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0})));
}

TEST_F(TaskPoseShould, HaveCorrectHomegeneousTransformationMatrixConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputHomogeneousTransformationMatrix1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix1_, outputHomogeneousTransformationMatrix_));
    ASSERT_THROW(sut_.reset(new TaskPose(nonOrthogonalMatrixPart12_)), std::invalid_argument);
    ASSERT_THROW(
        sut_.reset(new TaskPose(nonOrthogonalMatrixPart12_, 1e-12)), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(nonOrthogonalMatrixPart12_, 1e-11)));
    ASSERT_THROW(sut_.reset(new TaskPose(nonOrthogonalMatrixPart5_, 1e-5)), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(nonOrthogonalMatrixPart5_, 1e-4)));
    ASSERT_THROW(sut_.reset(new TaskPose(orthogonalNonDet1MatrixPart12_)), std::invalid_argument);
    ASSERT_THROW(
        sut_.reset(new TaskPose(orthogonalNonDet1MatrixPart12_, 1e-12)), std::invalid_argument);
    ASSERT_THROW(
        sut_.reset(new TaskPose(orthogonalNonDet1MatrixPart12_, 1e-11)), std::invalid_argument);
    ASSERT_THROW(
        sut_.reset(new TaskPose(orthogonalNonDet1MatrixPart5_, 1e-5)), std::invalid_argument);
    ASSERT_THROW(
        sut_.reset(new TaskPose(orthogonalNonDet1MatrixPart5_, 1e-4)), std::invalid_argument);
    ASSERT_THROW(sut_.reset(new TaskPose(wrongFourthRowMatrix12_)), std::invalid_argument);
    ASSERT_THROW(sut_.reset(new TaskPose(wrongFourthRowMatrix12_, 1e-12)), std::invalid_argument);
    ASSERT_THROW(sut_.reset(new TaskPose(wrongFourthRowMatrix12_, 1e-11)), std::invalid_argument);
    ASSERT_THROW(sut_.reset(new TaskPose(wrongFourthRowMatrix5_, 1e-5)), std::invalid_argument);
    ASSERT_THROW(sut_.reset(new TaskPose(wrongFourthRowMatrix5_, 1e-4)), std::invalid_argument);
}

TEST_F(TaskPoseShould, HaveCorrectPositionOrientationConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputOrientationQuaternion1_)));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion1_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputOrientationMatrix1_)));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix1_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputOrientationAngleAxis1_)));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis1_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputOrientationCardanXYZ1_)));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ1_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputOrientationEulerZXZ1_)));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ1_, outputEulerZXZ_));
}

TEST_F(TaskPoseShould, HaveCorrectPositionQuaternionConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion1_, outputQuaternion_));
    ASSERT_THROW(
        sut_.reset(new TaskPose(inputPosition1_, nonUnitaryQuaternion12_)), std::invalid_argument);
    ASSERT_THROW(
        sut_.reset(new TaskPose(inputPosition1_, nonUnitaryQuaternion12_, 1e-12)),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, nonUnitaryQuaternion12_, 1e-11)));
    ASSERT_THROW(
        sut_.reset(new TaskPose(inputPosition1_, nonUnitaryQuaternion5_, 1e-5)),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, nonUnitaryQuaternion5_, 1e-4)));
}

TEST_F(TaskPoseShould, HaveCorrectPositionMatrixConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix1_, outputRotationMatrix_));
    ASSERT_THROW(
        sut_.reset(new TaskPose(inputPosition1_, nonOrthogonalMatrix12_)), std::invalid_argument);
    ASSERT_THROW(
        sut_.reset(new TaskPose(inputPosition1_, nonOrthogonalMatrix12_, 1e-12)),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, nonOrthogonalMatrix12_, 1e-11)));
    ASSERT_THROW(
        sut_.reset(new TaskPose(inputPosition1_, nonOrthogonalMatrix5_, 1e-5)),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, nonOrthogonalMatrix5_, 1e-4)));
    ASSERT_THROW(
        sut_.reset(new TaskPose(inputPosition1_, orthogonalNonDet1Matrix12_)),
        std::invalid_argument);
    ASSERT_THROW(
        sut_.reset(new TaskPose(inputPosition1_, orthogonalNonDet1Matrix12_, 1e-12)),
        std::invalid_argument);
    ASSERT_THROW(
        sut_.reset(new TaskPose(inputPosition1_, orthogonalNonDet1Matrix12_, 1e-11)),
        std::invalid_argument);
    ASSERT_THROW(
        sut_.reset(new TaskPose(inputPosition1_, orthogonalNonDet1Matrix5_, 1e-5)),
        std::invalid_argument);
    ASSERT_THROW(
        sut_.reset(new TaskPose(inputPosition1_, orthogonalNonDet1Matrix5_, 1e-4)),
        std::invalid_argument);
}

TEST_F(TaskPoseShould, HaveCorrectPositionAngleAxisConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis1_, outputAngleAxis_));
}

TEST_F(TaskPoseShould, HaveCorrectPositionCardanXYZConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ1_, outputCardanXYZ_));
}

TEST_F(TaskPoseShould, HaveCorrectPositionEulerZXZConstructor) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ1_, outputEulerZXZ_));
}

TEST_F(TaskPoseShould, HaveCorrectHomegeneousTransformationMatrixSetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(
        sut_->setHomogeneousTransformationMatrix(inputHomogeneousTransformationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_, 1e-11),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart5_, 1e-4),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_, 1e-11),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix5_, 1e-4),
        std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(
        sut_->setHomogeneousTransformationMatrix(inputHomogeneousTransformationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_, 1e-11),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart5_, 1e-4),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_, 1e-11),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix5_, 1e-4),
        std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(
        sut_->setHomogeneousTransformationMatrix(inputHomogeneousTransformationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_, 1e-11),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart5_, 1e-4),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_, 1e-11),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix5_, 1e-4),
        std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(
        sut_->setHomogeneousTransformationMatrix(inputHomogeneousTransformationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_, 1e-11),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart5_, 1e-4),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_, 1e-11),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix5_, 1e-4),
        std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(
        sut_->setHomogeneousTransformationMatrix(inputHomogeneousTransformationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_, 1e-11),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart5_, 1e-4),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix12_, 1e-11),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(wrongFourthRowMatrix5_, 1e-4),
        std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputHomogeneousTransformationMatrix1_)));
    ASSERT_NO_THROW(
        sut_->setHomogeneousTransformationMatrix(inputHomogeneousTransformationMatrix2_));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_),
        std::invalid_argument);
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_, 1e-12),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart12_, 1e-11));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart5_, 1e-5),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_->setHomogeneousTransformationMatrix(nonOrthogonalMatrixPart5_, 1e-4));
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_),
        std::invalid_argument);
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_, 1e-12),
        std::invalid_argument);
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart12_, 1e-11),
        std::invalid_argument);
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart5_, 1e-5),
        std::invalid_argument);
    ASSERT_THROW(
        sut_->setHomogeneousTransformationMatrix(orthogonalNonDet1MatrixPart5_, 1e-4),
        std::invalid_argument);
    ASSERT_NO_THROW(
        sut_->setHomogeneousTransformationMatrix(inputHomogeneousTransformationMatrix1_));
    ASSERT_NO_THROW(
        sut_->setHomogeneousTransformationMatrix(inputHomogeneousTransformationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));
}

TEST_F(TaskPoseShould, HaveCorrectPositionSetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition2_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion1_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition2_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix1_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition2_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis1_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition2_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ1_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition2_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ1_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition1_));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition2_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion1_, outputQuaternion_));
}

TEST_F(TaskPoseShould, HaveCorrectOrientationSetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationQuaternion2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationQuaternion2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationQuaternion2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationQuaternion2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationQuaternion2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationMatrix2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationMatrix2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationMatrix2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationMatrix2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationMatrix2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationAngleAxis2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationAngleAxis2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationAngleAxis2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationAngleAxis2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationAngleAxis2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationCardanXYZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationCardanXYZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationCardanXYZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationCardanXYZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationCardanXYZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationEulerZXZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationEulerZXZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationEulerZXZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationEulerZXZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationEulerZXZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationQuaternion2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationQuaternion1_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationQuaternion2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    outputOrientation_ = sut_->getOrientation();
    ASSERT_TRUE(areAlmostEqual(inputOrientationQuaternion2_, outputOrientation_));
}

TEST_F(TaskPoseShould, HaveCorrectQuaternionSetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-5), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-5), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-5), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-5), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-5), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_), std::invalid_argument);
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-11));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-4));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion1_));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));
}

TEST_F(TaskPoseShould, HaveCorrectRotationMatrixSetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_, 1e-11), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix5_, 1e-4), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_, 1e-11), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix5_, 1e-4), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_, 1e-11), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix5_, 1e-4), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_, 1e-11), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix5_, 1e-4), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_, 1e-11), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix5_, 1e-4), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_), std::invalid_argument);
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix12_, 1e-11));
    ASSERT_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_->setRotationMatrix(nonOrthogonalMatrix5_, 1e-4));
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_), std::invalid_argument);
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_, 1e-12), std::invalid_argument);
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix12_, 1e-11), std::invalid_argument);
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix5_, 1e-5), std::invalid_argument);
    ASSERT_THROW(sut_->setRotationMatrix(orthogonalNonDet1Matrix5_, 1e-4), std::invalid_argument);
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix1_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));
}

TEST_F(TaskPoseShould, HaveCorrectAngleAxisSetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis1_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));
}

TEST_F(TaskPoseShould, HaveCorrectCardanXYZSetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ1_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));
}

TEST_F(TaskPoseShould, HaveCorrectEulerZXZSetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ1_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));
}

TEST_F(TaskPoseShould, HaveCorrectHomogeneousTransformationMatrixGetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputHomogeneousTransformationMatrix_ = sut_->getHomogeneousTransformationMatrix();
    ASSERT_TRUE(areAlmostEqualEigenMatrix4d(
        inputHomogeneousTransformationMatrix2_, outputHomogeneousTransformationMatrix_));
}

TEST_F(TaskPoseShould, HaveCorrectPositionGetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition1_, outputPosition_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition2_, outputPosition_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition2_, outputPosition_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition2_, outputPosition_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition2_, outputPosition_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setPosition(inputPosition2_));
    outputPosition_ = sut_->getPosition();
    ASSERT_TRUE(areAlmostEqualEigenVector3d(inputPosition2_, outputPosition_));
}

TEST_F(TaskPoseShould, HaveCorrectOrientationGetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputOrientation_ = sut_->getOrientation();
    ASSERT_TRUE(areAlmostEqual(inputOrientationQuaternion2_, outputOrientation_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputOrientation_ = sut_->getOrientation();
    ASSERT_TRUE(areAlmostEqual(inputOrientationMatrix2_, outputOrientation_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputOrientation_ = sut_->getOrientation();
    ASSERT_TRUE(areAlmostEqual(inputOrientationAngleAxis2_, outputOrientation_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputOrientation_ = sut_->getOrientation();
    ASSERT_TRUE(areAlmostEqual(inputOrientationCardanXYZ2_, outputOrientation_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputOrientation_ = sut_->getOrientation();
    ASSERT_TRUE(areAlmostEqual(inputOrientationEulerZXZ2_, outputOrientation_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationQuaternion2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationMatrix2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationAngleAxis2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationEulerZXZ2_));
    outputOrientation_ = sut_->getOrientation();
    ASSERT_TRUE(areAlmostEqual(inputOrientationQuaternion2_, outputOrientation_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationQuaternion2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationMatrix2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationAngleAxis2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationEulerZXZ2_));
    outputOrientation_ = sut_->getOrientation();
    ASSERT_TRUE(areAlmostEqual(inputOrientationMatrix2_, outputOrientation_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationQuaternion2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationMatrix2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationAngleAxis2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationEulerZXZ2_));
    outputOrientation_ = sut_->getOrientation();
    ASSERT_TRUE(areAlmostEqual(inputOrientationAngleAxis2_, outputOrientation_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationQuaternion2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationMatrix2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationAngleAxis2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationEulerZXZ2_));
    outputOrientation_ = sut_->getOrientation();
    ASSERT_TRUE(areAlmostEqual(inputOrientationCardanXYZ2_, outputOrientation_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationQuaternion2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationMatrix2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationAngleAxis2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setOrientation(inputOrientationEulerZXZ2_));
    outputOrientation_ = sut_->getOrientation();
    ASSERT_TRUE(areAlmostEqual(inputOrientationEulerZXZ2_, outputOrientation_));
}

TEST_F(TaskPoseShould, HaveCorrectQuaternionGetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));
}

TEST_F(TaskPoseShould, HaveCorrectRotationMatrixGetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputRotationMatrix_ = sut_->getRotationMatrix();
    ASSERT_TRUE(areAlmostEqual(inputRotationMatrix2_, outputRotationMatrix_));
}

TEST_F(TaskPoseShould, HaveCorrectAngleAxisGetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));
}

TEST_F(TaskPoseShould, HaveCorrectCardanXYZGetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));
}

TEST_F(TaskPoseShould, HaveCorrectEulerZXZGetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    EXPECT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));
}

TEST_F(TaskPoseShould, HaveCorrectOrientationRepresentationGetter) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    ASSERT_NO_THROW(sut_->setRotationMatrix(inputRotationMatrix2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Quaternion);
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::Matrix);
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::AngleAxis);
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::CardanXYZ);
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);

    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getOrientationRepresentation(), OrientationRepresentation::EulerZXZ);
}

TEST_F(TaskPoseShould, HaveCorrectStreamOperator) {
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputQuaternion1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "Position:\n"
        " -2.537896000000000\n"
        "  3.451273000000000\n"
        "-23.451267540000000\n"
        "Orientation:\n"
        "Quaternion:\n"
        "w:  0.550566651690211\n"
        "x: -0.636215237228148\n"
        "y:  0.361380431590003\n"
        "z:  0.401883960402980");
    os_ = std::stringstream();
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputRotationMatrix1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "Position:\n"
        " -2.537896000000000\n"
        "  3.451273000000000\n"
        "-23.451267540000000\n"
        "Orientation:\n"
        "Matrix:\n"
        " 0.415786932069279 -0.902359286921429 -0.113441369998196\n"
        "-0.017303661133149 -0.132561091420906  0.991023783949047\n"
        "-0.909297426825682 -0.410091787710933 -0.070731288834892");
    os_ = std::stringstream();
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputAngleAxis1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "Position:\n"
        " -2.537896000000000\n"
        "  3.451273000000000\n"
        "-23.451267540000000\n"
        "Orientation:\n"
        "Angle:\n"
        "1.975506892474911\n"
        "Axis:\n"
        "-0.762124984825468\n"
        " 0.432899182266813\n"
        " 0.481418534642680");
    os_ = std::stringstream();
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputCardanXYZ1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "Position:\n"
        " -2.537896000000000\n"
        "  3.451273000000000\n"
        "-23.451267540000000\n"
        "Orientation:\n"
        "CardanXYZ:\n"
        "X: 1.400000000000000\n"
        "Y: 2.000000000000000\n"
        "Z: 3.100000000000000");
    os_ = std::stringstream();
    ASSERT_NO_THROW(sut_.reset(new TaskPose(inputPosition1_, inputEulerZXZ1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "Position:\n"
        " -2.537896000000000\n"
        "  3.451273000000000\n"
        "-23.451267540000000\n"
        "Orientation:\n"
        "EulerZXZ:\n"
        "Z:  1.147112346463949\n"
        "X: -1.641586725909306\n"
        "Z:  0.113972795042484");
}
