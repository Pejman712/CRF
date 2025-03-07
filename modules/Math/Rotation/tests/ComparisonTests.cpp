/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/Comparison.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::math::rotation::areAlmostEqual;

using crf::math::rotation::Rotation;
using crf::math::rotation::RotationRepresentation;

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;

class ComparisonShould : public ::testing::Test {
 protected:
    ComparisonShould() :
        quaternion1_(
            {0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        minusQuaternion1_(
            {-0.5505666516902112, 0.6362152372281484, -0.3613804315900029, -0.4018839604029799}),
        matrix1_(
            {{0.4157869320692789, -0.9023592869214287, -0.1134413699981963},
             {-0.0173036611331485, -0.1325610914209059, 0.9910237839490472},
             {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917}}),
        angleAxis1_(
            1.9755068924749106,
            Eigen::Vector3d({-0.7621249848254679, 0.4328991822668132, 0.4814185346426796})),
        cardanXYZ1_({1.4, 2.0, 3.1}),
        eulerZXZ1_({1.1471123464639490, -1.6415867259093058, 0.1139727950424842}),
        quaternion2_(
            {0.3232781413160111, 0.5504467585659593, 0.7697351635084868, 0.0027179753598398}),
        matrix2_(
            {{-0.1849992186629872, 0.8456391273700271, 0.5006693073825711},
             {0.8491537754599140, 0.3940019571883435, -0.3517105675892164},
             {-0.4946849044758272, 0.3600790524212897, -0.7909677119144169}}),
        angleAxis2_(
            2.4832094549611980,
            Eigen::Vector3d({0.5816806901502302, 0.8134121496309343, 0.0028722010957822})),
        cardanXYZ2_({2.7143908348394090, 0.5174722288733832, 1.7853074093465211}),
        eulerZXZ2_({-0.9415926535897930, 2.4831853071795869, 0.9584073464102065}),
        quaternion1Diff7_(
            {0.550566622140754, -0.636215266863117, 0.361380448423139, 0.401883938833470}),
        minusQuaternion1Diff7_(
            {-0.550566622140754, 0.636215266863117, -0.361380448423139, -0.401883938833470}),
        matrix1Diff7_(
            {{0.415786942410374, -0.902359282257610, -0.113441369193796},
             {-0.017303751472951, -0.132561132164022, 0.991023776921803},
             {-0.909297420377950, -0.410091784803016, -0.070731388584432}}),
        angleAxis1Diff7_(
            1.975506963269755,
            Eigen::Vector3d({-0.762125002533116, 0.432899192325041, 0.481418497565502})),
        cardanXYZ1Diff7_({1.399999762602052, 2.000000015493885, 3.099999784134658}),
        eulerZXZ1Diff7_({1.1471123464639490, -1.6415868259093058, 0.1139727950424842}),
        rotationQuaternion1_(quaternion1_),
        rotationMatrix1_(matrix1_),
        rotationAngleAxis1_(angleAxis1_),
        rotationCardanXYZ1_(cardanXYZ1_),
        rotationEulerZXZ1_(eulerZXZ1_),
        rotationQuaternion2_(quaternion2_),
        rotationMatrix2_(matrix2_),
        rotationAngleAxis2_(angleAxis2_),
        rotationCardanXYZ2_(cardanXYZ2_),
        rotationEulerZXZ2_(eulerZXZ2_),
        rotationQuaternion1Diff7_(quaternion1Diff7_),
        rotationMatrix1Diff7_(matrix1Diff7_),
        rotationAngleAxis1Diff7_(angleAxis1Diff7_),
        rotationCardanXYZ1Diff7_(cardanXYZ1Diff7_),
        rotationEulerZXZ1Diff7_(eulerZXZ1Diff7_) {
    }

    const Eigen::Quaterniond quaternion1_;
    const Eigen::Quaterniond minusQuaternion1_;
    const Eigen::Matrix3d matrix1_;
    const Eigen::AngleAxisd angleAxis1_;
    const CardanXYZ cardanXYZ1_;
    const EulerZXZ eulerZXZ1_;

    const Eigen::Quaterniond quaternion2_;
    const Eigen::Matrix3d matrix2_;
    const Eigen::AngleAxisd angleAxis2_;
    const CardanXYZ cardanXYZ2_;
    const EulerZXZ eulerZXZ2_;

    const Eigen::Quaterniond quaternion1Diff7_;
    const Eigen::Quaterniond minusQuaternion1Diff7_;
    const Eigen::Matrix3d matrix1Diff7_;
    const Eigen::AngleAxisd angleAxis1Diff7_;
    const CardanXYZ cardanXYZ1Diff7_;
    const EulerZXZ eulerZXZ1Diff7_;

    const Rotation rotationQuaternion1_;
    const Rotation rotationMatrix1_;
    const Rotation rotationAngleAxis1_;
    const Rotation rotationCardanXYZ1_;
    const Rotation rotationEulerZXZ1_;

    const Rotation rotationQuaternion2_;
    const Rotation rotationMatrix2_;
    const Rotation rotationAngleAxis2_;
    const Rotation rotationCardanXYZ2_;
    const Rotation rotationEulerZXZ2_;

    const Rotation rotationQuaternion1Diff7_;
    const Rotation rotationMatrix1Diff7_;
    const Rotation rotationAngleAxis1Diff7_;
    const Rotation rotationCardanXYZ1Diff7_;
    const Rotation rotationEulerZXZ1Diff7_;
};

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameQuaternionArguments) {
    ASSERT_TRUE(areAlmostEqual(quaternion1_, quaternion1_));
    ASSERT_TRUE(areAlmostEqual(quaternion1_, minusQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(minusQuaternion1_, quaternion1_));
    ASSERT_TRUE(areAlmostEqual(quaternion1_, quaternion1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(quaternion1_, minusQuaternion1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(minusQuaternion1_, quaternion1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(quaternion1Diff7_, quaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(quaternion1Diff7_, minusQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(minusQuaternion1Diff7_, quaternion1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentQuaternionArguments) {
    ASSERT_FALSE(areAlmostEqual(quaternion1_, quaternion2_));
    ASSERT_FALSE(areAlmostEqual(quaternion2_, quaternion1_));
    ASSERT_FALSE(areAlmostEqual(quaternion1_, quaternion1Diff7_));
    ASSERT_FALSE(areAlmostEqual(quaternion1_, minusQuaternion1Diff7_));
    ASSERT_FALSE(areAlmostEqual(minusQuaternion1_, quaternion1Diff7_));
    ASSERT_FALSE(areAlmostEqual(quaternion1Diff7_, quaternion1_));
    ASSERT_FALSE(areAlmostEqual(quaternion1Diff7_, minusQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(minusQuaternion1Diff7_, quaternion1_));
    ASSERT_FALSE(areAlmostEqual(quaternion1_, quaternion1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(quaternion1_, minusQuaternion1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(minusQuaternion1_, quaternion1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(quaternion1Diff7_, quaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(quaternion1Diff7_, minusQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(minusQuaternion1Diff7_, quaternion1_, 1e-8));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameMatrixArguments) {
    ASSERT_TRUE(areAlmostEqual(matrix1_, matrix1_));
    ASSERT_TRUE(areAlmostEqual(matrix1_, matrix1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(matrix1Diff7_, matrix1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentMatrixArguments) {
    ASSERT_FALSE(areAlmostEqual(matrix1_, matrix2_));
    ASSERT_FALSE(areAlmostEqual(matrix2_, matrix1_));
    ASSERT_FALSE(areAlmostEqual(matrix1_, matrix1Diff7_));
    ASSERT_FALSE(areAlmostEqual(matrix1Diff7_, matrix1_));
    ASSERT_FALSE(areAlmostEqual(matrix1_, matrix1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(matrix1Diff7_, matrix1_, 1e-8));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameAngleAxisArguments) {
    ASSERT_TRUE(areAlmostEqual(angleAxis1_, angleAxis1_));
    ASSERT_TRUE(areAlmostEqual(angleAxis1_, angleAxis1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(angleAxis1Diff7_, angleAxis1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentAngleAxisArguments) {
    ASSERT_FALSE(areAlmostEqual(angleAxis1_, angleAxis2_));
    ASSERT_FALSE(areAlmostEqual(angleAxis2_, angleAxis1_));
    ASSERT_FALSE(areAlmostEqual(angleAxis1_, angleAxis1Diff7_));
    ASSERT_FALSE(areAlmostEqual(angleAxis1Diff7_, angleAxis1_));
    ASSERT_FALSE(areAlmostEqual(angleAxis1_, angleAxis1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(angleAxis1Diff7_, angleAxis1_, 1e-8));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameCardanXYZArguments) {
    ASSERT_TRUE(areAlmostEqual(cardanXYZ1_, cardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(cardanXYZ1_, cardanXYZ1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(cardanXYZ1Diff7_, cardanXYZ1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentCardanXYZArguments) {
    ASSERT_FALSE(areAlmostEqual(cardanXYZ1_, cardanXYZ2_));
    ASSERT_FALSE(areAlmostEqual(cardanXYZ2_, cardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(cardanXYZ1_, cardanXYZ1Diff7_));
    ASSERT_FALSE(areAlmostEqual(cardanXYZ1Diff7_, cardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(cardanXYZ1_, cardanXYZ1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(cardanXYZ1Diff7_, cardanXYZ1_, 1e-8));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameEulerZXZArguments) {
    ASSERT_TRUE(areAlmostEqual(eulerZXZ1_, eulerZXZ1_));
    ASSERT_TRUE(areAlmostEqual(eulerZXZ1_, eulerZXZ1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(eulerZXZ1Diff7_, eulerZXZ1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentEulerZXZArguments) {
    ASSERT_FALSE(areAlmostEqual(eulerZXZ1_, eulerZXZ2_));
    ASSERT_FALSE(areAlmostEqual(eulerZXZ2_, eulerZXZ1_));
    ASSERT_FALSE(areAlmostEqual(eulerZXZ1_, eulerZXZ1Diff7_));
    ASSERT_FALSE(areAlmostEqual(eulerZXZ1Diff7_, eulerZXZ1_));
    ASSERT_FALSE(areAlmostEqual(eulerZXZ1_, eulerZXZ1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(eulerZXZ1Diff7_, eulerZXZ1_, 1e-8));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameRotationArguments) {
    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1_, rotationQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1_, rotationMatrix1_));
    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1_, rotationAngleAxis1_));
    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1_, rotationCardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1_, rotationEulerZXZ1_));

    ASSERT_TRUE(areAlmostEqual(rotationMatrix1_, rotationQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix1_, rotationMatrix1_));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix1_, rotationAngleAxis1_));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix1_, rotationCardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix1_, rotationEulerZXZ1_));

    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1_, rotationQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1_, rotationMatrix1_));
    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1_, rotationAngleAxis1_));
    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1_, rotationCardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1_, rotationEulerZXZ1_));

    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1_, rotationQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1_, rotationMatrix1_));
    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1_, rotationAngleAxis1_));
    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1_, rotationCardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1_, rotationEulerZXZ1_));

    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1_, rotationQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1_, rotationMatrix1_));
    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1_, rotationAngleAxis1_));
    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1_, rotationCardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1_, rotationEulerZXZ1_));

    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1_, rotationQuaternion1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1_, rotationMatrix1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1_, rotationAngleAxis1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1_, rotationCardanXYZ1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1_, rotationEulerZXZ1Diff7_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(rotationMatrix1_, rotationQuaternion1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix1_, rotationMatrix1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix1_, rotationAngleAxis1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix1_, rotationCardanXYZ1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix1_, rotationEulerZXZ1Diff7_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1_, rotationQuaternion1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1_, rotationMatrix1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1_, rotationAngleAxis1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1_, rotationCardanXYZ1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1_, rotationEulerZXZ1Diff7_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1_, rotationQuaternion1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1_, rotationMatrix1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1_, rotationAngleAxis1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1_, rotationCardanXYZ1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1_, rotationEulerZXZ1Diff7_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1_, rotationQuaternion1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1_, rotationMatrix1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1_, rotationAngleAxis1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1_, rotationCardanXYZ1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1_, rotationEulerZXZ1Diff7_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1Diff7_, rotationQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1Diff7_, rotationMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1Diff7_, rotationAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1Diff7_, rotationCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationQuaternion1Diff7_, rotationEulerZXZ1_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(rotationMatrix1Diff7_, rotationQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix1Diff7_, rotationMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix1Diff7_, rotationAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix1Diff7_, rotationCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix1Diff7_, rotationEulerZXZ1_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationEulerZXZ1_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationEulerZXZ1_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationEulerZXZ1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentRotationArguments) {
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationQuaternion2_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationMatrix2_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationAngleAxis2_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationCardanXYZ2_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationEulerZXZ2_));

    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationQuaternion2_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationMatrix2_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationAngleAxis2_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationCardanXYZ2_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationEulerZXZ2_));

    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationQuaternion2_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationMatrix2_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationAngleAxis2_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationCardanXYZ2_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationEulerZXZ2_));

    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationQuaternion2_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationMatrix2_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationAngleAxis2_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationCardanXYZ2_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationEulerZXZ2_));

    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationQuaternion2_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationMatrix2_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationAngleAxis2_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationCardanXYZ2_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationEulerZXZ2_));

    ASSERT_FALSE(areAlmostEqual(rotationQuaternion2_, rotationQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion2_, rotationMatrix1_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion2_, rotationAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion2_, rotationCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion2_, rotationEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(rotationMatrix2_, rotationQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix2_, rotationMatrix1_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix2_, rotationAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix2_, rotationCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix2_, rotationEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis2_, rotationQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis2_, rotationMatrix1_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis2_, rotationAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis2_, rotationCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis2_, rotationEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ2_, rotationQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ2_, rotationMatrix1_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ2_, rotationAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ2_, rotationCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ2_, rotationEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ2_, rotationQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ2_, rotationMatrix1_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ2_, rotationAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ2_, rotationCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ2_, rotationEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationQuaternion1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationMatrix1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationAngleAxis1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationCardanXYZ1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationEulerZXZ1Diff7_));

    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationQuaternion1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationMatrix1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationAngleAxis1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationCardanXYZ1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationEulerZXZ1Diff7_));

    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationQuaternion1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationMatrix1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationAngleAxis1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationCardanXYZ1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationEulerZXZ1Diff7_));

    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationQuaternion1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationMatrix1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationAngleAxis1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationCardanXYZ1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationEulerZXZ1Diff7_));

    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationQuaternion1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationMatrix1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationAngleAxis1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationCardanXYZ1Diff7_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationEulerZXZ1Diff7_));

    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1Diff7_, rotationQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1Diff7_, rotationMatrix1_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1Diff7_, rotationAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1Diff7_, rotationCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1Diff7_, rotationEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(rotationMatrix1Diff7_, rotationQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1Diff7_, rotationMatrix1_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1Diff7_, rotationAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1Diff7_, rotationCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1Diff7_, rotationEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationMatrix1_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationMatrix1_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationMatrix1_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationQuaternion1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationMatrix1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationAngleAxis1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationCardanXYZ1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1_, rotationEulerZXZ1Diff7_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationQuaternion1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationMatrix1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationAngleAxis1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationCardanXYZ1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1_, rotationEulerZXZ1Diff7_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationQuaternion1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationMatrix1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationAngleAxis1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationCardanXYZ1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1_, rotationEulerZXZ1Diff7_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationQuaternion1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationMatrix1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationAngleAxis1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationCardanXYZ1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1_, rotationEulerZXZ1Diff7_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationQuaternion1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationMatrix1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationAngleAxis1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationCardanXYZ1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1_, rotationEulerZXZ1Diff7_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1Diff7_, rotationQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1Diff7_, rotationMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1Diff7_, rotationAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1Diff7_, rotationCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationQuaternion1Diff7_, rotationEulerZXZ1_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(rotationMatrix1Diff7_, rotationQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1Diff7_, rotationMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1Diff7_, rotationAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1Diff7_, rotationCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationMatrix1Diff7_, rotationEulerZXZ1_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationAngleAxis1Diff7_, rotationEulerZXZ1_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationCardanXYZ1Diff7_, rotationEulerZXZ1_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(rotationEulerZXZ1Diff7_, rotationEulerZXZ1_, 1e-8));
}
