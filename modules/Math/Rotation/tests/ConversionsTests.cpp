/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/Comparison.hpp"
#include "Rotation/Conversions.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::math::rotation::areAlmostEqual;

using crf::math::rotation::quaternionFromMatrix;
using crf::math::rotation::quaternionFromAngleAxis;
using crf::math::rotation::quaternionFromCardanXYZ;
using crf::math::rotation::quaternionFromEulerZXZ;
using crf::math::rotation::matrixFromQuaternion;
using crf::math::rotation::matrixFromAngleAxis;
using crf::math::rotation::matrixFromCardanXYZ;
using crf::math::rotation::matrixFromEulerZXZ;
using crf::math::rotation::angleAxisFromQuaternion;
using crf::math::rotation::angleAxisFromMatrix;
using crf::math::rotation::angleAxisFromCardanXYZ;
using crf::math::rotation::angleAxisFromEulerZXZ;
using crf::math::rotation::cardanXYZFromQuaternion;
using crf::math::rotation::cardanXYZFromMatrix;
using crf::math::rotation::cardanXYZFromAngleAxis;
using crf::math::rotation::cardanXYZFromEulerZXZ;
using crf::math::rotation::eulerZXZFromQuaternion;
using crf::math::rotation::eulerZXZFromMatrix;
using crf::math::rotation::eulerZXZFromAngleAxis;
using crf::math::rotation::eulerZXZFromCardanXYZ;

using crf::math::rotation::quaternionFromArray;
using crf::math::rotation::arrayFromQuaternion;
using crf::math::rotation::matrixFromArray;
using crf::math::rotation::arrayFromMatrix;
using crf::math::rotation::angleAxisFromArray;
using crf::math::rotation::arrayFromAngleAxis;

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;

class ConversionsShould : public ::testing::Test {
 protected:
    ConversionsShould() :
        quaternion1_(
            {0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        matrix1_(
            {{0.4157869320692789, -0.9023592869214287, -0.1134413699981963},
             {-0.0173036611331485, -0.1325610914209059, 0.9910237839490472},
             {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917}}),
        angleAxis1_(
            1.9755068924749106,
            Eigen::Vector3d({-0.7621249848254679, 0.4328991822668132, 0.4814185346426796})),
        cardanXYZ1_({1.4, 2.0, 3.1}),
        eulerZXZ1_({1.1471123464639490, -1.6415867259093058, 0.1139727950424842}),
        quaternionArray1_(
            {0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        matrixArray1_(
            {std::array<double, 3>({ 0.4157869320692789, -0.9023592869214287, -0.1134413699981963}),
             std::array<double, 3>({-0.0173036611331485, -0.1325610914209059,  0.9910237839490472}),
             std::array<double, 3>({-0.9092974268256818, -0.4100917877109334, -0.0707312888348917})}),  // NOLINT
        angleAxisArray1_(
            {1.9755068924749106, -0.7621249848254679, 0.4328991822668132, 0.4814185346426796}),
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
        quaternionArray2_(
            {0.3232781413160111, 0.5504467585659593, 0.7697351635084868, 0.0027179753598398}),
        matrixArray2_(
            {std::array<double, 3>({-0.1849992186629872,  0.8456391273700271,  0.5006693073825711}),
             std::array<double, 3>({ 0.8491537754599140,  0.3940019571883435, -0.3517105675892164}),
             std::array<double, 3>({-0.4946849044758272,  0.3600790524212897, -0.7909677119144169})}),  // NOLINT
        angleAxisArray2_(
            {2.4832094549611980, 0.5816806901502302, 0.8134121496309343, 0.0028722010957822}),
        quaternion3_({1.000000000000000, 0.000000000000000, 0.000000000000000, 0.000000000000000}),
        matrix3_(
            {{1.000000000000000, -0.000000000000001, 0.000000000000000},
             {0.000000000000001, 1.000000000000000, -0.000000000000001},
             {-0.000000000000000, 0.000000000000001, 1.000000000000000}}),
        angleAxis3_(
            0.000000000000001,
            Eigen::Vector3d({0.754203464417262, 0.315394176029037, 0.575937191009545})),
        cardanXYZ3_({0.000000000000001, 0.000000000000000, 0.000000000000001}),
        eulerZXZ3_({-0.396081441564307, 0.000000000000001, 0.396081441564307}),
        quaternion4_({1.000000000000000, 0.000000000000004, 0.000000000000002, 0.000000000000003}),
        matrix4_(
            {{ 1.000000000000000, -0.000000000000006, 0.000000000000003},
             { 0.000000000000006, 1.000000000000000, -0.000000000000008},
             {-0.000000000000003, 0.000000000000008, 1.000000000000000}}),
        angleAxis4_(
            0.000000000000010,
            Eigen::Vector3d({0.754203464417262, 0.315394176029037, 0.575937191009545})),
        cardanXYZ4_({0.000000000000008, 0.000000000000003, 0.000000000000006}),
        eulerZXZ4_({-0.396081441564302, 0.000000000000008, 0.396081441564307}) {
    }

    const Eigen::Quaterniond quaternion1_;
    const Eigen::Matrix3d matrix1_;
    const Eigen::AngleAxisd angleAxis1_;
    const CardanXYZ cardanXYZ1_;
    const EulerZXZ eulerZXZ1_;

    const std::array<double, 4> quaternionArray1_;
    const std::array<std::array<double, 3>, 3> matrixArray1_;
    const std::array<double, 4> angleAxisArray1_;

    const Eigen::Quaterniond quaternion2_;
    const Eigen::Matrix3d matrix2_;
    const Eigen::AngleAxisd angleAxis2_;
    const CardanXYZ cardanXYZ2_;
    const EulerZXZ eulerZXZ2_;

    const std::array<double, 4> quaternionArray2_;
    const std::array<std::array<double, 3>, 3> matrixArray2_;
    const std::array<double, 4> angleAxisArray2_;

    const Eigen::Quaterniond quaternion3_;
    const Eigen::Matrix3d matrix3_;
    const Eigen::AngleAxisd angleAxis3_;
    const CardanXYZ cardanXYZ3_;
    const EulerZXZ eulerZXZ3_;

    const Eigen::Quaterniond quaternion4_;
    const Eigen::Matrix3d matrix4_;
    const Eigen::AngleAxisd angleAxis4_;
    const CardanXYZ cardanXYZ4_;
    const EulerZXZ eulerZXZ4_;
};

TEST_F(ConversionsShould, GiveCorrectQuaternionFromAnotherRepresentation) {
    ASSERT_TRUE(areAlmostEqual(quaternion1_, quaternionFromMatrix(matrix1_)));
    ASSERT_TRUE(areAlmostEqual(quaternion1_, quaternionFromAngleAxis(angleAxis1_)));
    ASSERT_TRUE(areAlmostEqual(quaternion1_, quaternionFromCardanXYZ(cardanXYZ1_)));
    ASSERT_TRUE(areAlmostEqual(quaternion1_, quaternionFromEulerZXZ(eulerZXZ1_)));

    ASSERT_TRUE(areAlmostEqual(quaternion2_, quaternionFromMatrix(matrix2_)));
    ASSERT_TRUE(areAlmostEqual(quaternion2_, quaternionFromAngleAxis(angleAxis2_)));
    ASSERT_TRUE(areAlmostEqual(quaternion2_, quaternionFromCardanXYZ(cardanXYZ2_)));
    ASSERT_TRUE(areAlmostEqual(quaternion2_, quaternionFromEulerZXZ(eulerZXZ2_)));

    ASSERT_TRUE(areAlmostEqual(quaternion3_, quaternionFromMatrix(matrix3_)));
    ASSERT_TRUE(areAlmostEqual(quaternion3_, quaternionFromAngleAxis(angleAxis3_)));
    ASSERT_TRUE(areAlmostEqual(quaternion3_, quaternionFromCardanXYZ(cardanXYZ3_)));
    ASSERT_TRUE(areAlmostEqual(quaternion3_, quaternionFromEulerZXZ(eulerZXZ3_)));

    ASSERT_TRUE(areAlmostEqual(quaternion4_, quaternionFromMatrix(matrix4_)));
    ASSERT_TRUE(areAlmostEqual(quaternion4_, quaternionFromAngleAxis(angleAxis4_)));
    ASSERT_TRUE(areAlmostEqual(quaternion4_, quaternionFromCardanXYZ(cardanXYZ4_)));
    ASSERT_TRUE(areAlmostEqual(quaternion4_, quaternionFromEulerZXZ(eulerZXZ4_)));
}

TEST_F(ConversionsShould, GiveCorrectMatrixFromAnotherRepresentation) {
    ASSERT_TRUE(areAlmostEqual(matrix1_, matrixFromQuaternion(quaternion1_)));
    ASSERT_TRUE(areAlmostEqual(matrix1_, matrixFromAngleAxis(angleAxis1_)));
    ASSERT_TRUE(areAlmostEqual(matrix1_, matrixFromCardanXYZ(cardanXYZ1_)));
    ASSERT_TRUE(areAlmostEqual(matrix1_, matrixFromEulerZXZ(eulerZXZ1_)));

    ASSERT_TRUE(areAlmostEqual(matrix2_, matrixFromQuaternion(quaternion2_)));
    ASSERT_TRUE(areAlmostEqual(matrix2_, matrixFromAngleAxis(angleAxis2_)));
    ASSERT_TRUE(areAlmostEqual(matrix2_, matrixFromCardanXYZ(cardanXYZ2_)));
    ASSERT_TRUE(areAlmostEqual(matrix2_, matrixFromEulerZXZ(eulerZXZ2_)));

    ASSERT_TRUE(areAlmostEqual(matrix3_, matrixFromQuaternion(quaternion3_)));
    ASSERT_TRUE(areAlmostEqual(matrix3_, matrixFromAngleAxis(angleAxis3_)));
    ASSERT_TRUE(areAlmostEqual(matrix3_, matrixFromCardanXYZ(cardanXYZ3_)));
    ASSERT_TRUE(areAlmostEqual(matrix3_, matrixFromEulerZXZ(eulerZXZ3_)));

    ASSERT_TRUE(areAlmostEqual(matrix4_, matrixFromQuaternion(quaternion4_)));
    ASSERT_TRUE(areAlmostEqual(matrix4_, matrixFromAngleAxis(angleAxis4_)));
    ASSERT_TRUE(areAlmostEqual(matrix4_, matrixFromCardanXYZ(cardanXYZ4_)));
    ASSERT_TRUE(areAlmostEqual(matrix4_, matrixFromEulerZXZ(eulerZXZ4_)));
}

TEST_F(ConversionsShould, GiveCorrectAngleAxisFromAnotherRepresentation) {
    ASSERT_TRUE(areAlmostEqual(angleAxis1_, angleAxisFromQuaternion(quaternion1_)));
    ASSERT_TRUE(areAlmostEqual(angleAxis1_, angleAxisFromMatrix(matrix1_)));
    ASSERT_TRUE(areAlmostEqual(angleAxis1_, angleAxisFromCardanXYZ(cardanXYZ1_)));
    ASSERT_TRUE(areAlmostEqual(angleAxis1_, angleAxisFromEulerZXZ(eulerZXZ1_)));

    ASSERT_TRUE(areAlmostEqual(angleAxis2_, angleAxisFromQuaternion(quaternion2_)));
    ASSERT_TRUE(areAlmostEqual(angleAxis2_, angleAxisFromMatrix(matrix2_)));
    ASSERT_TRUE(areAlmostEqual(angleAxis2_, angleAxisFromCardanXYZ(cardanXYZ2_)));
    ASSERT_TRUE(areAlmostEqual(angleAxis2_, angleAxisFromEulerZXZ(eulerZXZ2_)));

    ASSERT_TRUE(areAlmostEqual(angleAxis3_, angleAxisFromQuaternion(quaternion3_)));
    ASSERT_TRUE(areAlmostEqual(angleAxis3_, angleAxisFromMatrix(matrix3_)));
    ASSERT_TRUE(areAlmostEqual(angleAxis3_, angleAxisFromCardanXYZ(cardanXYZ3_)));
    ASSERT_TRUE(areAlmostEqual(angleAxis3_, angleAxisFromEulerZXZ(eulerZXZ3_)));

    ASSERT_TRUE(areAlmostEqual(angleAxis4_, angleAxisFromQuaternion(quaternion4_)));
    ASSERT_TRUE(areAlmostEqual(angleAxis4_, angleAxisFromMatrix(matrix4_)));
    ASSERT_TRUE(areAlmostEqual(angleAxis4_, angleAxisFromCardanXYZ(cardanXYZ4_)));
    ASSERT_TRUE(areAlmostEqual(angleAxis4_, angleAxisFromEulerZXZ(eulerZXZ4_)));
}

TEST_F(ConversionsShould, GiveCorrectCardanXYZFromAnotherRepresentation) {
    ASSERT_TRUE(areAlmostEqual(cardanXYZ1_, cardanXYZFromQuaternion(quaternion1_)));
    ASSERT_TRUE(areAlmostEqual(cardanXYZ1_, cardanXYZFromMatrix(matrix1_)));
    ASSERT_TRUE(areAlmostEqual(cardanXYZ1_, cardanXYZFromAngleAxis(angleAxis1_)));
    ASSERT_TRUE(areAlmostEqual(cardanXYZ1_, cardanXYZFromEulerZXZ(eulerZXZ1_)));

    ASSERT_TRUE(areAlmostEqual(cardanXYZ2_, cardanXYZFromQuaternion(quaternion2_)));
    ASSERT_TRUE(areAlmostEqual(cardanXYZ2_, cardanXYZFromMatrix(matrix2_)));
    ASSERT_TRUE(areAlmostEqual(cardanXYZ2_, cardanXYZFromAngleAxis(angleAxis2_)));
    ASSERT_TRUE(areAlmostEqual(cardanXYZ2_, cardanXYZFromEulerZXZ(eulerZXZ2_)));

    ASSERT_TRUE(areAlmostEqual(cardanXYZ3_, cardanXYZFromQuaternion(quaternion3_)));
    ASSERT_TRUE(areAlmostEqual(cardanXYZ3_, cardanXYZFromMatrix(matrix3_)));
    ASSERT_TRUE(areAlmostEqual(cardanXYZ3_, cardanXYZFromAngleAxis(angleAxis3_)));
    ASSERT_TRUE(areAlmostEqual(cardanXYZ3_, cardanXYZFromEulerZXZ(eulerZXZ3_)));

    ASSERT_TRUE(areAlmostEqual(cardanXYZ4_, cardanXYZFromQuaternion(quaternion4_)));
    ASSERT_TRUE(areAlmostEqual(cardanXYZ4_, cardanXYZFromMatrix(matrix4_)));
    ASSERT_TRUE(areAlmostEqual(cardanXYZ4_, cardanXYZFromAngleAxis(angleAxis4_)));
    ASSERT_TRUE(areAlmostEqual(cardanXYZ4_, cardanXYZFromEulerZXZ(eulerZXZ4_)));
}

TEST_F(ConversionsShould, GiveCorrectEulerZXZFromAnotherRepresentation) {
    ASSERT_TRUE(areAlmostEqual(eulerZXZ1_, eulerZXZFromQuaternion(quaternion1_)));
    ASSERT_TRUE(areAlmostEqual(eulerZXZ1_, eulerZXZFromMatrix(matrix1_)));
    ASSERT_TRUE(areAlmostEqual(eulerZXZ1_, eulerZXZFromAngleAxis(angleAxis1_)));
    ASSERT_TRUE(areAlmostEqual(eulerZXZ1_, eulerZXZFromCardanXYZ(cardanXYZ1_)));

    ASSERT_TRUE(areAlmostEqual(eulerZXZ2_, eulerZXZFromQuaternion(quaternion2_)));
    ASSERT_TRUE(areAlmostEqual(eulerZXZ2_, eulerZXZFromMatrix(matrix2_)));
    ASSERT_TRUE(areAlmostEqual(eulerZXZ2_, eulerZXZFromAngleAxis(angleAxis2_)));
    ASSERT_TRUE(areAlmostEqual(eulerZXZ2_, eulerZXZFromCardanXYZ(cardanXYZ2_)));
}

TEST_F(ConversionsShould, CorrectlyConvertBetweenQuaternionAndArray) {
    ASSERT_TRUE(areAlmostEqual(quaternion1_, quaternionFromArray(quaternionArray1_)));
    ASSERT_EQ(quaternionArray1_, arrayFromQuaternion(quaternion1_));

    ASSERT_TRUE(areAlmostEqual(quaternion2_, quaternionFromArray(quaternionArray2_)));
    ASSERT_EQ(quaternionArray2_, arrayFromQuaternion(quaternion2_));
}

TEST_F(ConversionsShould, CorrectlyConvertBetweenMatrixAndArray) {
    ASSERT_TRUE(areAlmostEqual(matrix1_, matrixFromArray(matrixArray1_)));
    ASSERT_EQ(matrixArray1_, arrayFromMatrix(matrix1_));

    ASSERT_TRUE(areAlmostEqual(matrix2_, matrixFromArray(matrixArray2_)));
    ASSERT_EQ(matrixArray2_, arrayFromMatrix(matrix2_));
}

TEST_F(ConversionsShould, CorrectlyConvertBetweenAngleAxisAndArray) {
    ASSERT_TRUE(areAlmostEqual(angleAxis1_, angleAxisFromArray(angleAxisArray1_)));
    ASSERT_EQ(angleAxisArray1_, arrayFromAngleAxis(angleAxis1_));

    ASSERT_TRUE(areAlmostEqual(angleAxis2_, angleAxisFromArray(angleAxisArray2_)));
    ASSERT_EQ(angleAxisArray2_, arrayFromAngleAxis(angleAxis2_));
}
