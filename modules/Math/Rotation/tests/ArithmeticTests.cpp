/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/Arithmetic.hpp"
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

using crf::math::rotation::multiply;
using crf::math::rotation::invert;
using crf::math::rotation::angularVelocityFromRotation;
using crf::math::rotation::rotationFromAngularVelocity;

class ArithmeticShould : public ::testing::Test {
 protected:
    ArithmeticShould() :
        identityRotationQuaternion_(Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0})),
        identityRotationMatrix_(Eigen::Matrix3d::Identity()),
        identityRotationAngleAxis_(Eigen::AngleAxisd(0.0, Eigen::Vector3d({1.0, 0.0, 0.0}))),
        identityRotationCardanXYZ_(CardanXYZ({0.0, 0.0, 0.0})),
        identityRotationEulerZXZ_(EulerZXZ({0.0, 0.0, 0.0})),
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
            {{1.000000000000000, -0.000000000000006, 0.000000000000003},
             {0.000000000000006, 1.000000000000000, -0.000000000000008},
             {-0.000000000000003, 0.000000000000008, 1.000000000000000}}),
        angleAxis4_(
            0.000000000000010,
            Eigen::Vector3d({0.754203464417262, 0.315394176029037, 0.575937191009545})),
        cardanXYZ4_({0.000000000000008, 0.000000000000003, 0.000000000000006}),
        eulerZXZ4_({-0.396081441564302, 0.000000000000008, 0.396081441564307}),
        productQuaternion12_(
            {0.248929242610910, -0.210978843434287, 0.763561846555290, -0.557221200495248}),
        productQuaternion21_(
            {0.248929242610910, 0.405745142292985, 0.317671965231668, 0.820054653169320}),
        inverseQuaternion1_(
            {0.550566651690211, 0.636215237228148, -0.361380431590003, -0.401883960402980}),
        inverseQuaternion2_(
            {0.323278141316011, -0.550446758565959, -0.769735163508487, -0.002717975359840}),
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
        rotationQuaternion3_(quaternion3_),
        rotationMatrix3_(matrix3_),
        rotationAngleAxis3_(angleAxis3_),
        rotationCardanXYZ3_(cardanXYZ3_),
        rotationEulerZXZ3_(eulerZXZ3_),
        rotationQuaternion4_(quaternion4_),
        rotationMatrix4_(matrix4_),
        rotationAngleAxis4_(angleAxis4_),
        rotationCardanXYZ4_(cardanXYZ4_),
        rotationEulerZXZ4_(eulerZXZ4_),
        product12_(productQuaternion12_),
        product21_(productQuaternion21_),
        inverse1_(inverseQuaternion1_),
        inverse2_(inverseQuaternion2_),
        angularVelocity1_({-1.505583160450048, 0.855195318314842, 0.951045633351785}),
        angularVelocity2_({1.444434989549406, 2.019872740743849, 0.007132276917596}),
        angularVelocity3_({0.000000000000001, 0.000000000000000, 0.000000000000001}),
        angularVelocity4_({0.000000000000008, 0.000000000000003, 0.000000000000006}),
        rotationOutput_(Rotation()),
        angularVelocityOutput_(Eigen::Vector3d::Zero()) {
    }

    void SetUp() {
        rotationOutput_ = Rotation();
        angularVelocityOutput_ = Eigen::Vector3d::Zero();
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

    const Eigen::Quaterniond quaternion1_;
    const Eigen::Matrix3d matrix1_;
    const Eigen::AngleAxisd angleAxis1_;
    const CardanXYZ cardanXYZ1_;
    const EulerZXZ eulerZXZ1_;

    const Eigen::Quaterniond quaternion2_;
    const Eigen::Matrix3d matrix2_;
    const Eigen::AngleAxisd angleAxis2_;
    const CardanXYZ cardanXYZ2_;
    const EulerZXZ eulerZXZ2_;

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

    const Eigen::Quaterniond productQuaternion12_;
    const Eigen::Quaterniond productQuaternion21_;
    const Eigen::Quaterniond inverseQuaternion1_;
    const Eigen::Quaterniond inverseQuaternion2_;

 protected:
    const Rotation identityRotationQuaternion_;
    const Rotation identityRotationMatrix_;
    const Rotation identityRotationAngleAxis_;
    const Rotation identityRotationCardanXYZ_;
    const Rotation identityRotationEulerZXZ_;

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

    const Rotation rotationQuaternion3_;
    const Rotation rotationMatrix3_;
    const Rotation rotationAngleAxis3_;
    const Rotation rotationCardanXYZ3_;
    const Rotation rotationEulerZXZ3_;

    const Rotation rotationQuaternion4_;
    const Rotation rotationMatrix4_;
    const Rotation rotationAngleAxis4_;
    const Rotation rotationCardanXYZ4_;
    const Rotation rotationEulerZXZ4_;

    const Rotation product12_;
    const Rotation product21_;
    const Rotation inverse1_;
    const Rotation inverse2_;

    const Eigen::Vector3d angularVelocity1_;
    const Eigen::Vector3d angularVelocity2_;
    const Eigen::Vector3d angularVelocity3_;
    const Eigen::Vector3d angularVelocity4_;

    Rotation rotationOutput_;
    Eigen::Vector3d angularVelocityOutput_;
};

TEST_F(ArithmeticShould, GiveCorrectProduct) {
    ASSERT_TRUE(areAlmostEqual(multiply(rotationQuaternion1_, rotationQuaternion2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationQuaternion1_, rotationMatrix2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationQuaternion1_, rotationAngleAxis2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationQuaternion1_, rotationCardanXYZ2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationQuaternion1_, rotationEulerZXZ2_), product12_));

    ASSERT_TRUE(areAlmostEqual(multiply(rotationMatrix1_, rotationQuaternion2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationMatrix1_, rotationMatrix2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationMatrix1_, rotationAngleAxis2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationMatrix1_, rotationCardanXYZ2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationMatrix1_, rotationEulerZXZ2_), product12_));

    ASSERT_TRUE(areAlmostEqual(multiply(rotationAngleAxis1_, rotationQuaternion2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationAngleAxis1_, rotationMatrix2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationAngleAxis1_, rotationAngleAxis2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationAngleAxis1_, rotationCardanXYZ2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationAngleAxis1_, rotationEulerZXZ2_), product12_));

    ASSERT_TRUE(areAlmostEqual(multiply(rotationCardanXYZ1_, rotationQuaternion2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationCardanXYZ1_, rotationMatrix2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationCardanXYZ1_, rotationAngleAxis2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationCardanXYZ1_, rotationCardanXYZ2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationCardanXYZ1_, rotationEulerZXZ2_), product12_));

    ASSERT_TRUE(areAlmostEqual(multiply(rotationEulerZXZ1_, rotationQuaternion2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationEulerZXZ1_, rotationMatrix2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationEulerZXZ1_, rotationAngleAxis2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationEulerZXZ1_, rotationCardanXYZ2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationEulerZXZ1_, rotationEulerZXZ2_), product12_));

    ASSERT_TRUE(areAlmostEqual(multiply(rotationQuaternion2_, rotationQuaternion1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationQuaternion2_, rotationMatrix1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationQuaternion2_, rotationAngleAxis1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationQuaternion2_, rotationCardanXYZ1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationQuaternion2_, rotationEulerZXZ1_), product21_));

    ASSERT_TRUE(areAlmostEqual(multiply(rotationMatrix2_, rotationQuaternion1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationMatrix2_, rotationMatrix1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationMatrix2_, rotationAngleAxis1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationMatrix2_, rotationCardanXYZ1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationMatrix2_, rotationEulerZXZ1_), product21_));

    ASSERT_TRUE(areAlmostEqual(multiply(rotationAngleAxis2_, rotationQuaternion1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationAngleAxis2_, rotationMatrix1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationAngleAxis2_, rotationAngleAxis1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationAngleAxis2_, rotationCardanXYZ1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationAngleAxis2_, rotationEulerZXZ1_), product21_));

    ASSERT_TRUE(areAlmostEqual(multiply(rotationCardanXYZ2_, rotationQuaternion1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationCardanXYZ2_, rotationMatrix1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationCardanXYZ2_, rotationAngleAxis1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationCardanXYZ2_, rotationCardanXYZ1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationCardanXYZ2_, rotationEulerZXZ1_), product21_));

    ASSERT_TRUE(areAlmostEqual(multiply(rotationEulerZXZ2_, rotationQuaternion1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationEulerZXZ2_, rotationMatrix1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationEulerZXZ2_, rotationAngleAxis1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationEulerZXZ2_, rotationCardanXYZ1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(rotationEulerZXZ2_, rotationEulerZXZ1_), product21_));
}

TEST_F(ArithmeticShould, GiveCorrectInverse) {
    ASSERT_TRUE(areAlmostEqual(invert(rotationQuaternion1_), inverse1_));
    ASSERT_TRUE(areAlmostEqual(invert(rotationMatrix1_), inverse1_));
    ASSERT_TRUE(areAlmostEqual(invert(rotationAngleAxis1_), inverse1_));
    ASSERT_TRUE(areAlmostEqual(invert(rotationCardanXYZ1_), inverse1_));
    ASSERT_TRUE(areAlmostEqual(invert(rotationEulerZXZ1_), inverse1_));

    ASSERT_TRUE(areAlmostEqual(invert(rotationQuaternion2_), inverse2_));
    ASSERT_TRUE(areAlmostEqual(invert(rotationMatrix2_), inverse2_));
    ASSERT_TRUE(areAlmostEqual(invert(rotationAngleAxis2_), inverse2_));
    ASSERT_TRUE(areAlmostEqual(invert(rotationCardanXYZ2_), inverse2_));
    ASSERT_TRUE(areAlmostEqual(invert(rotationEulerZXZ2_), inverse2_));
}

TEST_F(ArithmeticShould, GiveCorrectAngularVelocityFromRotation) {
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity1_, angularVelocityFromRotation(rotationQuaternion1_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity1_, angularVelocityFromRotation(rotationMatrix1_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity1_, angularVelocityFromRotation(rotationAngleAxis1_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity1_, angularVelocityFromRotation(rotationCardanXYZ1_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity1_, angularVelocityFromRotation(rotationEulerZXZ1_)));

    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity2_, angularVelocityFromRotation(rotationQuaternion2_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity2_, angularVelocityFromRotation(rotationMatrix2_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity2_, angularVelocityFromRotation(rotationAngleAxis2_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity2_, angularVelocityFromRotation(rotationCardanXYZ2_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity2_, angularVelocityFromRotation(rotationEulerZXZ2_)));

    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity3_, angularVelocityFromRotation(rotationQuaternion3_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity3_, angularVelocityFromRotation(rotationMatrix3_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity3_, angularVelocityFromRotation(rotationAngleAxis3_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity3_, angularVelocityFromRotation(rotationCardanXYZ3_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity3_, angularVelocityFromRotation(rotationEulerZXZ3_)));

    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity4_, angularVelocityFromRotation(rotationQuaternion4_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity4_, angularVelocityFromRotation(rotationMatrix4_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity4_, angularVelocityFromRotation(rotationAngleAxis4_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity4_, angularVelocityFromRotation(rotationCardanXYZ4_)));
    ASSERT_TRUE(areAlmostEqualEigenVector3d(
        angularVelocity4_, angularVelocityFromRotation(rotationEulerZXZ4_)));
}

TEST_F(ArithmeticShould, GiveCorrectRotationFromAngularVelocity) {
    ASSERT_TRUE(
        areAlmostEqual(rotationQuaternion1_, rotationFromAngularVelocity(angularVelocity1_)));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix1_, rotationFromAngularVelocity(angularVelocity1_)));
    ASSERT_TRUE(
        areAlmostEqual(rotationAngleAxis1_, rotationFromAngularVelocity(angularVelocity1_)));
    ASSERT_TRUE(
        areAlmostEqual(rotationCardanXYZ1_, rotationFromAngularVelocity(angularVelocity1_)));
    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ1_, rotationFromAngularVelocity(angularVelocity1_)));

    ASSERT_TRUE(
        areAlmostEqual(rotationQuaternion2_, rotationFromAngularVelocity(angularVelocity2_)));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix2_, rotationFromAngularVelocity(angularVelocity2_)));
    ASSERT_TRUE(
        areAlmostEqual(rotationAngleAxis2_, rotationFromAngularVelocity(angularVelocity2_)));
    ASSERT_TRUE(
        areAlmostEqual(rotationCardanXYZ2_, rotationFromAngularVelocity(angularVelocity2_)));
    ASSERT_TRUE(areAlmostEqual(rotationEulerZXZ2_, rotationFromAngularVelocity(angularVelocity2_)));

    ASSERT_TRUE(areAlmostEqual(
        identityRotationQuaternion_, rotationFromAngularVelocity(angularVelocity3_)));
    ASSERT_TRUE(
        areAlmostEqual(identityRotationMatrix_, rotationFromAngularVelocity(angularVelocity3_)));
    ASSERT_TRUE(
        areAlmostEqual(identityRotationAngleAxis_, rotationFromAngularVelocity(angularVelocity3_)));
    ASSERT_TRUE(
        areAlmostEqual(identityRotationCardanXYZ_, rotationFromAngularVelocity(angularVelocity3_)));
    ASSERT_TRUE(
        areAlmostEqual(identityRotationEulerZXZ_, rotationFromAngularVelocity(angularVelocity3_)));

    ASSERT_TRUE(
        areAlmostEqual(rotationQuaternion4_, rotationFromAngularVelocity(angularVelocity4_)));
    ASSERT_TRUE(areAlmostEqual(rotationMatrix4_, rotationFromAngularVelocity(angularVelocity4_)));
    ASSERT_TRUE(
        areAlmostEqual(rotationAngleAxis4_, rotationFromAngularVelocity(angularVelocity4_)));
    ASSERT_TRUE(
        areAlmostEqual(rotationCardanXYZ4_, rotationFromAngularVelocity(angularVelocity4_)));
}
