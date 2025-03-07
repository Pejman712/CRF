/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Arithmetic.hpp"
#include "Types/Comparison.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::areAlmostEqual;

using crf::utility::types::TaskPose;
using crf::math::rotation::OrientationRepresentation;

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;

using crf::utility::types::multiply;
using crf::utility::types::invert;

class ArithmeticShould : public ::testing::Test {
 protected:
    ArithmeticShould() :
        position1_({2.8, -3.7, 15.12332332343423}),
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
        position2_({-2.2, -4.814231423423, 15.12332332343423}),
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
        productPosition12_({4.513824667026242, 11.963820932209373, 18.028362283203137}),
        productQuaternion12_(
            {0.248929242610910, -0.210978843434287, 0.763561846555290, -0.557221200495248}),
        productPosition21_({1.724921230141034, -9.213440723652397, 0.443852651264438}),
        productQuaternion21_(
            {0.248929242610910, 0.405745142292985, 0.317671965231668, 0.820054653169320}),
        inversePosition1_({12.523372027064923, 8.238080662960240, 5.054115986739689}),
        inverseQuaternion1_(
            {0.550566651690211, 0.636215237228148, -0.361380431590003, -0.401883960402980}),
        inversePosition2_({11.162304261688938, -1.688369248362640, 11.370316855482425}),
        inverseQuaternion2_(
            {0.323278141316011, -0.550446758565959, -0.769735163508487, -0.002717975359840}),
        taskPoseQuaternion1_(position1_, quaternion1_),
        taskPoseMatrix1_(position1_, matrix1_),
        taskPoseAngleAxis1_(position1_, angleAxis1_),
        taskPoseCardanXYZ1_(position1_, cardanXYZ1_),
        taskPoseEulerZXZ1_(position1_, eulerZXZ1_),
        taskPoseQuaternion2_(position2_, quaternion2_),
        taskPoseMatrix2_(position2_, matrix2_),
        taskPoseAngleAxis2_(position2_, angleAxis2_),
        taskPoseCardanXYZ2_(position2_, cardanXYZ2_),
        taskPoseEulerZXZ2_(position2_, eulerZXZ2_),
        product12_(productPosition12_, productQuaternion12_),
        product21_(productPosition21_, productQuaternion21_),
        inverse1_(inversePosition1_, inverseQuaternion1_),
        inverse2_(inversePosition2_, inverseQuaternion2_),
        taskPoseOutput_(TaskPose()) {
    }

    void SetUp() {
        taskPoseOutput_ = TaskPose();
    }

    const Eigen::Vector3d position1_;
    const Eigen::Quaterniond quaternion1_;
    const Eigen::Matrix3d matrix1_;
    const Eigen::AngleAxisd angleAxis1_;
    const CardanXYZ cardanXYZ1_;
    const EulerZXZ eulerZXZ1_;

    const Eigen::Vector3d position2_;
    const Eigen::Quaterniond quaternion2_;
    const Eigen::Matrix3d matrix2_;
    const Eigen::AngleAxisd angleAxis2_;
    const CardanXYZ cardanXYZ2_;
    const EulerZXZ eulerZXZ2_;

    const Eigen::Vector3d productPosition12_;
    const Eigen::Quaterniond productQuaternion12_;
    const Eigen::Vector3d productPosition21_;
    const Eigen::Quaterniond productQuaternion21_;
    const Eigen::Vector3d inversePosition1_;
    const Eigen::Quaterniond inverseQuaternion1_;
    const Eigen::Vector3d inversePosition2_;
    const Eigen::Quaterniond inverseQuaternion2_;

 protected:
    const TaskPose taskPoseQuaternion1_;
    const TaskPose taskPoseMatrix1_;
    const TaskPose taskPoseAngleAxis1_;
    const TaskPose taskPoseCardanXYZ1_;
    const TaskPose taskPoseEulerZXZ1_;

    const TaskPose taskPoseQuaternion2_;
    const TaskPose taskPoseMatrix2_;
    const TaskPose taskPoseAngleAxis2_;
    const TaskPose taskPoseCardanXYZ2_;
    const TaskPose taskPoseEulerZXZ2_;

    const TaskPose product12_;
    const TaskPose product21_;
    const TaskPose inverse1_;
    const TaskPose inverse2_;

    TaskPose taskPoseOutput_;
};

TEST_F(ArithmeticShould, GiveCorrectProduct) {
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseQuaternion1_, taskPoseQuaternion2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseQuaternion1_, taskPoseMatrix2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseQuaternion1_, taskPoseAngleAxis2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseQuaternion1_, taskPoseCardanXYZ2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseQuaternion1_, taskPoseEulerZXZ2_), product12_));

    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseMatrix1_, taskPoseQuaternion2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseMatrix1_, taskPoseMatrix2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseMatrix1_, taskPoseAngleAxis2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseMatrix1_, taskPoseCardanXYZ2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseMatrix1_, taskPoseEulerZXZ2_), product12_));

    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseAngleAxis1_, taskPoseQuaternion2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseAngleAxis1_, taskPoseMatrix2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseAngleAxis1_, taskPoseAngleAxis2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseAngleAxis1_, taskPoseCardanXYZ2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseAngleAxis1_, taskPoseEulerZXZ2_), product12_));

    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseCardanXYZ1_, taskPoseQuaternion2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseCardanXYZ1_, taskPoseMatrix2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseCardanXYZ1_, taskPoseAngleAxis2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseCardanXYZ1_, taskPoseCardanXYZ2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseCardanXYZ1_, taskPoseEulerZXZ2_), product12_));

    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseEulerZXZ1_, taskPoseQuaternion2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseEulerZXZ1_, taskPoseMatrix2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseEulerZXZ1_, taskPoseAngleAxis2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseEulerZXZ1_, taskPoseCardanXYZ2_), product12_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseEulerZXZ1_, taskPoseEulerZXZ2_), product12_));

    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseQuaternion2_, taskPoseQuaternion1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseQuaternion2_, taskPoseMatrix1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseQuaternion2_, taskPoseAngleAxis1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseQuaternion2_, taskPoseCardanXYZ1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseQuaternion2_, taskPoseEulerZXZ1_), product21_));

    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseMatrix2_, taskPoseQuaternion1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseMatrix2_, taskPoseMatrix1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseMatrix2_, taskPoseAngleAxis1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseMatrix2_, taskPoseCardanXYZ1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseMatrix2_, taskPoseEulerZXZ1_), product21_));

    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseAngleAxis2_, taskPoseQuaternion1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseAngleAxis2_, taskPoseMatrix1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseAngleAxis2_, taskPoseAngleAxis1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseAngleAxis2_, taskPoseCardanXYZ1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseAngleAxis2_, taskPoseEulerZXZ1_), product21_));

    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseCardanXYZ2_, taskPoseQuaternion1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseCardanXYZ2_, taskPoseMatrix1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseCardanXYZ2_, taskPoseAngleAxis1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseCardanXYZ2_, taskPoseCardanXYZ1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseCardanXYZ2_, taskPoseEulerZXZ1_), product21_));

    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseEulerZXZ2_, taskPoseQuaternion1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseEulerZXZ2_, taskPoseMatrix1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseEulerZXZ2_, taskPoseAngleAxis1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseEulerZXZ2_, taskPoseCardanXYZ1_), product21_));
    ASSERT_TRUE(areAlmostEqual(multiply(taskPoseEulerZXZ2_, taskPoseEulerZXZ1_), product21_));
}

TEST_F(ArithmeticShould, GiveCorrectInverse) {
    ASSERT_TRUE(areAlmostEqual(invert(taskPoseQuaternion1_), inverse1_));
    ASSERT_TRUE(areAlmostEqual(invert(taskPoseMatrix1_), inverse1_));
    ASSERT_TRUE(areAlmostEqual(invert(taskPoseAngleAxis1_), inverse1_));
    ASSERT_TRUE(areAlmostEqual(invert(taskPoseCardanXYZ1_), inverse1_));
    ASSERT_TRUE(areAlmostEqual(invert(taskPoseEulerZXZ1_), inverse1_));

    ASSERT_TRUE(areAlmostEqual(invert(taskPoseQuaternion2_), inverse2_));
    ASSERT_TRUE(areAlmostEqual(invert(taskPoseMatrix2_), inverse2_));
    ASSERT_TRUE(areAlmostEqual(invert(taskPoseAngleAxis2_), inverse2_));
    ASSERT_TRUE(areAlmostEqual(invert(taskPoseCardanXYZ2_), inverse2_));
    ASSERT_TRUE(areAlmostEqual(invert(taskPoseEulerZXZ2_), inverse2_));
}
