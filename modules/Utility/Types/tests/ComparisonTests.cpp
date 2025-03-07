/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Comparison.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::areAlmostEqual;
using crf::utility::types::isLesser;
using crf::utility::types::isGreater;
using crf::utility::types::isBetween;

using crf::utility::types::VectorXd;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

using crf::utility::types::TaskPose;
using crf::utility::types::Vector6d;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::TaskSpace;

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;

class ComparisonShould : public ::testing::Test {
 protected:
    ComparisonShould() :
        stdVector1_(
            {-2.55417,
             -42.5764243346,
             -std::numeric_limits<double>::infinity(),
             -8.16,
             -std::numeric_limits<double>::infinity(),
             2.0,
             -std::numeric_limits<double>::infinity()}),
        stdVector2_(
            {15.4,
             23.2,
             -std::numeric_limits<double>::infinity(),
             -3.16,
             84.44,
             80.455743523234,
             std::numeric_limits<double>::infinity()}),
        stdVector3_(
            {21.0,
             std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::infinity(),
             -0.1012,
             882.24,
             81.2,
             std::numeric_limits<double>::infinity()}),
        stdVector4_({21.0, 23.2000001, 516.0, -0.1012, 882.24, 81.2}),
        stdVector1Diff7_(
            {-2.55417,
             -42.5764244346,
             -std::numeric_limits<double>::infinity(),
             -8.16,
             -std::numeric_limits<double>::infinity(),
             2.0,
             -std::numeric_limits<double>::infinity()}),
        vectorXd1_(stdVector1_),
        vectorXd2_(stdVector2_),
        vectorXd3_(stdVector3_),
        vectorXd4_(stdVector4_),
        vectorXd1Diff7_(stdVector1Diff7_),
        jointPositions1_(stdVector1_),
        jointPositions2_(stdVector2_),
        jointPositions3_(stdVector3_),
        jointPositions4_(stdVector4_),
        jointPositions1Diff7_(stdVector1Diff7_),
        jointVelocities1_(stdVector1_),
        jointVelocities2_(stdVector2_),
        jointVelocities3_(stdVector3_),
        jointVelocities4_(stdVector4_),
        jointVelocities1Diff7_(stdVector1Diff7_),
        jointAccelerations1_(stdVector1_),
        jointAccelerations2_(stdVector2_),
        jointAccelerations3_(stdVector3_),
        jointAccelerations4_(stdVector4_),
        jointAccelerations1Diff7_(stdVector1Diff7_),
        jointForceTorques1_(stdVector1_),
        jointForceTorques2_(stdVector2_),
        jointForceTorques3_(stdVector3_),
        jointForceTorques4_(stdVector4_),
        jointForceTorques1Diff7_(stdVector1Diff7_),
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
        position1Diff7_({2.7999999, -3.7, 15.12332332343423}),
        quaternion1Diff7_(
            {0.550566622140754, -0.636215266863117, 0.361380448423139, 0.401883938833470}),
        matrix1Diff7_(
            {{0.415786942410374, -0.902359282257610, -0.113441369193796},
             {-0.017303751472951, -0.132561132164022, 0.991023776921803},
             {-0.909297420377950, -0.410091784803016, -0.070731388584432}}),
        angleAxis1Diff7_(
            1.975506963269755,
            Eigen::Vector3d({-0.762125002533116, 0.432899192325041, 0.481418497565502})),
        cardanXYZ1Diff7_({1.399999762602052, 2.000000015493885, 3.099999784134658}),
        eulerZXZ1Diff7_({1.1471123464639490, -1.6415868259093058, 0.1139727950424842}),
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
        taskPoseQuaternion1Diff7Position_(position1Diff7_, quaternion1_),
        taskPoseMatrix1Diff7Position_(position1Diff7_, matrix1_),
        taskPoseAngleAxis1Diff7Position_(position1Diff7_, angleAxis1_),
        taskPoseCardanXYZ1Diff7Position_(position1Diff7_, cardanXYZ1_),
        taskPoseEulerZXZ1Diff7Position_(position1Diff7_, eulerZXZ1_),
        taskPoseQuaternion1Diff7Orientation_(position1_, quaternion1Diff7_),
        taskPoseMatrix1Diff7Orientation_(position1_, matrix1Diff7_),
        taskPoseAngleAxis1Diff7Orientation_(position1_, angleAxis1Diff7_),
        taskPoseCardanXYZ1Diff7Orientation_(position1_, cardanXYZ1Diff7_),
        taskPoseEulerZXZ1Diff7Orientation_(position1_, eulerZXZ1Diff7_),
        stdArray1_(
            {-2.55417,
             -42.5764243346,
             -std::numeric_limits<double>::infinity(),
             -8.16,
             -std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity()}),
        stdArray2_(
            {15.4,
             23.2,
             -std::numeric_limits<double>::infinity(),
             -3.16,
             84.44,
             std::numeric_limits<double>::infinity()}),
        stdArray3_(
            {21.0,
             23.2000001,
             std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::infinity(),
             882.24,
             std::numeric_limits<double>::infinity()}),
        stdArray1Diff7_(
            {-2.55417,
             -42.5764244346,
             -std::numeric_limits<double>::infinity(),
             -8.16,
             -std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity()}),
        vector6d1_(stdArray1_),
        vector6d2_(stdArray2_),
        vector6d3_(stdArray3_),
        vector6d1Diff7_(stdArray1Diff7_),
        taskVelocity1_(stdArray1_),
        taskVelocity2_(stdArray2_),
        taskVelocity3_(stdArray3_),
        taskVelocity1Diff7_(stdArray1Diff7_),
        taskAcceleration1_(stdArray1_),
        taskAcceleration2_(stdArray2_),
        taskAcceleration3_(stdArray3_),
        taskAcceleration1Diff7_(stdArray1Diff7_),
        taskForceTorque1_(stdArray1_),
        taskForceTorque2_(stdArray2_),
        taskForceTorque3_(stdArray3_),
        taskForceTorque1Diff7_(stdArray1Diff7_),
        taskSpace1_({true, true, true, true, true, true}),
        taskSpace2_({true, false, true, true, false, false}) {
    }

    const std::vector<double> stdVector1_;
    const std::vector<double> stdVector2_;
    const std::vector<double> stdVector3_;
    const std::vector<double> stdVector4_;
    const std::vector<double> stdVector1Diff7_;

    const VectorXd vectorXd1_;
    const VectorXd vectorXd2_;
    const VectorXd vectorXd3_;
    const VectorXd vectorXd4_;
    const VectorXd vectorXd1Diff7_;

    const JointPositions jointPositions1_;
    const JointPositions jointPositions2_;
    const JointPositions jointPositions3_;
    const JointPositions jointPositions4_;
    const JointPositions jointPositions1Diff7_;

    const JointVelocities jointVelocities1_;
    const JointVelocities jointVelocities2_;
    const JointVelocities jointVelocities3_;
    const JointVelocities jointVelocities4_;
    const JointVelocities jointVelocities1Diff7_;

    const JointAccelerations jointAccelerations1_;
    const JointAccelerations jointAccelerations2_;
    const JointAccelerations jointAccelerations3_;
    const JointAccelerations jointAccelerations4_;
    const JointAccelerations jointAccelerations1Diff7_;

    const JointForceTorques jointForceTorques1_;
    const JointForceTorques jointForceTorques2_;
    const JointForceTorques jointForceTorques3_;
    const JointForceTorques jointForceTorques4_;
    const JointForceTorques jointForceTorques1Diff7_;

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

    const Eigen::Vector3d position1Diff7_;
    const Eigen::Quaterniond quaternion1Diff7_;
    const Eigen::Matrix3d matrix1Diff7_;
    const Eigen::AngleAxisd angleAxis1Diff7_;
    const CardanXYZ cardanXYZ1Diff7_;
    const EulerZXZ eulerZXZ1Diff7_;

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

    const TaskPose taskPoseQuaternion1Diff7Position_;
    const TaskPose taskPoseMatrix1Diff7Position_;
    const TaskPose taskPoseAngleAxis1Diff7Position_;
    const TaskPose taskPoseCardanXYZ1Diff7Position_;
    const TaskPose taskPoseEulerZXZ1Diff7Position_;

    const TaskPose taskPoseQuaternion1Diff7Orientation_;
    const TaskPose taskPoseMatrix1Diff7Orientation_;
    const TaskPose taskPoseAngleAxis1Diff7Orientation_;
    const TaskPose taskPoseCardanXYZ1Diff7Orientation_;
    const TaskPose taskPoseEulerZXZ1Diff7Orientation_;

    const std::array<double, 6> stdArray1_;
    const std::array<double, 6> stdArray2_;
    const std::array<double, 6> stdArray3_;
    const std::array<double, 6> stdArray1Diff7_;

    const Vector6d vector6d1_;
    const Vector6d vector6d2_;
    const Vector6d vector6d3_;
    const Vector6d vector6d1Diff7_;

    const TaskVelocity taskVelocity1_;
    const TaskVelocity taskVelocity2_;
    const TaskVelocity taskVelocity3_;
    const TaskVelocity taskVelocity1Diff7_;

    const TaskAcceleration taskAcceleration1_;
    const TaskAcceleration taskAcceleration2_;
    const TaskAcceleration taskAcceleration3_;
    const TaskAcceleration taskAcceleration1Diff7_;

    const TaskForceTorque taskForceTorque1_;
    const TaskForceTorque taskForceTorque2_;
    const TaskForceTorque taskForceTorque3_;
    const TaskForceTorque taskForceTorque1Diff7_;

    const TaskSpace taskSpace1_;
    const TaskSpace taskSpace2_;
};

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameVectorXdArguments) {
    ASSERT_TRUE(areAlmostEqual(vectorXd1_, vectorXd1_));
    ASSERT_TRUE(areAlmostEqual(vectorXd1_, vectorXd1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(vectorXd1Diff7_, vectorXd1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentVectorXdArguments) {
    ASSERT_FALSE(areAlmostEqual(vectorXd1_, vectorXd2_));
    ASSERT_FALSE(areAlmostEqual(vectorXd2_, vectorXd1_));
    ASSERT_FALSE(areAlmostEqual(vectorXd1_, vectorXd1Diff7_));
    ASSERT_FALSE(areAlmostEqual(vectorXd1Diff7_, vectorXd1_));
    ASSERT_FALSE(areAlmostEqual(vectorXd1_, vectorXd1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(vectorXd1Diff7_, vectorXd1_, 1e-8));
}

TEST_F(ComparisonShould, ThrowAreAlmostEqualOnVectorXdArgumentsOfDifferentSizes) {
    ASSERT_THROW(areAlmostEqual(vectorXd1_, vectorXd4_), std::invalid_argument);
    ASSERT_THROW(areAlmostEqual(vectorXd4_, vectorXd1_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameJointPositionsArguments) {
    ASSERT_TRUE(areAlmostEqual(jointPositions1_, jointPositions1_));
    ASSERT_TRUE(areAlmostEqual(jointPositions1_, jointPositions1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(jointPositions1Diff7_, jointPositions1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentJointPositionsArguments) {
    ASSERT_FALSE(areAlmostEqual(jointPositions1_, jointPositions2_));
    ASSERT_FALSE(areAlmostEqual(jointPositions2_, jointPositions1_));
    ASSERT_FALSE(areAlmostEqual(jointPositions1_, jointPositions1Diff7_));
    ASSERT_FALSE(areAlmostEqual(jointPositions1Diff7_, jointPositions1_));
    ASSERT_FALSE(areAlmostEqual(jointPositions1_, jointPositions1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(jointPositions1Diff7_, jointPositions1_, 1e-8));
}

TEST_F(ComparisonShould, ThrowAreAlmostEqualOnJointPositionsArgumentsOfDifferentSizes) {
    ASSERT_THROW(areAlmostEqual(jointPositions1_, jointPositions4_), std::invalid_argument);
    ASSERT_THROW(areAlmostEqual(jointPositions4_, jointPositions1_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameJointVelocitiesArguments) {
    ASSERT_TRUE(areAlmostEqual(jointVelocities1_, jointVelocities1_));
    ASSERT_TRUE(areAlmostEqual(jointVelocities1_, jointVelocities1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(jointVelocities1Diff7_, jointVelocities1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentJointVelocitiesArguments) {
    ASSERT_FALSE(areAlmostEqual(jointVelocities1_, jointVelocities2_));
    ASSERT_FALSE(areAlmostEqual(jointVelocities2_, jointVelocities1_));
    ASSERT_FALSE(areAlmostEqual(jointVelocities1_, jointVelocities1Diff7_));
    ASSERT_FALSE(areAlmostEqual(jointVelocities1Diff7_, jointVelocities1_));
    ASSERT_FALSE(areAlmostEqual(jointVelocities1_, jointVelocities1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(jointVelocities1Diff7_, jointVelocities1_, 1e-8));
}

TEST_F(ComparisonShould, ThrowAreAlmostEqualOnJointVelocitiesArgumentsOfDifferentSizes) {
    ASSERT_THROW(areAlmostEqual(jointVelocities1_, jointVelocities4_), std::invalid_argument);
    ASSERT_THROW(areAlmostEqual(jointVelocities4_, jointVelocities1_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameJointAccelerationsArguments) {
    ASSERT_TRUE(areAlmostEqual(jointAccelerations1_, jointAccelerations1_));
    ASSERT_TRUE(areAlmostEqual(jointAccelerations1_, jointAccelerations1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(jointAccelerations1Diff7_, jointAccelerations1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentJointAccelerationsArguments) {
    ASSERT_FALSE(areAlmostEqual(jointAccelerations1_, jointAccelerations2_));
    ASSERT_FALSE(areAlmostEqual(jointAccelerations2_, jointAccelerations1_));
    ASSERT_FALSE(areAlmostEqual(jointAccelerations1_, jointAccelerations1Diff7_));
    ASSERT_FALSE(areAlmostEqual(jointAccelerations1Diff7_, jointAccelerations1_));
    ASSERT_FALSE(areAlmostEqual(jointAccelerations1_, jointAccelerations1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(jointAccelerations1Diff7_, jointAccelerations1_, 1e-8));
}

TEST_F(ComparisonShould, ThrowAreAlmostEqualOnJointAccelerationsArgumentsOfDifferentSizes) {
    ASSERT_THROW(areAlmostEqual(jointAccelerations1_, jointAccelerations4_), std::invalid_argument);
    ASSERT_THROW(areAlmostEqual(jointAccelerations4_, jointAccelerations1_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameJointForceTorquesArguments) {
    ASSERT_TRUE(areAlmostEqual(jointForceTorques1_, jointForceTorques1_));
    ASSERT_TRUE(areAlmostEqual(jointForceTorques1_, jointForceTorques1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(jointForceTorques1Diff7_, jointForceTorques1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentJointForceTorquesArguments) {
    ASSERT_FALSE(areAlmostEqual(jointForceTorques1_, jointForceTorques2_));
    ASSERT_FALSE(areAlmostEqual(jointForceTorques2_, jointForceTorques1_));
    ASSERT_FALSE(areAlmostEqual(jointForceTorques1_, jointForceTorques1Diff7_));
    ASSERT_FALSE(areAlmostEqual(jointForceTorques1Diff7_, jointForceTorques1_));
    ASSERT_FALSE(areAlmostEqual(jointForceTorques1_, jointForceTorques1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(jointForceTorques1Diff7_, jointForceTorques1_, 1e-8));
}

TEST_F(ComparisonShould, ThrowAreAlmostEqualOnJointForceTorquesArgumentsOfDifferentSizes) {
    ASSERT_THROW(areAlmostEqual(jointForceTorques1_, jointForceTorques4_), std::invalid_argument);
    ASSERT_THROW(areAlmostEqual(jointForceTorques4_, jointForceTorques1_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameTaskPoseArguments) {
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseMatrix1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseAngleAxis1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseCardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseEulerZXZ1_));

    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseMatrix1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseAngleAxis1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseCardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseEulerZXZ1_));

    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseMatrix1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseAngleAxis1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseCardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseEulerZXZ1_));

    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseMatrix1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseAngleAxis1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseCardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseEulerZXZ1_));

    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseMatrix1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseAngleAxis1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseCardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseEulerZXZ1_));

    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseQuaternion1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseMatrix1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseAngleAxis1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseCardanXYZ1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseEulerZXZ1Diff7Orientation_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseQuaternion1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseMatrix1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseAngleAxis1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseCardanXYZ1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseEulerZXZ1Diff7Orientation_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseQuaternion1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseMatrix1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseAngleAxis1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseCardanXYZ1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseEulerZXZ1Diff7Orientation_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseQuaternion1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseMatrix1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseAngleAxis1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseCardanXYZ1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseEulerZXZ1Diff7Orientation_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseQuaternion1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseMatrix1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseAngleAxis1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseCardanXYZ1Diff7Orientation_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseEulerZXZ1Diff7Orientation_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseEulerZXZ1_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseEulerZXZ1_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseEulerZXZ1_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseEulerZXZ1_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseEulerZXZ1_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseQuaternion1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseMatrix1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseAngleAxis1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseCardanXYZ1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1_, taskPoseEulerZXZ1Diff7Position_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseQuaternion1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseMatrix1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseAngleAxis1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseCardanXYZ1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1_, taskPoseEulerZXZ1Diff7Position_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseQuaternion1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseMatrix1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseAngleAxis1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseCardanXYZ1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseEulerZXZ1Diff7Position_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseQuaternion1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseMatrix1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseAngleAxis1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseCardanXYZ1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseEulerZXZ1Diff7Position_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseQuaternion1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseMatrix1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseAngleAxis1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseCardanXYZ1Diff7Position_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseEulerZXZ1Diff7Position_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseEulerZXZ1_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseEulerZXZ1_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseEulerZXZ1_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseEulerZXZ1_, 1e-6));

    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseQuaternion1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseMatrix1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseAngleAxis1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseCardanXYZ1_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseEulerZXZ1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentTaskPoseArguments) {
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseQuaternion2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseMatrix2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseAngleAxis2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseCardanXYZ2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseEulerZXZ2_));

    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseQuaternion2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseMatrix2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseAngleAxis2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseCardanXYZ2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseEulerZXZ2_));

    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseQuaternion2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseMatrix2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseAngleAxis2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseCardanXYZ2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseEulerZXZ2_));

    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseQuaternion2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseMatrix2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseAngleAxis2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseCardanXYZ2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseEulerZXZ2_));

    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseQuaternion2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseMatrix2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseAngleAxis2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseCardanXYZ2_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseEulerZXZ2_));

    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion2_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion2_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion2_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion2_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion2_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix2_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix2_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix2_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix2_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix2_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis2_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis2_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis2_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis2_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis2_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ2_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ2_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ2_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ2_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ2_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ2_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ2_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ2_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ2_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ2_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseQuaternion1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseMatrix1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseAngleAxis1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseCardanXYZ1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseEulerZXZ1Diff7Orientation_));

    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseQuaternion1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseMatrix1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseAngleAxis1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseCardanXYZ1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseEulerZXZ1Diff7Orientation_));

    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseQuaternion1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseMatrix1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseAngleAxis1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseCardanXYZ1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseEulerZXZ1Diff7Orientation_));

    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseQuaternion1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseMatrix1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseAngleAxis1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseCardanXYZ1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseEulerZXZ1Diff7Orientation_));

    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseQuaternion1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseMatrix1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseAngleAxis1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseCardanXYZ1Diff7Orientation_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseEulerZXZ1Diff7Orientation_));

    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseQuaternion1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseMatrix1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseAngleAxis1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseCardanXYZ1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseEulerZXZ1Diff7Orientation_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseQuaternion1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseMatrix1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseAngleAxis1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseCardanXYZ1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseEulerZXZ1Diff7Orientation_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseQuaternion1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseMatrix1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseAngleAxis1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseCardanXYZ1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseEulerZXZ1Diff7Orientation_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseQuaternion1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseMatrix1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseAngleAxis1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseCardanXYZ1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseEulerZXZ1Diff7Orientation_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseQuaternion1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseMatrix1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseAngleAxis1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseCardanXYZ1Diff7Orientation_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseEulerZXZ1Diff7Orientation_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Orientation_, taskPoseEulerZXZ1_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Orientation_, taskPoseEulerZXZ1_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Orientation_, taskPoseEulerZXZ1_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Orientation_, taskPoseEulerZXZ1_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Orientation_, taskPoseEulerZXZ1_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseQuaternion1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseMatrix1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseAngleAxis1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseCardanXYZ1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseEulerZXZ1Diff7Position_));

    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseQuaternion1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseMatrix1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseAngleAxis1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseCardanXYZ1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseEulerZXZ1Diff7Position_));

    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseQuaternion1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseMatrix1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseAngleAxis1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseCardanXYZ1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseEulerZXZ1Diff7Position_));

    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseQuaternion1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseMatrix1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseAngleAxis1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseCardanXYZ1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseEulerZXZ1Diff7Position_));

    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseQuaternion1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseMatrix1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseAngleAxis1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseCardanXYZ1Diff7Position_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseEulerZXZ1Diff7Position_));

    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseQuaternion1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseMatrix1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseAngleAxis1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseCardanXYZ1_));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseEulerZXZ1_));

    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseQuaternion1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseMatrix1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseAngleAxis1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseCardanXYZ1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1_, taskPoseEulerZXZ1Diff7Position_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseQuaternion1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseMatrix1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseAngleAxis1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseCardanXYZ1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1_, taskPoseEulerZXZ1Diff7Position_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseQuaternion1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseMatrix1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseAngleAxis1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseCardanXYZ1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1_, taskPoseEulerZXZ1Diff7Position_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseQuaternion1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseMatrix1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseAngleAxis1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseCardanXYZ1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1_, taskPoseEulerZXZ1Diff7Position_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseQuaternion1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseMatrix1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseAngleAxis1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseCardanXYZ1Diff7Position_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1_, taskPoseEulerZXZ1Diff7Position_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseQuaternion1Diff7Position_, taskPoseEulerZXZ1_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseMatrix1Diff7Position_, taskPoseEulerZXZ1_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseAngleAxis1Diff7Position_, taskPoseEulerZXZ1_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseCardanXYZ1Diff7Position_, taskPoseEulerZXZ1_, 1e-8));

    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseQuaternion1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseMatrix1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseAngleAxis1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseCardanXYZ1_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskPoseEulerZXZ1Diff7Position_, taskPoseEulerZXZ1_, 1e-8));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameVector6dArguments) {
    ASSERT_TRUE(areAlmostEqual(vector6d1_, vector6d1_));
    ASSERT_TRUE(areAlmostEqual(vector6d1_, vector6d1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(vector6d1Diff7_, vector6d1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentVector6dArguments) {
    ASSERT_FALSE(areAlmostEqual(vector6d1_, vector6d2_));
    ASSERT_FALSE(areAlmostEqual(vector6d2_, vector6d1_));
    ASSERT_FALSE(areAlmostEqual(vector6d1_, vector6d1Diff7_));
    ASSERT_FALSE(areAlmostEqual(vector6d1Diff7_, vector6d1_));
    ASSERT_FALSE(areAlmostEqual(vector6d1_, vector6d1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(vector6d1Diff7_, vector6d1_, 1e-8));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameTaskVelocityArguments) {
    ASSERT_TRUE(areAlmostEqual(taskVelocity1_, taskVelocity1_));
    ASSERT_TRUE(areAlmostEqual(taskVelocity1_, taskVelocity1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskVelocity1Diff7_, taskVelocity1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentTaskVelocityArguments) {
    ASSERT_FALSE(areAlmostEqual(taskVelocity1_, taskVelocity2_));
    ASSERT_FALSE(areAlmostEqual(taskVelocity2_, taskVelocity1_));
    ASSERT_FALSE(areAlmostEqual(taskVelocity1_, taskVelocity1Diff7_));
    ASSERT_FALSE(areAlmostEqual(taskVelocity1Diff7_, taskVelocity1_));
    ASSERT_FALSE(areAlmostEqual(taskVelocity1_, taskVelocity1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskVelocity1Diff7_, taskVelocity1_, 1e-8));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameTaskAccelerationArguments) {
    ASSERT_TRUE(areAlmostEqual(taskAcceleration1_, taskAcceleration1_));
    ASSERT_TRUE(areAlmostEqual(taskAcceleration1_, taskAcceleration1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskAcceleration1Diff7_, taskAcceleration1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentTaskAccelerationArguments) {
    ASSERT_FALSE(areAlmostEqual(taskAcceleration1_, taskAcceleration2_));
    ASSERT_FALSE(areAlmostEqual(taskAcceleration2_, taskAcceleration1_));
    ASSERT_FALSE(areAlmostEqual(taskAcceleration1_, taskAcceleration1Diff7_));
    ASSERT_FALSE(areAlmostEqual(taskAcceleration1Diff7_, taskAcceleration1_));
    ASSERT_FALSE(areAlmostEqual(taskAcceleration1_, taskAcceleration1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskAcceleration1Diff7_, taskAcceleration1_, 1e-8));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualTrueForTheSameTaskForceTorqueArguments) {
    ASSERT_TRUE(areAlmostEqual(taskForceTorque1_, taskForceTorque1_));
    ASSERT_TRUE(areAlmostEqual(taskForceTorque1_, taskForceTorque1Diff7_, 1e-6));
    ASSERT_TRUE(areAlmostEqual(taskForceTorque1Diff7_, taskForceTorque1_, 1e-6));
}

TEST_F(ComparisonShould, ReturnAreAlmostEqualFalseForDifferentTaskForceTorqueArguments) {
    ASSERT_FALSE(areAlmostEqual(taskForceTorque1_, taskForceTorque2_));
    ASSERT_FALSE(areAlmostEqual(taskForceTorque2_, taskForceTorque1_));
    ASSERT_FALSE(areAlmostEqual(taskForceTorque1_, taskForceTorque1Diff7_));
    ASSERT_FALSE(areAlmostEqual(taskForceTorque1Diff7_, taskForceTorque1_));
    ASSERT_FALSE(areAlmostEqual(taskForceTorque1_, taskForceTorque1Diff7_, 1e-8));
    ASSERT_FALSE(areAlmostEqual(taskForceTorque1Diff7_, taskForceTorque1_, 1e-8));
}

TEST_F(ComparisonShould, ReturnAreEqualTrueForTheSameTaskSpaceArguments) {
    ASSERT_TRUE(areEqual(taskSpace1_, taskSpace1_));
}

TEST_F(ComparisonShould, ReturnAreEqualFalseForDifferentTaskSpaceArguments) {
    ASSERT_FALSE(areEqual(taskSpace1_, taskSpace2_));
    ASSERT_FALSE(areEqual(taskSpace2_, taskSpace1_));
}

TEST_F(ComparisonShould, ReturnIsBetweenTrueForOrderedVectorXdArguments) {
    ASSERT_TRUE(isBetween(vectorXd1_, vectorXd3_, vectorXd2_));
}

TEST_F(ComparisonShould, ReturnIsBetweenFalseForNonOrderedVectorXdArguments) {
    ASSERT_FALSE(isBetween(vectorXd2_, vectorXd3_, vectorXd1_));
    ASSERT_FALSE(isBetween(vectorXd1_, vectorXd2_, vectorXd3_));
    ASSERT_FALSE(isBetween(vectorXd3_, vectorXd1_, vectorXd2_));
}

TEST_F(ComparisonShould, ThrowIsBetweenOnVectorXdArgumentsOfDifferentSizes) {
    ASSERT_THROW(isBetween(vectorXd4_, vectorXd3_, vectorXd2_), std::invalid_argument);
    ASSERT_THROW(isBetween(vectorXd1_, vectorXd3_, vectorXd4_), std::invalid_argument);
    ASSERT_THROW(isBetween(vectorXd1_, vectorXd4_, vectorXd2_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsBetweenTrueForOrderedJointPositionsArguments) {
    ASSERT_TRUE(isBetween(jointPositions1_, jointPositions3_, jointPositions2_));
}

TEST_F(ComparisonShould, ReturnIsBetweenFalseForNonOrderedJointPositionsArguments) {
    ASSERT_FALSE(isBetween(jointPositions2_, jointPositions3_, jointPositions1_));
    ASSERT_FALSE(isBetween(jointPositions1_, jointPositions2_, jointPositions3_));
    ASSERT_FALSE(isBetween(jointPositions3_, jointPositions1_, jointPositions2_));
}

TEST_F(ComparisonShould, ThrowIsBetweenOnJointPositionsArgumentsOfDifferentSizes) {
    ASSERT_THROW(
        isBetween(jointPositions4_, jointPositions3_, jointPositions2_), std::invalid_argument);
    ASSERT_THROW(
        isBetween(jointPositions1_, jointPositions3_, jointPositions4_), std::invalid_argument);
    ASSERT_THROW(
        isBetween(jointPositions1_, jointPositions4_, jointPositions2_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsBetweenTrueForOrderedJointVelocitiesArguments) {
    ASSERT_TRUE(isBetween(jointVelocities1_, jointVelocities3_, jointVelocities2_));
}

TEST_F(ComparisonShould, ReturnIsBetweenFalseForNonOrderedJointVelocitiesArguments) {
    ASSERT_FALSE(isBetween(jointVelocities2_, jointVelocities3_, jointVelocities1_));
    ASSERT_FALSE(isBetween(jointVelocities1_, jointVelocities2_, jointVelocities3_));
    ASSERT_FALSE(isBetween(jointVelocities3_, jointVelocities1_, jointVelocities2_));
}

TEST_F(ComparisonShould, ThrowIsBetweenOnJointVelocitiesArgumentsOfDifferentSizes) {
    ASSERT_THROW(
        isBetween(jointVelocities4_, jointVelocities3_, jointVelocities2_), std::invalid_argument);
    ASSERT_THROW(
        isBetween(jointVelocities1_, jointVelocities3_, jointVelocities4_), std::invalid_argument);
    ASSERT_THROW(
        isBetween(jointVelocities1_, jointVelocities4_, jointVelocities2_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsBetweenTrueForOrderedJointAccelerationsArguments) {
    ASSERT_TRUE(isBetween(jointAccelerations1_, jointAccelerations3_, jointAccelerations2_));
}

TEST_F(ComparisonShould, ReturnIsBetweenFalseForNonOrderedJointAccelerationsArguments) {
    ASSERT_FALSE(isBetween(jointAccelerations2_, jointAccelerations3_, jointAccelerations1_));
    ASSERT_FALSE(isBetween(jointAccelerations1_, jointAccelerations2_, jointAccelerations3_));
    ASSERT_FALSE(isBetween(jointAccelerations3_, jointAccelerations1_, jointAccelerations2_));
}

TEST_F(ComparisonShould, ThrowIsBetweenOnJointAccelerationsArgumentsOfDifferentSizes) {
    ASSERT_THROW(
        isBetween(jointAccelerations4_, jointAccelerations3_, jointAccelerations2_),
        std::invalid_argument);
    ASSERT_THROW(
        isBetween(jointAccelerations1_, jointAccelerations3_, jointAccelerations4_),
        std::invalid_argument);
    ASSERT_THROW(
        isBetween(jointAccelerations1_, jointAccelerations4_, jointAccelerations2_),
        std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsBetweenTrueForOrderedJointForceTorquesArguments) {
    ASSERT_TRUE(isBetween(jointForceTorques1_, jointForceTorques3_, jointForceTorques2_));
}

TEST_F(ComparisonShould, ReturnIsBetweenFalseForNonOrderedJointForceTorquesArguments) {
    ASSERT_FALSE(isBetween(jointForceTorques2_, jointForceTorques3_, jointForceTorques1_));
    ASSERT_FALSE(isBetween(jointForceTorques1_, jointForceTorques2_, jointForceTorques3_));
    ASSERT_FALSE(isBetween(jointForceTorques3_, jointForceTorques1_, jointForceTorques2_));
}

TEST_F(ComparisonShould, ThrowIsBetweenOnJointForceTorquesArgumentsOfDifferentSizes) {
    ASSERT_THROW(
        isBetween(jointForceTorques4_, jointForceTorques3_, jointForceTorques2_),
        std::invalid_argument);
    ASSERT_THROW(
        isBetween(jointForceTorques1_, jointForceTorques3_, jointForceTorques4_),
        std::invalid_argument);
    ASSERT_THROW(
        isBetween(jointForceTorques1_, jointForceTorques4_, jointForceTorques2_),
        std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsBetweenTrueForOrderedVector6dArguments) {
    ASSERT_TRUE(isBetween(vector6d1_, vector6d3_, vector6d2_));
}

TEST_F(ComparisonShould, ReturnIsBetweenFalseForNonOrderedVector6dArguments) {
    ASSERT_FALSE(isBetween(vector6d2_, vector6d3_, vector6d1_));
    ASSERT_FALSE(isBetween(vector6d1_, vector6d2_, vector6d3_));
    ASSERT_FALSE(isBetween(vector6d3_, vector6d1_, vector6d2_));
}

TEST_F(ComparisonShould, ReturnIsBetweenTrueForOrderedTaskVelocityArguments) {
    ASSERT_TRUE(isBetween(taskVelocity1_, taskVelocity3_, taskVelocity2_));
}

TEST_F(ComparisonShould, ReturnIsBetweenFalseForNonOrderedTaskVelocityArguments) {
    ASSERT_FALSE(isBetween(taskVelocity2_, taskVelocity3_, taskVelocity1_));
    ASSERT_FALSE(isBetween(taskVelocity1_, taskVelocity2_, taskVelocity3_));
    ASSERT_FALSE(isBetween(taskVelocity3_, taskVelocity1_, taskVelocity2_));
}

TEST_F(ComparisonShould, ReturnIsBetweenTrueForOrderedTaskAccelerationArguments) {
    ASSERT_TRUE(isBetween(taskAcceleration1_, taskAcceleration3_, taskAcceleration2_));
}

TEST_F(ComparisonShould, ReturnIsBetweenFalseForNonOrderedTaskAccelerationArguments) {
    ASSERT_FALSE(isBetween(taskAcceleration2_, taskAcceleration3_, taskAcceleration1_));
    ASSERT_FALSE(isBetween(taskAcceleration1_, taskAcceleration2_, taskAcceleration3_));
    ASSERT_FALSE(isBetween(taskAcceleration3_, taskAcceleration1_, taskAcceleration2_));
}

TEST_F(ComparisonShould, ReturnIsBetweenTrueForOrderedTaskForceTorqueArguments) {
    ASSERT_TRUE(isBetween(taskForceTorque1_, taskForceTorque3_, taskForceTorque2_));
}

TEST_F(ComparisonShould, ReturnIsBetweenFalseForNonOrderedTaskForceTorqueArguments) {
    ASSERT_FALSE(isBetween(taskForceTorque2_, taskForceTorque3_, taskForceTorque1_));
    ASSERT_FALSE(isBetween(taskForceTorque1_, taskForceTorque2_, taskForceTorque3_));
    ASSERT_FALSE(isBetween(taskForceTorque3_, taskForceTorque1_, taskForceTorque2_));
}

TEST_F(ComparisonShould, ReturnIsLesserTrueForOrderedVectorXdArguments) {
    ASSERT_TRUE(isLesser(vectorXd1_, vectorXd2_));
}

TEST_F(ComparisonShould, ReturnIsLesserFalseForNonOrderedVectorXdArguments) {
    ASSERT_FALSE(isLesser(vectorXd2_, vectorXd1_));
}

TEST_F(ComparisonShould, ThrowIsLesserOnVectorXdArgumentsOfDifferentSizes) {
    ASSERT_THROW(isLesser(vectorXd4_, vectorXd2_), std::invalid_argument);
    ASSERT_THROW(isLesser(vectorXd1_, vectorXd4_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsLesserTrueForOrderedJointPositionsArguments) {
    ASSERT_TRUE(isLesser(jointPositions1_, jointPositions2_));
}

TEST_F(ComparisonShould, ReturnIsLesserFalseForNonOrderedJointPositionsArguments) {
    ASSERT_FALSE(isLesser(jointPositions2_, jointPositions1_));
}

TEST_F(ComparisonShould, ThrowIsLesserOnJointPositionsArgumentsOfDifferentSizes) {
    ASSERT_THROW(isLesser(jointPositions4_, jointPositions2_), std::invalid_argument);
    ASSERT_THROW(isLesser(jointPositions1_, jointPositions4_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsLesserTrueForOrderedJointVelocitiesArguments) {
    ASSERT_TRUE(isLesser(jointVelocities1_, jointVelocities2_));
}

TEST_F(ComparisonShould, ReturnIsLesserFalseForNonOrderedJointVelocitiesArguments) {
    ASSERT_FALSE(isLesser(jointVelocities2_, jointVelocities1_));
}

TEST_F(ComparisonShould, ThrowIsLesserOnJointVelocitiesArgumentsOfDifferentSizes) {
    ASSERT_THROW(isLesser(jointVelocities4_, jointVelocities2_), std::invalid_argument);
    ASSERT_THROW(isLesser(jointVelocities1_, jointVelocities4_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsLesserTrueForOrderedJointAccelerationsArguments) {
    ASSERT_TRUE(isLesser(jointAccelerations1_, jointAccelerations2_));
}

TEST_F(ComparisonShould, ReturnIsLesserFalseForNonOrderedJointAccelerationsArguments) {
    ASSERT_FALSE(isLesser(jointAccelerations2_, jointAccelerations1_));
}

TEST_F(ComparisonShould, ThrowIsLesserOnJointAccelerationsArgumentsOfDifferentSizes) {
    ASSERT_THROW(isLesser(jointAccelerations4_, jointAccelerations2_), std::invalid_argument);
    ASSERT_THROW(isLesser(jointAccelerations1_, jointAccelerations4_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsLesserTrueForOrderedJointForceTorquesArguments) {
    ASSERT_TRUE(isLesser(jointForceTorques1_, jointForceTorques2_));
}

TEST_F(ComparisonShould, ReturnIsLesserFalseForNonOrderedJointForceTorquesArguments) {
    ASSERT_FALSE(isLesser(jointForceTorques2_, jointForceTorques1_));
}

TEST_F(ComparisonShould, ThrowIsLesserOnJointForceTorquesArgumentsOfDifferentSizes) {
    ASSERT_THROW(isLesser(jointForceTorques4_, jointForceTorques2_), std::invalid_argument);
    ASSERT_THROW(isLesser(jointForceTorques1_, jointForceTorques4_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsLesserTrueForOrderedVector6dArguments) {
    ASSERT_TRUE(isLesser(vector6d1_, vector6d2_));
}

TEST_F(ComparisonShould, ReturnIsLesserFalseForNonOrderedVector6dArguments) {
    ASSERT_FALSE(isLesser(vector6d2_, vector6d1_));
}

TEST_F(ComparisonShould, ReturnIsLesserTrueForOrderedTaskVelocityArguments) {
    ASSERT_TRUE(isLesser(taskVelocity1_, taskVelocity2_));
}

TEST_F(ComparisonShould, ReturnIsLesserFalseForNonOrderedTaskVelocityArguments) {
    ASSERT_FALSE(isLesser(taskVelocity2_, taskVelocity1_));
}

TEST_F(ComparisonShould, ReturnIsLesserTrueForOrderedTaskAccelerationArguments) {
    ASSERT_TRUE(isLesser(taskAcceleration1_, taskAcceleration2_));
}

TEST_F(ComparisonShould, ReturnIsLesserFalseForNonOrderedTaskAccelerationArguments) {
    ASSERT_FALSE(isLesser(taskAcceleration2_, taskAcceleration1_));
}

TEST_F(ComparisonShould, ReturnIsLesserTrueForOrderedTaskForceTorqueArguments) {
    ASSERT_TRUE(isLesser(taskForceTorque1_, taskForceTorque2_));
}

TEST_F(ComparisonShould, ReturnIsLesserFalseForNonOrderedTaskForceTorqueArguments) {
    ASSERT_FALSE(isLesser(taskForceTorque2_, taskForceTorque1_));
}

TEST_F(ComparisonShould, ReturnIsGreaterTrueForOrderedVectorXdArguments) {
    ASSERT_TRUE(isGreater(vectorXd2_, vectorXd1_));
}

TEST_F(ComparisonShould, ReturnIsGreaterFalseForNonOrderedVectorXdArguments) {
    ASSERT_FALSE(isGreater(vectorXd1_, vectorXd2_));
}

TEST_F(ComparisonShould, ThrowIsGreaterOnVectorXdArgumentsOfDifferentSizes) {
    ASSERT_THROW(isGreater(vectorXd4_, vectorXd1_), std::invalid_argument);
    ASSERT_THROW(isGreater(vectorXd2_, vectorXd4_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsGreaterTrueForOrderedJointPositionsArguments) {
    ASSERT_TRUE(isGreater(jointPositions2_, jointPositions1_));
}

TEST_F(ComparisonShould, ReturnIsGreaterFalseForNonOrderedJointPositionsArguments) {
    ASSERT_FALSE(isGreater(jointPositions1_, jointPositions2_));
}

TEST_F(ComparisonShould, ThrowIsGreaterOnJointPositionsArgumentsOfDifferentSizes) {
    ASSERT_THROW(isGreater(jointPositions4_, jointPositions1_), std::invalid_argument);
    ASSERT_THROW(isGreater(jointPositions2_, jointPositions4_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsGreaterTrueForOrderedJointVelocitiesArguments) {
    ASSERT_TRUE(isGreater(jointVelocities2_, jointVelocities1_));
}

TEST_F(ComparisonShould, ReturnIsGreaterFalseForNonOrderedJointVelocitiesArguments) {
    ASSERT_FALSE(isGreater(jointVelocities1_, jointVelocities2_));
}

TEST_F(ComparisonShould, ThrowIsGreaterOnJointVelocitiesArgumentsOfDifferentSizes) {
    ASSERT_THROW(isGreater(jointVelocities4_, jointVelocities1_), std::invalid_argument);
    ASSERT_THROW(isGreater(jointVelocities2_, jointVelocities4_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsGreaterTrueForOrderedJointAccelerationsArguments) {
    ASSERT_TRUE(isGreater(jointAccelerations2_, jointAccelerations1_));
}

TEST_F(ComparisonShould, ReturnIsGreaterFalseForNonOrderedJointAccelerationsArguments) {
    ASSERT_FALSE(isGreater(jointAccelerations1_, jointAccelerations2_));
}

TEST_F(ComparisonShould, ThrowIsGreaterOnJointAccelerationsArgumentsOfDifferentSizes) {
    ASSERT_THROW(isGreater(jointAccelerations4_, jointAccelerations1_), std::invalid_argument);
    ASSERT_THROW(isGreater(jointAccelerations2_, jointAccelerations4_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsGreaterTrueForOrderedJointForceTorquesArguments) {
    ASSERT_TRUE(isGreater(jointForceTorques2_, jointForceTorques1_));
}

TEST_F(ComparisonShould, ReturnIsGreaterFalseForNonOrderedJointForceTorquesArguments) {
    ASSERT_FALSE(isGreater(jointForceTorques1_, jointForceTorques2_));
}

TEST_F(ComparisonShould, ThrowIsGreaterOnJointForceTorquesArgumentsOfDifferentSizes) {
    ASSERT_THROW(isGreater(jointForceTorques4_, jointForceTorques1_), std::invalid_argument);
    ASSERT_THROW(isGreater(jointForceTorques2_, jointForceTorques4_), std::invalid_argument);
}

TEST_F(ComparisonShould, ReturnIsGreaterTrueForOrderedVector6dArguments) {
    ASSERT_TRUE(isGreater(vector6d2_, vector6d1_));
}

TEST_F(ComparisonShould, ReturnIsGreaterFalseForNonOrderedVector6dArguments) {
    ASSERT_FALSE(isGreater(vector6d1_, vector6d2_));
}
TEST_F(ComparisonShould, ReturnIsGreaterTrueForOrderedTaskVelocityArguments) {
    ASSERT_TRUE(isGreater(taskVelocity2_, taskVelocity1_));
}

TEST_F(ComparisonShould, ReturnIsGreaterFalseForNonOrderedTaskVelocityArguments) {
    ASSERT_FALSE(isGreater(taskVelocity1_, taskVelocity2_));
}

TEST_F(ComparisonShould, ReturnIsGreaterTrueForOrderedTaskAccelerationArguments) {
    ASSERT_TRUE(isGreater(taskAcceleration2_, taskAcceleration1_));
}

TEST_F(ComparisonShould, ReturnIsGreaterFalseForNonOrderedTaskAccelerationArguments) {
    ASSERT_FALSE(isGreater(taskAcceleration1_, taskAcceleration2_));
}

TEST_F(ComparisonShould, ReturnIsGreaterTrueForOrderedTaskForceTorqueArguments) {
    ASSERT_TRUE(isGreater(taskForceTorque2_, taskForceTorque1_));
}

TEST_F(ComparisonShould, ReturnIsGreaterFalseForNonOrderedTaskForceTorqueArguments) {
    ASSERT_FALSE(isGreater(taskForceTorque1_, taskForceTorque2_));
}
