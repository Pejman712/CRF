/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Comparison.hpp"
#include "Types/JsonConverters.hpp"

#include <fstream>
#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::VectorXd;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;
using crf::math::rotation::Orientation;
using crf::math::rotation::OrientationRepresentation;
using crf::utility::types::TaskPose;
using crf::utility::types::Vector6d;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;

class JsonConvertersShould : public ::testing::Test {
 protected:
    JsonConvertersShould() :
        jointTypesVector3_(
            {-1.2452654789023454,
             -std::numeric_limits<double>::infinity(),
             2.8456356344354765,
             4.0,
             std::numeric_limits<double>::infinity(),
             0.0001231242,
             5.22784}),
        jointPositions3_(jointTypesVector3_),
        jointVelocities3_(jointTypesVector3_),
        jointAccelerations3_(jointTypesVector3_),
        jointForceTorques3_(jointTypesVector3_),
        position1_({-2.537896, 3.451273, -23.45126754}),
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
        taskPoseQuaternion1_(position1_, quaternion1_),
        taskPoseMatrix1_(position1_, matrix1_),
        taskPoseAngleAxis1_(position1_, angleAxis1_),
        taskPoseCardanXYZ1_(position1_, cardanXYZ1_),
        taskPoseEulerZXZ1_(position1_, eulerZXZ1_),
        taskTypesArray1_(
            {2.2,
             -std::numeric_limits<double>::infinity(),
             4.7,
             std::numeric_limits<double>::infinity(),
             4.7,
             3.9}),
        taskVelocity1_(taskTypesArray1_),
        taskAcceleration1_(taskTypesArray1_),
        taskForceTorque1_(taskTypesArray1_),
        taskSpace1_({true, false, true, true, false, false}),
        jsonJointPositions3_(),
        jsonJointVelocities3_(),
        jsonJointAccelerations3_(),
        jsonJointForceTorques3_(),
        jsonTaskPoseQuaternion1_(),
        jsonTaskPoseMatrix1_(),
        jsonTaskPoseAngleAxis1_(),
        jsonTaskPoseCardanXYZ1_(),
        jsonTaskPoseEulerZXZ1_(),
        jsonTaskVelocity1_(),
        jsonTaskAcceleration1_(),
        jsonTaskForceTorque1_(),
        jsonTaskSpace1_(),
        jsonJointPositions3FromFile_(),
        jsonJointVelocities3FromFile_(),
        jsonJointAccelerations3FromFile_(),
        jsonJointForceTorques3FromFile_(),
        jsonTaskPoseQuaternion1FromFile_(),
        jsonTaskPoseMatrix1FromFile_(),
        jsonTaskPoseAngleAxis1FromFile_(),
        jsonTaskPoseCardanXYZ1FromFile_(),
        jsonTaskPoseEulerZXZ1FromFile_(),
        jsonTaskVelocity1FromFile_(),
        jsonTaskAcceleration1FromFile_(),
        jsonTaskForceTorque1FromFile_(),
        jsonTaskSpace1FromFile_(),
        outputJson_() {
        jsonJointPositions3_ = {
            -1.2452654789023454, "-inf", 2.8456356344354765, 4.0, "inf", 0.0001231242, 5.22784};
        jsonJointVelocities3_ = {
            -1.2452654789023454, "-inf", 2.8456356344354765, 4.0, "inf", 0.0001231242, 5.22784};
        jsonJointAccelerations3_ = {
            -1.2452654789023454, "-inf", 2.8456356344354765, 4.0, "inf", 0.0001231242, 5.22784};
        jsonJointForceTorques3_ = {
            -1.2452654789023454, "-inf", 2.8456356344354765, 4.0, "inf", 0.0001231242, 5.22784};
        jsonTaskPoseQuaternion1_["Position"] = {-2.537896, 3.451273, -23.45126754};
        jsonTaskPoseQuaternion1_["Orientation"]["Quaternion"]["W"] = 0.5505666516902112;
        jsonTaskPoseQuaternion1_["Orientation"]["Quaternion"]["X"] = -0.6362152372281484;
        jsonTaskPoseQuaternion1_["Orientation"]["Quaternion"]["Y"] = 0.3613804315900029;
        jsonTaskPoseQuaternion1_["Orientation"]["Quaternion"]["Z"] = 0.4018839604029799;
        jsonTaskPoseMatrix1_["Position"] = {-2.537896, 3.451273, -23.45126754};
        jsonTaskPoseMatrix1_["Orientation"]["Matrix"]["Row1"] = {
            0.4157869320692789, -0.9023592869214287, -0.1134413699981963};
        jsonTaskPoseMatrix1_["Orientation"]["Matrix"]["Row2"] = {
            -0.0173036611331485, -0.1325610914209059, 0.9910237839490472};
        jsonTaskPoseMatrix1_["Orientation"]["Matrix"]["Row3"] = {
            -0.9092974268256818, -0.4100917877109334, -0.0707312888348917};
        jsonTaskPoseAngleAxis1_["Position"] = {-2.537896, 3.451273, -23.45126754};
        jsonTaskPoseAngleAxis1_["Orientation"]["AngleAxis"]["Angle"] = 1.9755068924749106;
        jsonTaskPoseAngleAxis1_["Orientation"]["AngleAxis"]["Axis"] = {
            -0.7621249848254679, 0.4328991822668132, 0.4814185346426796};
        jsonTaskPoseCardanXYZ1_["Position"] = {-2.537896, 3.451273, -23.45126754};
        jsonTaskPoseCardanXYZ1_["Orientation"]["CardanXYZ"]["X"] = 1.4;
        jsonTaskPoseCardanXYZ1_["Orientation"]["CardanXYZ"]["Y"] = 2.0;
        jsonTaskPoseCardanXYZ1_["Orientation"]["CardanXYZ"]["Z"] = 3.1;
        jsonTaskPoseEulerZXZ1_["Position"] = {-2.537896, 3.451273, -23.45126754};
        jsonTaskPoseEulerZXZ1_["Orientation"]["EulerZXZ"]["Z1"] = 1.1471123464639490;
        jsonTaskPoseEulerZXZ1_["Orientation"]["EulerZXZ"]["X"] = -1.6415867259093058;
        jsonTaskPoseEulerZXZ1_["Orientation"]["EulerZXZ"]["Z2"] = 0.1139727950424842;
        jsonTaskVelocity1_ = {2.2, "-inf", 4.7, "inf", 4.7, 3.9};
        jsonTaskAcceleration1_ = {2.2, "-inf", 4.7, "inf", 4.7, 3.9};
        jsonTaskForceTorque1_ = {2.2, "-inf", 4.7, "inf", 4.7, 3.9};
        jsonTaskSpace1_["Vx"] = true;
        jsonTaskSpace1_["Vy"] = false;
        jsonTaskSpace1_["Vz"] = true;
        jsonTaskSpace1_["Wx"] = true;
        jsonTaskSpace1_["Wy"] = false;
        jsonTaskSpace1_["Wz"] = false;

        std::string typesJsonPath = __FILE__;
        typesJsonPath = typesJsonPath.substr(0, typesJsonPath.find("cpproboticframework"));
        typesJsonPath += "cpproboticframework/modules/Utility/Types/tests/config/Types.json";
        std::ifstream typesJsonFile(typesJsonPath);
        nlohmann::json typesJson = nlohmann::json::parse(typesJsonFile);

        jsonJointPositions3FromFile_ = typesJson["JointPositions"];
        jsonJointVelocities3FromFile_ = typesJson["JointVelocities"];
        jsonJointAccelerations3FromFile_ = typesJson["JointAccelerations"];
        jsonJointForceTorques3FromFile_ = typesJson["JointForceTorques"];
        jsonTaskPoseQuaternion1FromFile_ = typesJson["TaskPoseQuaternion"];
        jsonTaskPoseMatrix1FromFile_ = typesJson["TaskPoseMatrix"];
        jsonTaskPoseAngleAxis1FromFile_ = typesJson["TaskPoseAngleAxis"];
        jsonTaskPoseCardanXYZ1FromFile_ = typesJson["TaskPoseCardanXYZ"];
        jsonTaskPoseEulerZXZ1FromFile_ = typesJson["TaskPoseEulerZXZ"];
        jsonTaskVelocity1FromFile_ = typesJson["TaskVelocity"];
        jsonTaskAcceleration1FromFile_ = typesJson["TaskAcceleration"];
        jsonTaskForceTorque1FromFile_ = typesJson["TaskForceTorque"];
        jsonTaskSpace1FromFile_ = typesJson["TaskSpace"];
    }

    void SetUp() override {
        outputJson_ = nlohmann::json();
    }

    const std::vector<double> jointTypesVector3_;
    const JointPositions jointPositions3_;
    const JointVelocities jointVelocities3_;
    const JointAccelerations jointAccelerations3_;
    const JointForceTorques jointForceTorques3_;

    const Eigen::Vector3d position1_;
    const Eigen::Quaterniond quaternion1_;
    const Eigen::Matrix3d matrix1_;
    const Eigen::AngleAxisd angleAxis1_;
    const CardanXYZ cardanXYZ1_;
    const EulerZXZ eulerZXZ1_;

    const TaskPose taskPoseQuaternion1_;
    const TaskPose taskPoseMatrix1_;
    const TaskPose taskPoseAngleAxis1_;
    const TaskPose taskPoseCardanXYZ1_;
    const TaskPose taskPoseEulerZXZ1_;

    const std::array<double, 6> taskTypesArray1_;
    const TaskVelocity taskVelocity1_;
    const TaskAcceleration taskAcceleration1_;
    const TaskForceTorque taskForceTorque1_;
    const TaskSpace taskSpace1_;

    nlohmann::json jsonJointPositions3_;
    nlohmann::json jsonJointVelocities3_;
    nlohmann::json jsonJointAccelerations3_;
    nlohmann::json jsonJointForceTorques3_;
    nlohmann::json jsonTaskPoseQuaternion1_;
    nlohmann::json jsonTaskPoseMatrix1_;
    nlohmann::json jsonTaskPoseAngleAxis1_;
    nlohmann::json jsonTaskPoseCardanXYZ1_;
    nlohmann::json jsonTaskPoseEulerZXZ1_;
    nlohmann::json jsonTaskVelocity1_;
    nlohmann::json jsonTaskAcceleration1_;
    nlohmann::json jsonTaskForceTorque1_;
    nlohmann::json jsonTaskSpace1_;

    nlohmann::json jsonJointPositions3FromFile_;
    nlohmann::json jsonJointVelocities3FromFile_;
    nlohmann::json jsonJointAccelerations3FromFile_;
    nlohmann::json jsonJointForceTorques3FromFile_;
    nlohmann::json jsonTaskPoseQuaternion1FromFile_;
    nlohmann::json jsonTaskPoseMatrix1FromFile_;
    nlohmann::json jsonTaskPoseAngleAxis1FromFile_;
    nlohmann::json jsonTaskPoseCardanXYZ1FromFile_;
    nlohmann::json jsonTaskPoseEulerZXZ1FromFile_;
    nlohmann::json jsonTaskVelocity1FromFile_;
    nlohmann::json jsonTaskAcceleration1FromFile_;
    nlohmann::json jsonTaskForceTorque1FromFile_;
    nlohmann::json jsonTaskSpace1FromFile_;

    nlohmann::json outputJson_;
};

TEST_F(JsonConvertersShould, GiveCorrectJointPositions) {
    ASSERT_TRUE(areAlmostEqual(jsonJointPositions3_.get<JointPositions>(), jointPositions3_));
}

TEST_F(JsonConvertersShould, GiveCorrectJointPositionsJson) {
    ASSERT_NO_THROW(outputJson_ = jointPositions3_);
    ASSERT_EQ(outputJson_, jsonJointPositions3_);
}

TEST_F(JsonConvertersShould, GiveCorrectJointVelocities) {
    ASSERT_TRUE(areAlmostEqual(jsonJointVelocities3_.get<JointVelocities>(), jointVelocities3_));
}

TEST_F(JsonConvertersShould, GiveCorrectJointVelocitiesJson) {
    ASSERT_NO_THROW(outputJson_ = jointVelocities3_);
    ASSERT_EQ(outputJson_, jsonJointVelocities3_);
}

TEST_F(JsonConvertersShould, GiveCorrectJointAccelerations) {
    ASSERT_TRUE(
        areAlmostEqual(jsonJointAccelerations3_.get<JointAccelerations>(), jointAccelerations3_));
}

TEST_F(JsonConvertersShould, GiveCorrectJointAccelerationsJson) {
    ASSERT_NO_THROW(outputJson_ = jointAccelerations3_);
    ASSERT_EQ(outputJson_, jsonJointAccelerations3_);
}

TEST_F(JsonConvertersShould, GiveCorrectJointForceTorques) {
    ASSERT_TRUE(
        areAlmostEqual(jsonJointForceTorques3_.get<JointForceTorques>(), jointForceTorques3_));
}

TEST_F(JsonConvertersShould, GiveCorrectJointForceTorquesJson) {
    ASSERT_NO_THROW(outputJson_ = jointForceTorques3_);
    ASSERT_EQ(outputJson_, jsonJointForceTorques3_);
}

TEST_F(JsonConvertersShould, GiveCorrectTaskPose) {
    ASSERT_TRUE(areAlmostEqual(jsonTaskPoseQuaternion1_.get<TaskPose>(), taskPoseQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(jsonTaskPoseMatrix1_.get<TaskPose>(), taskPoseMatrix1_));
    ASSERT_TRUE(areAlmostEqual(jsonTaskPoseAngleAxis1_.get<TaskPose>(), taskPoseAngleAxis1_));
    ASSERT_TRUE(areAlmostEqual(jsonTaskPoseCardanXYZ1_.get<TaskPose>(), taskPoseCardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(jsonTaskPoseEulerZXZ1_.get<TaskPose>(), taskPoseEulerZXZ1_));
}

TEST_F(JsonConvertersShould, GiveCorrectTaskPoseJson) {
    ASSERT_NO_THROW(outputJson_ = taskPoseQuaternion1_);
    ASSERT_EQ(outputJson_, jsonTaskPoseQuaternion1_);

    ASSERT_NO_THROW(outputJson_ = taskPoseMatrix1_);
    ASSERT_EQ(outputJson_, jsonTaskPoseMatrix1_);

    ASSERT_NO_THROW(outputJson_ = taskPoseAngleAxis1_);
    ASSERT_EQ(outputJson_, jsonTaskPoseAngleAxis1_);

    ASSERT_NO_THROW(outputJson_ = taskPoseCardanXYZ1_);
    ASSERT_EQ(outputJson_, jsonTaskPoseCardanXYZ1_);

    ASSERT_NO_THROW(outputJson_ = taskPoseEulerZXZ1_);
    ASSERT_EQ(outputJson_, jsonTaskPoseEulerZXZ1_);
}

TEST_F(JsonConvertersShould, GiveCorrectTaskVelocity) {
    ASSERT_TRUE(areAlmostEqual(jsonTaskVelocity1_.get<TaskVelocity>(), taskVelocity1_));
}

TEST_F(JsonConvertersShould, GiveCorrectTaskVelocityJson) {
    ASSERT_NO_THROW(outputJson_ = taskVelocity1_);
    ASSERT_EQ(outputJson_, jsonTaskVelocity1_);
}

TEST_F(JsonConvertersShould, GiveCorrectTaskAcceleration) {
    ASSERT_TRUE(areAlmostEqual(jsonTaskAcceleration1_.get<TaskAcceleration>(), taskAcceleration1_));
}

TEST_F(JsonConvertersShould, GiveCorrectTaskAccelerationJson) {
    ASSERT_NO_THROW(outputJson_ = taskAcceleration1_);
    ASSERT_EQ(outputJson_, jsonTaskAcceleration1_);
}

TEST_F(JsonConvertersShould, GiveCorrectTaskForceTorque) {
    ASSERT_TRUE(areAlmostEqual(jsonTaskForceTorque1_.get<TaskForceTorque>(), taskForceTorque1_));
}

TEST_F(JsonConvertersShould, GiveCorrectTaskForceTorqueJson) {
    ASSERT_NO_THROW(outputJson_ = taskForceTorque1_);
    ASSERT_EQ(outputJson_, jsonTaskForceTorque1_);
}

TEST_F(JsonConvertersShould, GiveCorrectTaskSpace) {
    ASSERT_TRUE(areEqual(jsonTaskSpace1_.get<TaskSpace>(), taskSpace1_));
}

TEST_F(JsonConvertersShould, GiveCorrectTaskSpaceJson) {
    ASSERT_NO_THROW(outputJson_ = taskSpace1_);
    ASSERT_EQ(outputJson_, jsonTaskSpace1_);
}

TEST_F(JsonConvertersShould, GiveCorrectJointPositionsFromFile) {
    ASSERT_TRUE(
        areAlmostEqual(jsonJointPositions3FromFile_.get<JointPositions>(), jointPositions3_));
}

TEST_F(JsonConvertersShould, GiveCorrectJointPositionsJsonFromFile) {
    ASSERT_NO_THROW(outputJson_ = jointPositions3_);
    ASSERT_EQ(outputJson_, jsonJointPositions3FromFile_);
}

TEST_F(JsonConvertersShould, GiveCorrectJointVelocitiesFromFile) {
    ASSERT_TRUE(
        areAlmostEqual(jsonJointVelocities3FromFile_.get<JointVelocities>(), jointVelocities3_));
}

TEST_F(JsonConvertersShould, GiveCorrectJointVelocitiesJsonFromFile) {
    ASSERT_NO_THROW(outputJson_ = jointVelocities3_);
    ASSERT_EQ(outputJson_, jsonJointVelocities3FromFile_);
}

TEST_F(JsonConvertersShould, GiveCorrectJointAccelerationsFromFile) {
    ASSERT_TRUE(areAlmostEqual(
        jsonJointAccelerations3FromFile_.get<JointAccelerations>(), jointAccelerations3_));
}

TEST_F(JsonConvertersShould, GiveCorrectJointAccelerationsJsonFromFile) {
    ASSERT_NO_THROW(outputJson_ = jointAccelerations3_);
    ASSERT_EQ(outputJson_, jsonJointAccelerations3FromFile_);
}

TEST_F(JsonConvertersShould, GiveCorrectJointForceTorquesFromFile) {
    ASSERT_TRUE(areAlmostEqual(
        jsonJointForceTorques3FromFile_.get<JointForceTorques>(), jointForceTorques3_));
}

TEST_F(JsonConvertersShould, GiveCorrectJointForceTorquesJsonFromFile) {
    ASSERT_NO_THROW(outputJson_ = jointForceTorques3_);
    ASSERT_EQ(outputJson_, jsonJointForceTorques3FromFile_);
}

TEST_F(JsonConvertersShould, GiveCorrectTaskPoseFromFile) {
    ASSERT_TRUE(
        areAlmostEqual(jsonTaskPoseQuaternion1FromFile_.get<TaskPose>(), taskPoseQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(jsonTaskPoseMatrix1FromFile_.get<TaskPose>(), taskPoseMatrix1_));
    ASSERT_TRUE(
        areAlmostEqual(jsonTaskPoseAngleAxis1FromFile_.get<TaskPose>(), taskPoseAngleAxis1_));
    ASSERT_TRUE(
        areAlmostEqual(jsonTaskPoseCardanXYZ1FromFile_.get<TaskPose>(), taskPoseCardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(jsonTaskPoseEulerZXZ1FromFile_.get<TaskPose>(), taskPoseEulerZXZ1_));
}

TEST_F(JsonConvertersShould, GiveCorrectTaskPoseJsonFromFile) {
    ASSERT_NO_THROW(outputJson_ = taskPoseQuaternion1_);
    ASSERT_EQ(outputJson_, jsonTaskPoseQuaternion1FromFile_);

    ASSERT_NO_THROW(outputJson_ = taskPoseMatrix1_);
    ASSERT_EQ(outputJson_, jsonTaskPoseMatrix1FromFile_);

    ASSERT_NO_THROW(outputJson_ = taskPoseAngleAxis1_);
    ASSERT_EQ(outputJson_, jsonTaskPoseAngleAxis1FromFile_);

    ASSERT_NO_THROW(outputJson_ = taskPoseCardanXYZ1_);
    ASSERT_EQ(outputJson_, jsonTaskPoseCardanXYZ1FromFile_);

    ASSERT_NO_THROW(outputJson_ = taskPoseEulerZXZ1_);
    ASSERT_EQ(outputJson_, jsonTaskPoseEulerZXZ1FromFile_);
}

TEST_F(JsonConvertersShould, GiveCorrectTaskVelocityFromFile) {
    ASSERT_TRUE(areAlmostEqual(jsonTaskVelocity1FromFile_.get<TaskVelocity>(), taskVelocity1_));
}

TEST_F(JsonConvertersShould, GiveCorrectTaskVelocityJsonFromFile) {
    ASSERT_NO_THROW(outputJson_ = taskVelocity1_);
    ASSERT_EQ(outputJson_, jsonTaskVelocity1FromFile_);
}

TEST_F(JsonConvertersShould, GiveCorrectTaskAccelerationFromFile) {
    ASSERT_TRUE(
        areAlmostEqual(jsonTaskAcceleration1FromFile_.get<TaskAcceleration>(), taskAcceleration1_));
}

TEST_F(JsonConvertersShould, GiveCorrectTaskAccelerationJsonFromFile) {
    ASSERT_NO_THROW(outputJson_ = taskAcceleration1_);
    ASSERT_EQ(outputJson_, jsonTaskAcceleration1FromFile_);
}

TEST_F(JsonConvertersShould, GiveCorrectTaskForceTorqueFromFile) {
    ASSERT_TRUE(
        areAlmostEqual(jsonTaskForceTorque1FromFile_.get<TaskForceTorque>(), taskForceTorque1_));
}

TEST_F(JsonConvertersShould, GiveCorrectTaskForceTorqueJsonFromFile) {
    ASSERT_NO_THROW(outputJson_ = taskForceTorque1_);
    ASSERT_EQ(outputJson_, jsonTaskForceTorque1FromFile_);
}

TEST_F(JsonConvertersShould, GiveCorrectTaskSpaceFromFile) {
    ASSERT_TRUE(areEqual(jsonTaskSpace1FromFile_.get<TaskSpace>(), taskSpace1_));
}

TEST_F(JsonConvertersShould, GiveCorrectTaskSpaceJsonFromFile) {
    ASSERT_NO_THROW(outputJson_ = taskSpace1_);
    ASSERT_EQ(outputJson_, jsonTaskSpace1FromFile_);
}
