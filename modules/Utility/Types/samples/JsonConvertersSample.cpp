/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/JsonConverters.hpp"

using crf::utility::types::JointPositions;

using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;

int main() {
    nlohmann::json outputJson;

    /**
     * JointPositions
     * Types JointVelocities, JointAccelerations, JointForceTorques abides the same
     * convention as JointPositions.
    */
    JointPositions outputJointPositions;
    JointPositions jointPositions3(
        {-1.2452654789023454,
         std::numeric_limits<double>::infinity(),
         2.8456356344354765,
         4.0,
         23.134142312313420,
         -std::numeric_limits<double>::infinity(),
         5.22784});
    nlohmann::json jsonJointPositions3;
    jsonJointPositions3 = {
        -1.2452654789023454, "inf", 2.8456356344354765, 4.0, 23.134142312313420, "-inf", 5.22784};
    /**
     * From json
    */
    outputJointPositions = jsonJointPositions3.get<JointPositions>();
    /**
     * To json
    */
    outputJson = jointPositions3;

    /**
     * TaskPose
    */
    TaskPose outputTaskPose;
    Eigen::Vector3d position1({-2.537896, 3.451273, -23.45126754});
    Eigen::Quaterniond quaternion1(
        {0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799});
    Eigen::Matrix3d matrix1(
        {{0.4157869320692789, -0.9023592869214287, -0.1134413699981963},
         {-0.0173036611331485, -0.1325610914209059, 0.9910237839490472},
         {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917}});
    Eigen::AngleAxisd angleAxis1(
        1.9755068924749106,
        Eigen::Vector3d({-0.7621249848254679, 0.4328991822668132, 0.4814185346426796}));
    CardanXYZ cardanXYZ1({1.4, 2.0, 3.1});
    EulerZXZ eulerZXZ1({1.1471123464639490, -1.6415867259093058, 0.1139727950424842});
    TaskPose taskPoseQuaternion1(position1, quaternion1);
    TaskPose taskPoseMatrix1(position1, matrix1);
    TaskPose taskPoseAngleAxis1(position1, angleAxis1);
    TaskPose taskPoseCardanXYZ1(position1, cardanXYZ1);
    TaskPose taskPoseEulerZXZ1(position1, eulerZXZ1);
    nlohmann::json jsonTaskPoseQuaternion1;
    jsonTaskPoseQuaternion1["Position"] = {-2.537896, 3.451273, -23.45126754};
    jsonTaskPoseQuaternion1["Orientation"]["Quaternion"]["W"] = 0.5505666516902112;
    jsonTaskPoseQuaternion1["Orientation"]["Quaternion"]["X"] = -0.6362152372281484;
    jsonTaskPoseQuaternion1["Orientation"]["Quaternion"]["Y"] = 0.3613804315900029;
    jsonTaskPoseQuaternion1["Orientation"]["Quaternion"]["Z"] = 0.4018839604029799;
    nlohmann::json jsonTaskPoseMatrix1;
    jsonTaskPoseMatrix1["Position"] = {-2.537896, 3.451273, -23.45126754};
    jsonTaskPoseMatrix1["Orientation"]["Matrix"]["Row1"] = {
        0.4157869320692789, -0.9023592869214287, -0.1134413699981963};
    jsonTaskPoseMatrix1["Orientation"]["Matrix"]["Row2"] = {
        -0.0173036611331485, -0.1325610914209059, 0.9910237839490472};
    jsonTaskPoseMatrix1["Orientation"]["Matrix"]["Row3"] = {
        -0.9092974268256818, -0.4100917877109334, -0.0707312888348917};
    nlohmann::json jsonTaskPoseAngleAxis1;
    jsonTaskPoseAngleAxis1["Position"] = {-2.537896, 3.451273, -23.45126754};
    jsonTaskPoseAngleAxis1["Orientation"]["AngleAxis"]["Angle"] = 1.9755068924749106;
    jsonTaskPoseAngleAxis1["Orientation"]["AngleAxis"]["Axis"] = {
        -0.7621249848254679, 0.4328991822668132, 0.4814185346426796};
    nlohmann::json jsonTaskPoseCardanXYZ1;
    jsonTaskPoseCardanXYZ1["Position"] = {-2.537896, 3.451273, -23.45126754};
    jsonTaskPoseCardanXYZ1["Orientation"]["CardanXYZ"]["X"] = 1.4;
    jsonTaskPoseCardanXYZ1["Orientation"]["CardanXYZ"]["Y"] = 2.0;
    jsonTaskPoseCardanXYZ1["Orientation"]["CardanXYZ"]["Z"] = 3.1;
    nlohmann::json jsonTaskPoseEulerZXZ1;
    jsonTaskPoseEulerZXZ1["Position"] = {-2.537896, 3.451273, -23.45126754};
    jsonTaskPoseEulerZXZ1["Orientation"]["EulerZXZ"]["Z1"] = 1.1471123464639490;
    jsonTaskPoseEulerZXZ1["Orientation"]["EulerZXZ"]["X"] = -1.6415867259093058;
    jsonTaskPoseEulerZXZ1["Orientation"]["EulerZXZ"]["Z2"] = 0.1139727950424842;
    /**
     * From json
    */
    outputTaskPose = jsonTaskPoseQuaternion1.get<TaskPose>();
    outputTaskPose = jsonTaskPoseMatrix1.get<TaskPose>();
    outputTaskPose = jsonTaskPoseAngleAxis1.get<TaskPose>();
    outputTaskPose = jsonTaskPoseCardanXYZ1.get<TaskPose>();
    outputTaskPose = jsonTaskPoseEulerZXZ1.get<TaskPose>();
    /**
     * To json
    */
    outputJson = taskPoseQuaternion1;
    outputJson = taskPoseMatrix1;
    outputJson = taskPoseAngleAxis1;
    outputJson = taskPoseCardanXYZ1;
    outputJson = taskPoseEulerZXZ1;

    /**
     * TaskVelocity
     * Types TaskAcceleration, TaskForceTorque abides the same convention as TaskVelocity.
    */
    TaskVelocity outputTaskVelocity;
    TaskVelocity taskVelocity1(
        {2.2,
         -3.8,
         std::numeric_limits<double>::infinity(),
         5.8,
         -std::numeric_limits<double>::infinity(),
         3.9});
    nlohmann::json jsonTaskVelocity1;
    jsonTaskVelocity1 = {2.2, -3.8, "inf", 5.8, "-inf", 3.9};
    /**
     * From json
    */
    outputTaskVelocity = jsonTaskVelocity1.get<TaskVelocity>();
    /**
     * To json
    */
    outputJson = taskVelocity1;

    return 0;
}
