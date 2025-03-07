/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *         Jorge Playan Garai CERN BE/CEM/MRO 2023
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include "Types/JsonConverters.hpp"

namespace nlohmann {

// Joint Space

JointPositions adl_serializer<JointPositions>::from_json(const json& json) {
    return JointPositions(stdVectorDoubleFromJson(json));
}

void adl_serializer<JointPositions>::to_json(json& json, const VectorXd& vector) {  // NOLINT
    json = jsonFromStdVectorDouble(stdVectorFromEigenVector(vector.raw()));
}

JointVelocities adl_serializer<JointVelocities>::from_json(const json& json) {
    return JointVelocities(stdVectorDoubleFromJson(json));
}

void adl_serializer<JointVelocities>::to_json(json& json, const VectorXd& vector) {  // NOLINT
    json = jsonFromStdVectorDouble(stdVectorFromEigenVector(vector.raw()));
}

JointAccelerations adl_serializer<JointAccelerations>::from_json(const json& json) {
    return JointAccelerations(stdVectorDoubleFromJson(json));
}

void adl_serializer<JointAccelerations>::to_json(json& json, const VectorXd& vector) {  // NOLINT
    json = jsonFromStdVectorDouble(stdVectorFromEigenVector(vector.raw()));
}

JointForceTorques adl_serializer<JointForceTorques>::from_json(const json& json) {
    return JointForceTorques(stdVectorDoubleFromJson(json));
}

void adl_serializer<JointForceTorques>::to_json(json& json, const VectorXd& vector) {  // NOLINT
    json = jsonFromStdVectorDouble(stdVectorFromEigenVector(vector.raw()));
}

// Task Space

TaskPose adl_serializer<TaskPose>::from_json(const json& json) {
    std::array<double, 3> positionStdArray = stdArrayDoubleFromJson<3>(json["Position"]);
    Eigen::Vector3d position(eigenVectorFromStdArray<3>(positionStdArray));
    Orientation orientation(json["Orientation"].get<Orientation>());
    return TaskPose(position, orientation);
}

void adl_serializer<TaskPose>::to_json(json& json, const TaskPose& pose) {  // NOLINT
    Eigen::Vector3d position = pose.getPosition();
    Orientation orientation = pose.getOrientation();
    json["Position"] = jsonFromStdArrayDouble<3>(stdArrayFromEigenVector<3>(position));
    json["Orientation"] = orientation;
}

TaskVelocity adl_serializer<TaskVelocity>::from_json(const json& json) {
    return TaskVelocity(stdArrayDoubleFromJson<6>(json));
}

void adl_serializer<TaskVelocity>::to_json(json& json, const Vector6d& vector) {  // NOLINT
    json = jsonFromStdArrayDouble<6>(stdArrayFromEigenVector<6>(vector.raw()));
}

TaskAcceleration adl_serializer<TaskAcceleration>::from_json(const json& json) {
    return TaskAcceleration(stdArrayDoubleFromJson<6>(json));
}

void adl_serializer<TaskAcceleration>::to_json(json& json, const Vector6d& vector) {  // NOLINT
    json = jsonFromStdArrayDouble<6>(stdArrayFromEigenVector<6>(vector.raw()));
}

TaskForceTorque adl_serializer<TaskForceTorque>::from_json(const json& json) {
    return TaskForceTorque(stdArrayDoubleFromJson<6>(json));
}
void adl_serializer<TaskForceTorque>::to_json(json& json, const Vector6d& vector) {  // NOLINT
    json = jsonFromStdArrayDouble<6>(stdArrayFromEigenVector<6>(vector.raw()));
}

TaskSpace adl_serializer<TaskSpace>::from_json(const json& json) {
    TaskSpace taskSpace;
    taskSpace[TaskSpaceTangentDimension::Vx] = json["Vx"].get<bool>();
    taskSpace[TaskSpaceTangentDimension::Vy] = json["Vy"].get<bool>();
    taskSpace[TaskSpaceTangentDimension::Vz] = json["Vz"].get<bool>();
    taskSpace[TaskSpaceTangentDimension::Wx] = json["Wx"].get<bool>();
    taskSpace[TaskSpaceTangentDimension::Wy] = json["Wy"].get<bool>();
    taskSpace[TaskSpaceTangentDimension::Wz] = json["Wz"].get<bool>();
    return taskSpace;
}

void adl_serializer<TaskSpace>::to_json(json& json, const TaskSpace& taskSpace) {  // NOLINT
    json["Vx"] = taskSpace[TaskSpaceTangentDimension::Vx];
    json["Vy"] = taskSpace[TaskSpaceTangentDimension::Vy];
    json["Vz"] = taskSpace[TaskSpaceTangentDimension::Vz];
    json["Wx"] = taskSpace[TaskSpaceTangentDimension::Wx];
    json["Wy"] = taskSpace[TaskSpaceTangentDimension::Wy];
    json["Wz"] = taskSpace[TaskSpaceTangentDimension::Wz];
}

}  // namespace nlohmann
