/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *         Bartosz Sójka CERN BE/CM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <exception>

#include "CommunicationUtility/StdJsonConverters.hpp"
#include "Types/Conversions.hpp"
#include "Types/JointTypes/JointTypes.hpp"
#include "Types/TaskTypes/TaskTypes.hpp"

using crf::utility::communicationutility::stdArrayDoubleFromJson;
using crf::utility::communicationutility::jsonFromStdArrayDouble;
using crf::utility::communicationutility::stdVectorDoubleFromJson;
using crf::utility::communicationutility::jsonFromStdVectorDouble;

// Joints

using crf::utility::types::VectorXd;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

using crf::utility::types::stdVectorFromEigenVector;
using crf::utility::types::eigenVectorFromStdVector;

// Task

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;
using crf::math::rotation::Orientation;
using crf::math::rotation::OrientationRepresentation;
using crf::utility::types::TaskPose;
using crf::utility::types::Vector6d;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::TaskSpace;
using crf::utility::types::TaskSpaceTangentDimension;

using crf::utility::types::stdArrayFromEigenVector;
using crf::utility::types::eigenVectorFromStdArray;

using crf::math::rotation::quaternionFromArray;
using crf::math::rotation::arrayFromQuaternion;
using crf::math::rotation::matrixFromArray;
using crf::math::rotation::arrayFromMatrix;
using crf::math::rotation::angleAxisFromArray;
using crf::math::rotation::arrayFromAngleAxis;

namespace nlohmann {

template <>
struct adl_serializer<JointPositions> {
    static JointPositions from_json(const json& j);
    static void to_json(json& j, const VectorXd& t);  // NOLINT
};

template <>
struct adl_serializer<JointVelocities> {
    static JointVelocities from_json(const json& j);
    static void to_json(json& j, const VectorXd& t);  // NOLINT
};

template <>
struct adl_serializer<JointAccelerations> {
    static JointAccelerations from_json(const json& j);
    static void to_json(json& j, const VectorXd& t);  // NOLINT
};

template <>
struct adl_serializer<JointForceTorques> {
    static JointForceTorques from_json(const json& j);
    static void to_json(json& j, const VectorXd& t);  // NOLINT
};

template <>
struct adl_serializer<TaskPose> {
    static TaskPose from_json(const json& j);
    static void to_json(json& j, const TaskPose& t);  // NOLINT
};

template <>
struct adl_serializer<TaskVelocity> {
    static TaskVelocity from_json(const json& j);
    static void to_json(json& j, const Vector6d& t);  // NOLINT
};

template <>
struct adl_serializer<TaskAcceleration> {
    static TaskAcceleration from_json(const json& j);
    static void to_json(json& j, const Vector6d& t);  // NOLINT
};

template <>
struct adl_serializer<TaskForceTorque> {
    static TaskForceTorque from_json(const json& j);
    static void to_json(json& j, const Vector6d& t);  // NOLINT
};

template <> struct adl_serializer<TaskSpace> {
    static TaskSpace from_json(const json& j);
    static void to_json(json& j, const TaskSpace& t);  // NOLINT
};

}  // namespace nlohmann
