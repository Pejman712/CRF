/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include "Rotation/JsonConverters.hpp"

namespace nlohmann {

CardanXYZ adl_serializer<CardanXYZ>::from_json(const json& json) {
    CardanXYZ cardanXYZ;
    std::array<std::string, 3> axes({"X", "Y", "Z"});
    for (std::size_t i = 0; i < 3; i++) {
        cardanXYZ[i] = doubleFromJson(json[axes[i]]);
    }
    return cardanXYZ;
}

void adl_serializer<CardanXYZ>::to_json(json& json, const CardanXYZ& cardanXYZ) {  // NOLINT
    std::array<std::string, 3> axes({"X", "Y", "Z"});
    for (std::size_t i = 0; i < 3; i++) {
        json[axes[i]] = jsonFromDouble(cardanXYZ[i]);
    }
}

EulerZXZ adl_serializer<EulerZXZ>::from_json(const json& json) {
    EulerZXZ eulerZXZ;
    std::array<std::string, 3> axes({"Z1", "X", "Z2"});
    for (std::size_t i = 0; i < 3; i++) {
        eulerZXZ[i] = doubleFromJson(json[axes[i]]);
    }
    return eulerZXZ;
}

void adl_serializer<EulerZXZ>::to_json(json& json, const EulerZXZ& eulerZXZ) {  // NOLINT
    std::array<std::string, 3> axes({"Z1", "X", "Z2"});
    for (std::size_t i = 0; i < 3; i++) {
        json[axes[i]] = jsonFromDouble(eulerZXZ[i]);
    }
}

Rotation adl_serializer<Rotation>::from_json(const json& json) {
    if (json.contains("Quaternion")) {
        const double w(doubleFromJson(json["Quaternion"]["W"]));
        const double x(doubleFromJson(json["Quaternion"]["X"]));
        const double y(doubleFromJson(json["Quaternion"]["Y"]));
        const double z(doubleFromJson(json["Quaternion"]["Z"]));
        Eigen::Quaterniond quaternion({w, x, y, z});
        return Rotation(quaternion);
    }
    if (json.contains("Matrix")) {
        std::array<std::array<double, 3>, 3> matrixStdArray;
        matrixStdArray[0] = stdArrayDoubleFromJson<3>(json["Matrix"]["Row1"]);
        matrixStdArray[1] = stdArrayDoubleFromJson<3>(json["Matrix"]["Row2"]);
        matrixStdArray[2] = stdArrayDoubleFromJson<3>(json["Matrix"]["Row3"]);
        const Eigen::Matrix3d matrix(matrixFromArray(matrixStdArray));
        return Rotation(matrix);
    }
    if (json.contains("AngleAxis")) {
        const double angle(doubleFromJson(json["AngleAxis"]["Angle"]));
        const std::array<double, 3> axisArray(stdArrayDoubleFromJson<3>(json["AngleAxis"]["Axis"]));
        const Eigen::AngleAxisd angleAxis(
            angle, Eigen::Vector3d({axisArray[0], axisArray[1], axisArray[2]}));
        return Rotation(angleAxis);
    }
    if (json.contains("CardanXYZ")) {
        const CardanXYZ cardanXYZ(json["CardanXYZ"].get<CardanXYZ>());
        return Rotation(cardanXYZ);
    }
    if (json.contains("EulerZXZ")) {
        const EulerZXZ eulerZXZ(json["EulerZXZ"].get<EulerZXZ>());
        return Rotation(eulerZXZ);
    }
    throw std::logic_error("nlohmann::adl_serializer<Rotation>::from_json: "
        "Control reached an impossible point.");
}

void adl_serializer<Rotation>::to_json(json& json, const Rotation& rotation) {  // NOLINT
    RotationRepresentation representation = rotation.getRepresentation();
    if (representation == RotationRepresentation::Quaternion) {
        const Eigen::Quaterniond quaternion(rotation.getQuaternion());
        json["Quaternion"]["W"] = jsonFromDouble(quaternion.w());
        json["Quaternion"]["X"] = jsonFromDouble(quaternion.x());
        json["Quaternion"]["Y"] = jsonFromDouble(quaternion.y());
        json["Quaternion"]["Z"] = jsonFromDouble(quaternion.z());
    } else if (representation == RotationRepresentation::Matrix) {
        const std::array<std::array<double, 3>, 3> matrixArray(
            arrayFromMatrix(rotation.getMatrix()));
        json["Matrix"]["Row1"] = jsonFromStdArrayDouble<3>(matrixArray[0]);
        json["Matrix"]["Row2"] = jsonFromStdArrayDouble<3>(matrixArray[1]);
        json["Matrix"]["Row3"] = jsonFromStdArrayDouble<3>(matrixArray[2]);
    } else if (representation == RotationRepresentation::AngleAxis) {
        const Eigen::AngleAxisd angleAxis(rotation.getAngleAxis());
        json["AngleAxis"]["Angle"] = jsonFromDouble(angleAxis.angle());
        const Eigen::Vector3d axis(angleAxis.axis());
        json["AngleAxis"]["Axis"] = jsonFromStdArrayDouble<3>({axis(0), axis(1), axis(2)});
    } else if (representation == RotationRepresentation::CardanXYZ) {
        json["CardanXYZ"] = rotation.getCardanXYZ();
    } else if (representation == RotationRepresentation::EulerZXZ) {
        json["EulerZXZ"] = rotation.getEulerZXZ();
    } else {
        throw std::logic_error("nlohmann::adl_serializer<Rotation>::to_json: "
            "OrientationRepresentation invalid.");
    }
}

}  // namespace nlohmann
