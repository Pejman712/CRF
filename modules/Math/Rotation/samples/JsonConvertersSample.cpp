/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/JsonConverters.hpp"

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;
using crf::math::rotation::Rotation;
using crf::math::rotation::RotationRepresentation;

int main() {
    nlohmann::json outputJson;

    /**
     * CardanXYZ
    */
    CardanXYZ outputCardanXYZ;
    CardanXYZ cardanXYZ({2.17, 3.15, 0.7});
    nlohmann::json jsonCardanXYZ;
    jsonCardanXYZ["X"] = 2.17;
    jsonCardanXYZ["Y"] = 3.15;
    jsonCardanXYZ["Z"] = 0.7;
    /**
     * From json
    */
    outputCardanXYZ = jsonCardanXYZ.get<CardanXYZ>();
    /**
     * To json
    */
    outputJson = cardanXYZ;

    /**
     * EulerZXZ
    */
    EulerZXZ outputEulerZXZ;
    EulerZXZ eulerZXZ({2.17, 3.15, 0.7});
    nlohmann::json jsonEulerZXZ;
    jsonEulerZXZ["Z1"] = 2.17;
    jsonEulerZXZ["X"] = 3.15;
    jsonEulerZXZ["Z2"] = 0.7;
    /**
    * From json
    */
    outputEulerZXZ = jsonEulerZXZ.get<EulerZXZ>();
    /**
    * To json
    */
    outputJson = eulerZXZ;

    /**
     * Rotation
    */
    Rotation outputRotation;
    Rotation rotationQuaternion(Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0}));
    Rotation rotationMatrix(Eigen::Matrix3d(Eigen::Matrix3d::Identity()));
    Rotation rotationAngleAxis(Eigen::AngleAxisd(1.0, Eigen::Vector3d({1.0, 0.0, 0.0})));
    Rotation rotationCardanXYZ(CardanXYZ({1.0, 2.0, 3.0}));
    Rotation rotationEulerZXZ(EulerZXZ({1.0, 2.0, 3.0}));
    nlohmann::json jsonRotationQuaternion;
    jsonRotationQuaternion["Quaternion"]["W"] = 1.0;
    jsonRotationQuaternion["Quaternion"]["X"] = 0.0;
    jsonRotationQuaternion["Quaternion"]["Y"] = 0.0;
    jsonRotationQuaternion["Quaternion"]["Z"] = 0.0;
    nlohmann::json jsonRotationMatrix;
    jsonRotationMatrix["Matrix"]["Row1"] = {1.0, 0.0, 0.0};
    jsonRotationMatrix["Matrix"]["Row2"] = {0.0, 1.0, 0.0};
    jsonRotationMatrix["Matrix"]["Row3"] = {0.0, 0.0, 1.0};
    nlohmann::json jsonRotationAngleAxis;
    jsonRotationAngleAxis["AngleAxis"]["Angle"] = 0.0;
    jsonRotationAngleAxis["AngleAxis"]["axiz"] = {1.0, 0.0, 0.0};
    nlohmann::json jsonRotationCardanXYZ;
    jsonRotationCardanXYZ["CardanXYZ"]["X"] = 1.0;
    jsonRotationCardanXYZ["CardanXYZ"]["Y"] = 2.0;
    jsonRotationCardanXYZ["CardanXYZ"]["Z"] = 3.0;
    nlohmann::json jsonRotationEulerZXZ;
    jsonRotationEulerZXZ["EulerZXZ"]["Z1"] = 1.0;
    jsonRotationEulerZXZ["EulerZXZ"]["X"] = 2.0;
    jsonRotationEulerZXZ["EulerZXZ"]["Z2"] = 3.0;
    /**
     * From json
    */
    outputRotation = jsonRotationQuaternion.get<Rotation>();
    outputRotation = jsonRotationMatrix.get<Rotation>();
    outputRotation = jsonRotationAngleAxis.get<Rotation>();
    outputRotation = jsonRotationCardanXYZ.get<Rotation>();
    outputRotation = jsonRotationEulerZXZ.get<Rotation>();

    /**
     * To json
    */
    outputJson = rotationAngleAxis;
    outputJson = rotationQuaternion;
    outputJson = rotationAngleAxis;
    outputJson = rotationEulerZXZ;
    outputJson = rotationCardanXYZ;

    return 0;
}
