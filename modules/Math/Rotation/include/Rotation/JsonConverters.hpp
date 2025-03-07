/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <exception>

#include "CommunicationUtility/StdJsonConverters.hpp"
#include "Rotation/RotationClass.hpp"
#include <nlohmann/json.hpp>
#include <string>

using crf::utility::communicationutility::stdArrayDoubleFromJson;
using crf::utility::communicationutility::jsonFromStdArrayDouble;
using crf::utility::communicationutility::doubleFromJson;
using crf::utility::communicationutility::jsonFromDouble;

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;
using crf::math::rotation::Rotation;
using crf::math::rotation::RotationRepresentation;

using crf::math::rotation::quaternionFromArray;
using crf::math::rotation::arrayFromQuaternion;
using crf::math::rotation::matrixFromArray;
using crf::math::rotation::arrayFromMatrix;
using crf::math::rotation::angleAxisFromArray;
using crf::math::rotation::arrayFromAngleAxis;

namespace nlohmann {

template <>
struct adl_serializer<CardanXYZ> {
    static CardanXYZ from_json(const json& json);
    static void to_json(json& json, const CardanXYZ& cardanXYZ);  // NOLINT
};

template <>
struct adl_serializer<EulerZXZ> {
    static EulerZXZ from_json(const json& json);
    static void to_json(json& json, const EulerZXZ& eulerZXZ);  // NOLINT
};

template <>
struct adl_serializer<Rotation> {
    static Rotation from_json(const json& json);
    static void to_json(json& json, const Rotation& rotation);  // NOLINT
};

}  // namespace nlohmann
