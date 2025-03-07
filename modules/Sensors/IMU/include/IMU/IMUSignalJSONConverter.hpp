/*
 * © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Sebastien Collomb BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */
#pragma once

#include <vector>
#include <string>

#include <nlohmann/json.hpp>

#include "IMU/IMUSignals.hpp"
#include "CommunicationUtility/ExpectedJSONConverter.hpp"

namespace nlohmann {

inline void to_json(nlohmann::json& j, const crf::sensors::imu::IMUSignals& sig) {  // NOLINT
    j["Position"] = sig.position;
    j["Quaternion"] = sig.quaternion;
    j["EulerZYX"] = sig.eulerZYX;
    j["LinearVelocity"] = sig.linearVelocity;
    j["AngularVelocity"] = sig.angularVelocity;
    j["LinearAcceleration"] = sig.linearAcceleration;
    j["AngularAcceleration"] = sig.angularAcceleration;
    j["MagneticField"] = sig.magneticField;
}

inline void from_json(const nlohmann::json& j, crf::sensors::imu::IMUSignals& sig) {  // NOLINT
    sig.position = j["Position"].get<crf::expected<std::array<double, 3>>>();
    sig.quaternion = j["Quaternion"].get<crf::expected<std::array<double, 4>>>();
    sig.eulerZYX = j["EulerZYX"].get<crf::expected<std::array<double, 3>>>();
    sig.linearVelocity = j["LinearVelocity"].get<crf::expected<std::array<double, 3>>>();
    sig.angularVelocity = j["AngularVelocity"].get<crf::expected<std::array<double, 3>>>();
    sig.linearAcceleration = j["LinearAcceleration"].get<crf::expected<std::array<double, 3>>>();
    sig.angularAcceleration = j["AngularAcceleration"].get<crf::expected<std::array<double, 3>>>();
    sig.magneticField = j["MagneticField"].get<crf::expected<std::array<double, 3>>>();
}

}  // namespace nlohmann
