/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <vector>

#include <nlohmann/json.hpp>

#include "Cameras/CameraProfile.hpp"

namespace nlohmann {

inline void to_json(nlohmann::json& json, const crf::sensors::cameras::Profile& profile) {  // NOLINT
    json["resolution"]["width"] = profile.resolution.width;
    json["resolution"]["height"] = profile.resolution.height;
    json["framerate"] = profile.framerates;
}

inline void from_json(const nlohmann::json& json, crf::sensors::cameras::Profile& profile) {  // NOLINT
    profile.framerates = json.at("framerate").get<std::vector<uint64_t>>();
    profile.framerate = profile.framerates[0];
    profile.resolution = cv::Size(
        json.at("resolution").at("width").get<int>(),
        json.at("resolution").at("height").get<int>());
}

}  // namespace nlohmann
