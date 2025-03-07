/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <crf/expected.hpp>
#include <nlohmann/json.hpp>

namespace nlohmann {

template <class T>
inline void to_json(nlohmann::json& j, const crf::expected<T>& v) {  // NOLINT
    if (v) {
        j["value"] = v.value();
    }
    j["code"] = v.get_response().code();
    j["detail"] = v.get_response().detail();
}

template <class T>
inline void from_json(const nlohmann::json& j, crf::expected<T>& v) {  // NOLINT
    if (j.contains("value")) {
        v = j["value"].get<T>();
    }
    v.attach(j["code"].get<crf::Code>(), j["detail"].get<int64_t>());
}

inline void to_json(nlohmann::json& j, const crf::ResponseCode& v) {  // NOLINT
    j["code"] = v.code();
    j["detail"] = v.detail();
}

inline void from_json(const nlohmann::json& j, crf::ResponseCode& v) {  // NOLINT
    v = crf::ResponseCode(j["code"].get<crf::Code>(), j["detail"].get<int64_t>());
}


}  // namespace nlohmann
