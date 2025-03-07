/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <optional>

#include <nlohmann/json.hpp>

/**
 * @brief nlohmann json is a library intended for c++11, still working in c++17.
 *  They don't implement std::optional for now, this breaks some communications
 *  since we can't use json.get<T> when T is std::optional<U> (jplayang).
 *
 *  From github nlohmann json issues
 *     - https://github.com/nlohmann/json/pull/2117
 *     - https://github.com/nlohmann/json/issues/1749 <- From here
 */

namespace nlohmann {

template <class T>
inline void to_json(nlohmann::json& j, const std::optional<T>& v) {  // NOLINT
    if (v.has_value()) {
        j = *v;
    } else {
        j = nullptr;
    }
}

template <class T>
inline void from_json(const nlohmann::json& j, std::optional<T>& v) {  // NOLINT
    if (j.is_null()) {
        v = std::nullopt;
    } else {
        v = j.get<T>();
    }
}

}  // namespace nlohmann
