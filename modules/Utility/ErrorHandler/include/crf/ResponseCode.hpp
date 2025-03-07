/*
 * © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/
#pragma once

#include <iostream>

#include "crf/ResponseDefinitions.hpp"

namespace crf {

/**
 * @ingroup group_error_handler
 * @brief Class response used to give more information about the code attached to the payload
 */
class ResponseCode {
 public:
    ResponseCode():
        code_(Code::Empty) {
    }
    ~ResponseCode() {}
    explicit ResponseCode(crf::Code code) noexcept:
        code_(code),
        detail_(0) {}

    explicit ResponseCode(crf::Code code, uint64_t detail) noexcept:
        code_(code),
        detail_(detail) {}
    ResponseCode& operator=(crf::Code code) noexcept {
        code_ = code;
        return *this;
    }

    constexpr crf::Code code() const noexcept {
        return code_;
    }
    constexpr uint64_t detail() const {
        return detail_;
    }
    constexpr void detail(const int64_t& detail) noexcept {
        detail_ = detail;
    }

    // Operators
    constexpr bool operator==(const ResponseCode& other) const noexcept {
        return code_ == other.code_ && detail_ == other.detail_;
    }
    constexpr bool operator==(const Code& code) const noexcept {
        return code_ == code;
    }
    constexpr bool operator!=(const ResponseCode& other) const noexcept {
        return !operator==(other);
    }
    constexpr bool operator!=(const Code& code) const noexcept {
        return !operator==(code);
    }
    friend std::ostream& operator<<(std::ostream& os, const ResponseCode& er) {
        os << std::to_string(static_cast<int>(er.code_)) << " - " << er.detail_;
        return os;
    }

 private:
    Code code_;
    int64_t detail_;
};

}  // namespace crf
