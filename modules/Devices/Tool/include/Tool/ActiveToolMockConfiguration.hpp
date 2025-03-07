/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN EN/SMM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include <memory>

#include "Tool/ActiveToolMock.hpp"

using testing::Invoke;
using testing::_;

namespace crf::devices::tool {

class ActiveToolMockConfiguration : public ActiveToolMock {
 public:
    ActiveToolMockConfiguration():
        initialized_(false) {
    }

    ~ActiveToolMockConfiguration() override {}

    void configureMock() {
        ON_CALL(*this, initialize()).WillByDefault(Invoke(
            [this] {
                if (initialized_) return false;
                initialized_ = true;
                return true;
            }));
        ON_CALL(*this, deinitialize()).WillByDefault(Invoke(
            [this] {
                if (!initialized_) return false;
                initialized_ = false;
                return true;
            }));

        ON_CALL(*this, activate()).WillByDefault(Invoke(
            [this] () -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                return true;
            }));
        ON_CALL(*this, deactivate()).WillByDefault(Invoke(
            [this] () -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                return true;
            }));
    }

 private:
    std::atomic<bool> initialized_;
};

}  // namespace crf::devices::tool
