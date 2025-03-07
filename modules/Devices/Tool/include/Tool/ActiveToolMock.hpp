/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include <memory>

#include "Tool/IActiveTool.hpp"

namespace crf::devices::tool {

class ActiveToolMock : public IActiveTool {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));

    MOCK_METHOD(crf::expected<bool>, activate, (), (override));
    MOCK_METHOD(crf::expected<bool>, deactivate, (), (override));

    MOCK_METHOD(crf::expected<bool>, isActive, (), (override));

    MOCK_METHOD(std::shared_ptr<urdf::ModelInterface>, getURDF, (), (override));
};

}  // namespace crf::devices::tool
