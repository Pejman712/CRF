/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>

#include <nlohmann/json.hpp>
#include "CommunicationUtility/ExpectedJSONConverter.hpp"

#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "Tool/IActiveTool.hpp"

namespace crf::devices::tool {

/**
 * @ingroup group_active_tool
 * @brief
 */
class ActiveToolManager: public crf::utility::devicemanager::DeviceManagerWithPriorityAccess {  // NOLINT
 public:
    ActiveToolManager() = delete;
    ActiveToolManager(std::shared_ptr<crf::devices::tool::IActiveTool> activetool,  // NOLINT
        const std::chrono::milliseconds& inizializationTimeout  = std::chrono::seconds(60),
        const std::chrono::milliseconds& controlAccessTimeout  = std::chrono::seconds(10));
    ActiveToolManager(const ActiveToolManager& other) = delete;
    ActiveToolManager(ActiveToolManager&& other) = delete;
    ~ActiveToolManager();

    crf::expected<bool> activate(const uint32_t& priority);
    crf::expected<bool> deactivate(const uint32_t& priority);

    nlohmann::json getStatus();

 private:
    std::shared_ptr<crf::devices::tool::IActiveTool> activetool_;
};

}  // namespace crf::devices::tool
