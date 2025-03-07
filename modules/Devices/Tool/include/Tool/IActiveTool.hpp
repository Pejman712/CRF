/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <memory>

#include "CommonInterfaces/IInitializable.hpp"
#include "Tool/IPassiveTool.hpp"
#include "crf/expected.hpp"

namespace crf::devices::tool {

/**
 * @ingroup group_tool
 * @brief
 */
class IActiveTool : public crf::utility::commoninterfaces::IInitializable,
                    public crf::devices::tool::IPassiveTool {
 public:
    virtual ~IActiveTool() = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief Method to activate the tool.
     *
     * @return crf::expected<bool>
     */
    virtual crf::expected<bool> activate() = 0;
    /**
     * @brief Method to deactivate the tool
     *
     * @return crf::expected<bool>
     */
    virtual crf::expected<bool> deactivate() = 0;
    /**
     * @brief Method to know the current status of the gripper and if it's active
     *
     * @return crf::expected<bool>
     */
    virtual crf::expected<bool> isActive() = 0;
};

}  // namespace crf::devices::tool
