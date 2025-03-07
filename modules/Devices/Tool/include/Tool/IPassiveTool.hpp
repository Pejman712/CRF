/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

//  Forward declaration of an object of the URDF external library.
namespace urdf {
class ModelInterface;
}  // namespace urdf

namespace crf::devices::tool {

/**
 * @ingroup group_tool
 * @brief
 */
class IPassiveTool {
 public:
    virtual ~IPassiveTool() = default;

    /**
     * @brief Method to return a pointer to the URDF file of the tool
     *
     * @return std::shared_ptr<urdf::ModelInterface>, URDF of the tool
     */
    virtual std::shared_ptr<urdf::ModelInterface> getURDF() = 0;
};

}  // namespace crf::devices::tool
