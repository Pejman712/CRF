/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "Tool/IPassiveTool.hpp"

namespace crf::devices::tool {

class PassiveToolMock : public IPassiveTool {
 public:
    MOCK_METHOD(std::shared_ptr<urdf::ModelInterface>, getURDF, (), override);
};

}  // namespace crf::devices::tool
