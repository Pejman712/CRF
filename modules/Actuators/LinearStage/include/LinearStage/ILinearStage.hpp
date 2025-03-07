/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <boost/optional.hpp>
#include <memory>

#include "CommonInterfaces/IInitializable.hpp"
#include "LinearStage/LinearStageConfiguration.hpp"

namespace crf::actuators::linearstage {

/**
 * @ingroup group_linear_stage
 * @brief
 */
class ILinearStage : public utility::commoninterfaces::IInitializable {
 public:
    ~ILinearStage() override = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    virtual bool setTargetPosition(float position) = 0;
    virtual bool setTargetVelocity(float velocity) = 0;

    virtual boost::optional<float> getActualPosition() = 0;
    virtual boost::optional<float> getActualVelocity() = 0;

    virtual std::shared_ptr<LinearStageConfiguration> getConfiguration() = 0;
};

}  // namespace crf::actuators::linearstage
