/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "crf/expected.hpp"

#include "CommonInterfaces/IInitializable.hpp"

namespace crf::actuators::linearactuator {

/**
 * @ingroup group_linear_stage
 * @brief
 */
class ILinearActuator : public utility::commoninterfaces::IInitializable {
 public:
    ~ILinearActuator() override = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief Set the Position object
     *
     * @param position
     * @return crf::expected<bool>
     */
    virtual crf::expected<bool> setPosition(const double& position) = 0;
    /**
     * @brief Set the Velocity object
     *
     * @param velocity
     * @return crf::expected<bool>
     */
    virtual crf::expected<bool> setVelocity(const double& velocity) = 0;
    /**
     * @brief Get the Position object
     *
     * @return crf::expected<double>
     */
    virtual crf::expected<double> getPosition() const = 0;
    /**
     * @brief Get the Velocity object
     *
     * @return crf::expected<double>
     */
    virtual crf::expected<double> getVelocity() const = 0;
};

}  // namespace crf::actuators::linearactuator
