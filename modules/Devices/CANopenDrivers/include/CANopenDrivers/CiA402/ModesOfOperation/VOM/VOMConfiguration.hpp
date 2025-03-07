/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>
#include <string>
#include <optional>
#include <map>

#include <nlohmann/json.hpp>

#include "CANopenDrivers/CiA402/CiA402Definitions.hpp"
#include "CANopenDrivers/CiA402/ModesOfOperation/VOM/VOMDefinitions.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_vom
 * @brief Class VOMConfiguration is a class for configuring the registers of the
 * velocity mode. The point of the class is to get the values of the
 * configuration registers of the mode from the json file so that they can be later used in the
 * VOM class for writing values in the registers.
 *
 */
class VOMConfiguration {
 public:
    explicit VOMConfiguration(const nlohmann::json& json);
    explicit VOMConfiguration(const std::string& str) = delete;
    ~VOMConfiguration() = default;

    /**
     * @brief Function that returns the value of the velocity min/max register.
     * The velocity min/max register has 2 subindexes corresponding to min and max value
     * of the velocity for this mode.
    */
    std::optional<VelocityMinMaxAmountVOM> getVelocityMinMaxAmount() const;

    /**
     * @brief Function that returns the value of the velocity quickstop register.
     * The velocity quickstop is calculated as: quickstop = deltaSpeed/deltaTime.
    */
    std::optional<VelocityQuickStopVOM> getVelocityQuickStop() const;

    /**
     * @brief Function that returns the value of the velocity set-point factor register.
     * The value of this register is calculated as: factor = numerator/denominator.
    */
    std::optional<VelocitySetPointFactorVOM> getVelocitySetPointFactor() const;

    /**
     * @brief Function that returns the value of the dimension factor register.
     * This register is used to configure the units of the velocity as
     * unit = numerator/denominator. The default unit for velocity is rpm.
    */
    std::optional<DimensionFactorVOM> getDimensionFactor() const;

 private:
    void parse(const nlohmann::json& json);
    void parse(const std::string&) = delete;

    std::optional<VelocitySetPointFactorVOM> velocitySetPointFactor_;
    std::optional<DimensionFactorVOM> dimensionFactor_;
    std::optional<VelocityMinMaxAmountVOM> velocityMinMaxAmount_;
    std::optional<VelocityQuickStopVOM> velocityQuickStop_;
};

}  // namespace crf::devices::canopendrivers
