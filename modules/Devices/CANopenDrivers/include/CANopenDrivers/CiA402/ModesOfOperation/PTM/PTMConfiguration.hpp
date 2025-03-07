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
#include "CANopenDrivers/CiA402/ModesOfOperation/PTM/PTMDefinitions.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_ptm
 * @brief Class PTMConfiguration is a class for configuring the registers of the
 * profile torque mode. The point of the class is to get the values of the
 * configuration registers of the mode from the json file so that they can be later used in the
 * PTM class for writing values in the registers.
 *
*/
class PTMConfiguration {
 public:
    explicit PTMConfiguration(const nlohmann::json& json);
    explicit PTMConfiguration(const std::string& str) = delete;
    ~PTMConfiguration() = default;

    /**
     * @brief: Function that returns the value of the positive torque limit.
     * @param: none
     * @return: value of uint16_t
     */
    std::optional<uint16_t> getPositiveTorqueLimitValue() const;

    /**
     * @brief: Function that returns the value of the negative torque limit.
     * @param: none
     * @return: value of uint16_t
     */
    std::optional<uint16_t> getNegativeTorqueLimitValue() const;

    /**
     * @brief: Function that returns the value of the torque slope.
     * @param: none
     * @return: value of uint32_t
     */
    std::optional<uint32_t> getTorqueSlope() const;

    /**
     * @brief: Function that returns the value of the torque profile type.
     * @param: none
     * @return: value of int16_t
     */
    std::optional<TorqueProfileTypePTM> getTorqueProfileType() const;

 private:
    void parse(const nlohmann::json& json);
    void parse(const std::string&) = delete;

    std::optional<uint16_t> maxTorque_;
    std::optional<uint16_t> positiveTorqueLimitValue_;
    std::optional<uint16_t> negativeTorqueLimitValue_;
    std::optional<uint16_t> maxCurrent_;
    std::optional<uint32_t> motorRatedCurrent_;
    std::optional<uint32_t> motorRatedTorque_;
    std::optional<uint32_t> torqueSlope_;
    std::optional<TorqueProfileTypePTM> torqueProfileType_;

    const std::map<std::string, TorqueProfileTypePTM> profileMap_ {
        {"Linear", TorqueProfileTypePTM::Linear},
        {"Sin^2", TorqueProfileTypePTM::Sin2}
    };
};

}  // namespace crf::devices::canopendrivers
