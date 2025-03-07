/* © Copyright CERN 2024.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#pragma once

#include <array>
#include <limits>
#include <nlohmann/json.hpp>
#include <vector>

namespace crf::utility::communicationutility {
/**
 * @ingroup group_communication_utility_std_json_converters
 * @brief Function to parse a json vector into an std array
 * with a check for infinite values
 */
double doubleFromJson(const nlohmann::json& json);

/**
 * @ingroup group_communication_utility_std_json_converters
 * @brief Function to transform an std array into a json vector
 * with that represents infinite values
 */
nlohmann::json jsonFromDouble(const double& number);

/**
 * @ingroup group_communication_utility_std_json_converters
 * @brief Function to parse a json vector into an std vector
 * with a check for infinite values
 */
std::vector<double> stdVectorDoubleFromJson(const nlohmann::json& json);

/**
 * @ingroup group_communication_utility_std_json_converters
 * @brief Function to transform an std vector into a json vector
 * with that represents infinite values
 */
nlohmann::json jsonFromStdVectorDouble(const std::vector<double>& vector);

/**
 * @ingroup group_communication_utility_std_json_converters
 * @brief Function to parse a json vector into an std array
 * with a check for infinite values.
 * @throws Size -- If the size of the parameter 'json' is different than template
 * parameter 'size', function throws a 'std::invalid_argument' excepton.
 */
template <size_t size>
std::array<double, size> stdArrayDoubleFromJson(const nlohmann::json& json);

/**
 * @ingroup group_communication_utility_std_json_converters
 * @brief Function to transform an std array into a json vector
 * with that represents infinite values
 */
template <size_t size>
nlohmann::json jsonFromStdArrayDouble(const std::array<double, size>& array);

}  // namespace crf::utility::communicationutility
