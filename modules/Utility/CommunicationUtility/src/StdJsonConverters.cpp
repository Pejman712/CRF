/* © Copyright CERN 2024.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include "CommunicationUtility/StdJsonConverters.hpp"

namespace crf::utility::communicationutility {

double doubleFromJson(const nlohmann::json& json) {
    double number;
    if (json == "inf") {
        number = std::numeric_limits<double>::infinity();
    } else if (json == "-inf") {
        number = -std::numeric_limits<double>::infinity();
    } else if (json == "NaN") {
        number = std::numeric_limits<double>::quiet_NaN();
    } else {
        number = json.get<double>();
    }
    return number;
}

nlohmann::json jsonFromDouble(const double& number) {
    nlohmann::json json;
    if (number == std::numeric_limits<double>::infinity()) {
        json = "inf";
    } else if (number == -std::numeric_limits<double>::infinity()) {
        json = "-inf";
    } else if (std::isnan(number)) {
        json = "NaN";
    } else {
        json = number;
    }
    return json;
}

std::vector<double> stdVectorDoubleFromJson(const nlohmann::json& json) {
    std::vector<double> vector(json.size());
    for (size_t i = 0; i < json.size(); i++) {
        vector[i] = doubleFromJson(json[i]);
    }
    return vector;
}

nlohmann::json jsonFromStdVectorDouble(const std::vector<double>& vector) {
    nlohmann::json json = std::vector<double>(vector.size());
    for (size_t i = 0; i < vector.size(); i++) {
        json[i] = jsonFromDouble(vector[i]);
    }
    return json;
}

template <size_t size>
std::array<double, size> stdArrayDoubleFromJson(const nlohmann::json& json) {
    if (json.size() != size) {
        throw std::invalid_argument(
            "crf::utility::types::stdArrayDoubleFromJson<" + std::to_string(size) +
            ">: json was not of the size " + std::to_string(size) + ", it was instead of the size "
            + std::to_string(json.size()) + ".");
    }
    std::array<double, size> array;
    for (size_t i = 0; i < size; i++) {
        array[i] = doubleFromJson(json[i]);
    }
    return array;
}

template std::array<double, 3> stdArrayDoubleFromJson(const nlohmann::json& json);
template std::array<double, 4> stdArrayDoubleFromJson(const nlohmann::json& json);
template std::array<double, 6> stdArrayDoubleFromJson(const nlohmann::json& json);

template <size_t size>
nlohmann::json jsonFromStdArrayDouble(const std::array<double, size>& array) {
    nlohmann::json json = std::vector<double>(size);
    for (size_t i = 0; i < size; i++) {
        json[i] = jsonFromDouble(array[i]);
    }
    return json;
}

template nlohmann::json jsonFromStdArrayDouble(const std::array<double, 3>& array);
template nlohmann::json jsonFromStdArrayDouble(const std::array<double, 4>& array);
template nlohmann::json jsonFromStdArrayDouble(const std::array<double, 6>& array);

}  // namespace crf::utility::communicationutility
