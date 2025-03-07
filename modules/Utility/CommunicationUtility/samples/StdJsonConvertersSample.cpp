/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "JsonConverters/StdJsonConverters.hpp"

using crf::utility::communicationutility::doubleFromJson;
using crf::utility::communicationutility::jsonFromDouble;
using crf::utility::communicationutility::stdVectorDoubleFromJson;
using crf::utility::communicationutility::jsonFromStdVectorDouble;
using crf::utility::communicationutility::stdArrayDoubleFromJson;
using crf::utility::communicationutility::jsonFromStdArrayDouble;

int main() {
    nlohmann::json outputJson;

    /**
     * Double
    */
    double outputDouble;
    double double1_ = 15.27485943725416;
    double doubleInf1_(std::numeric_limits<double>::infinity());
    double doubleMinusInf1_(-std::numeric_limits<double>::infinity());
    nlohmann::json jsonDouble1_ = 15.27485943725416;
    nlohmann::json jsonDoubleInf1_ = "inf";
    nlohmann::json jsonDoubleMinusInf1_ = "-inf";
    /**
     * From json
    */
    outputDouble = doubleFromJson(jsonDouble1_);
    outputDouble = doubleFromJson(jsonDoubleInf1_);
    outputDouble = doubleFromJson(jsonDoubleMinusInf1_);
    /**
     * To json
    */
    outputJson = jsonFromDouble(double1_);
    outputJson = jsonFromDouble(doubleInf1_);
    outputJson = jsonFromDouble(doubleMinusInf1_);

    /**
     * Std vector
    */
    std::vector<double> outputVector;
    std::vector<double> vector1_(
        {doubleInf1_, 15.27485943725416, doubleMinusInf1_, -24.15, 15.3, 24.8});
    nlohmann::json jsonVector1_;
    jsonVector1_ = {"inf", 15.27485943725416, "-inf", -24.15, 15.3, 24.8};
    /**
     * From json
    */
    outputVector = stdVectorDoubleFromJson(jsonVector1_);
    /**
     * To json
    */
    outputJson = jsonFromStdVectorDouble(vector1_);

    /**
     * Std array 3, 4 or 6
    */
    std::array<double, 3> outputArray3_;
    std::array<double, 4> outputArray4_;
    std::array<double, 6> outputArray6_;
    std::array<double, 3> array3_1_({doubleInf1_, 15.27485943725416, doubleMinusInf1_});
    std::array<double, 4> array4_1_({doubleInf1_, 15.27485943725416, doubleMinusInf1_, -24.15});
    std::array<double, 6> array6_1_(
        {doubleInf1_, 15.27485943725416, doubleMinusInf1_, -24.15, 15.3, 24.8});
    nlohmann::json jsonArray3_1_;
    nlohmann::json jsonArray4_1_;
    nlohmann::json jsonArray6_1_;
    jsonArray3_1_ = {"inf", 15.27485943725416, "-inf"};
    jsonArray4_1_ = {"inf", 15.27485943725416, "-inf", -24.15};
    jsonArray6_1_ = {"inf", 15.27485943725416, "-inf", -24.15, 15.3, 24.8};
    /**
     * From json
    */
    outputArray3_ = stdArrayDoubleFromJson<3>(jsonArray3_1_);
    outputArray4_ = stdArrayDoubleFromJson<4>(jsonArray4_1_);
    outputArray6_ = stdArrayDoubleFromJson<6>(jsonArray6_1_);
    /**
     * To json
    */
    outputArray3_ = jsonFromStdArrayDouble<3>(array3_1_);
    outputArray4_ = jsonFromStdArrayDouble<4>(array4_1_);
    outputArray6_ = jsonFromStdArrayDouble<6>(array6_1_);

    return 0;
}
