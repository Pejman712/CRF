/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "CommunicationUtility/StdJsonConverters.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::communicationutility::doubleFromJson;
using crf::utility::communicationutility::jsonFromDouble;
using crf::utility::communicationutility::stdVectorDoubleFromJson;
using crf::utility::communicationutility::jsonFromStdVectorDouble;
using crf::utility::communicationutility::stdArrayDoubleFromJson;
using crf::utility::communicationutility::jsonFromStdArrayDouble;

class StdJsonConvertersShould : public ::testing::Test {
 protected:
    StdJsonConvertersShould() :
        double1_(15.27485943725416),
        double1Json_(),
        doubleInf1_(std::numeric_limits<double>::infinity()),
        doubleInf1Json_(),
        doubleMinusInf1_(-std::numeric_limits<double>::infinity()),
        doubleMinusInf1Json_(),
        doubleNaN1_(std::numeric_limits<double>::quiet_NaN()),
        doubleNaN1Json_(),
        vector1_({doubleInf1_, 15.27485943725416, doubleMinusInf1_, -24.15, doubleNaN1_, 24.8}),
        vector1Json_(),
        array3_1_({doubleInf1_, 15.27485943725416, doubleMinusInf1_}),
        array3_1json_(),
        array3_2_({doubleNaN1_, 15.27485943725416, doubleMinusInf1_}),
        array3_2json_(),
        array4_1_({doubleInf1_, 15.27485943725416, doubleMinusInf1_, doubleNaN1_}),
        array4_1json_(),
        array6_1_({doubleInf1_, doubleNaN1_, doubleMinusInf1_, -24.15, 15.3, 24.8}),
        array6_1json_(),
        outputArray3_(),
        outputArray4_(),
        outputArray6_() {
        double1Json_ = double1_;
        doubleInf1Json_ = "inf";
        doubleMinusInf1Json_ = "-inf";
        doubleNaN1Json_ = "NaN";
        vector1Json_ = {"inf", 15.27485943725416, "-inf", -24.15, "NaN", 24.8};
        array3_1json_ = {"inf", 15.27485943725416, "-inf"};
        array3_2json_ = {"NaN", 15.27485943725416, "-inf"};
        array4_1json_ = {"inf", 15.27485943725416, "-inf", "NaN"};
        array6_1json_ = {"inf", "NaN", "-inf", -24.15, 15.3, 24.8};
    }

    void SetUp() {
        outputArray3_ = {0.0, 0.0, 0.0};
        outputArray4_ = {0.0, 0.0, 0.0, 0.0};
        outputArray6_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    bool areEqualWithNaNs(std::vector<double> vector1, std::vector<double> vector2) {
        if (vector1.size() != vector2.size()) {
            return false;
        }
        for (std::size_t i = 0; i < vector1.size(); i++) {
            if (vector1[i] != vector2[i] && !(std::isnan(vector1[i]) && std::isnan(vector2[i]))) {
                return false;
            }
        }
        return true;
    }

    template <size_t size>
    bool areEqualWithNaNs(std::array<double, size> array1, std::array<double, size> array2) {
        for (std::size_t i = 0; i < size; i++) {
            if (!(array1[i] == array2[i] || (std::isnan(array1[i]) && std::isnan(array2[i])))) {
                return false;
            }
        }
        return true;
    }

    const double double1_;
    nlohmann::json double1Json_;
    const double doubleInf1_;
    nlohmann::json doubleInf1Json_;
    const double doubleMinusInf1_;
    nlohmann::json doubleMinusInf1Json_;
    const double doubleNaN1_;
    nlohmann::json doubleNaN1Json_;

    const std::vector<double> vector1_;
    nlohmann::json vector1Json_;

    const std::array<double, 3> array3_1_;
    nlohmann::json array3_1json_;
    const std::array<double, 3> array3_2_;
    nlohmann::json array3_2json_;
    const std::array<double, 4> array4_1_;
    nlohmann::json array4_1json_;
    const std::array<double, 6> array6_1_;
    nlohmann::json array6_1json_;

    std::array<double, 3> outputArray3_;
    std::array<double, 4> outputArray4_;
    std::array<double, 6> outputArray6_;
};

TEST_F(StdJsonConvertersShould, ReturnCorrectDoubleFromJson) {
    ASSERT_EQ(double1_, doubleFromJson(double1Json_));
    ASSERT_EQ(doubleInf1_, doubleFromJson(doubleInf1Json_));
    ASSERT_EQ(doubleMinusInf1_, doubleFromJson(doubleMinusInf1Json_));
}

TEST_F(StdJsonConvertersShould, ReturnCorrectJsonFromDouble) {
    ASSERT_EQ(double1Json_, jsonFromDouble(double1_));
    ASSERT_EQ(doubleInf1Json_, jsonFromDouble(doubleInf1_));
    ASSERT_EQ(doubleMinusInf1Json_, jsonFromDouble(doubleMinusInf1_));
}

TEST_F(StdJsonConvertersShould, ReturnCorrectStdVectorDoubleFromJson) {
    ASSERT_TRUE(areEqualWithNaNs(vector1_, stdVectorDoubleFromJson(vector1Json_)));
}

TEST_F(StdJsonConvertersShould, ReturnCorrectJsonFromStdVectorDouble) {
    ASSERT_EQ(vector1Json_, jsonFromStdVectorDouble(vector1_));
}

TEST_F(StdJsonConvertersShould, ReturnCorrectStdArrayDoubleFromJson) {
    ASSERT_NO_THROW(outputArray3_ = stdArrayDoubleFromJson<3>(array3_1json_));
    ASSERT_EQ(array3_1_, stdArrayDoubleFromJson<3>(array3_1json_));
    ASSERT_NO_THROW(outputArray3_ = stdArrayDoubleFromJson<3>(array3_2json_));
    ASSERT_TRUE(areEqualWithNaNs(array3_2_, stdArrayDoubleFromJson<3>(array3_2json_)));
    ASSERT_NO_THROW(outputArray4_ = stdArrayDoubleFromJson<4>(array4_1json_));
    ASSERT_TRUE(areEqualWithNaNs(array4_1_, stdArrayDoubleFromJson<4>(array4_1json_)));
    ASSERT_NO_THROW(outputArray6_ = stdArrayDoubleFromJson<6>(array6_1json_));
    ASSERT_TRUE(areEqualWithNaNs(array6_1_, stdArrayDoubleFromJson<6>(array6_1json_)));
}

TEST_F(StdJsonConvertersShould, ThrowOnIncorrectJsonSizeInStdArrayDoubleFromJson) {
    ASSERT_THROW(outputArray3_ = stdArrayDoubleFromJson<3>(array4_1json_), std::invalid_argument);
    ASSERT_THROW(outputArray4_ = stdArrayDoubleFromJson<4>(array6_1json_), std::invalid_argument);
    ASSERT_THROW(outputArray6_ = stdArrayDoubleFromJson<6>(array3_1json_), std::invalid_argument);
}

TEST_F(StdJsonConvertersShould, ReturnCorrectJsonFromStdArrayDouble) {
    ASSERT_EQ(array3_1json_, jsonFromStdArrayDouble<3>(array3_1_));
    ASSERT_EQ(array3_2json_, jsonFromStdArrayDouble<3>(array3_2_));
    ASSERT_EQ(array4_1json_, jsonFromStdArrayDouble<4>(array4_1_));
    ASSERT_EQ(array6_1json_, jsonFromStdArrayDouble<6>(array6_1_));
}
