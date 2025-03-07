/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include <vector>

#include "GeometricMethods/DeBoor/DeBoor.hpp"

class DeBoorShould: public ::testing::Test {
 protected:
    DeBoorShould(): logger_("DeBoorShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~DeBoorShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::math::geometricmethods::DeBoor> sut_;
};

TEST_F(DeBoorShould, returnVariablesAreNotCorrectlyInitialized) {
    std::vector<double> knotsEmpty{};
    std::vector<double> knots = {0, 0, 0, 0, 0, 0.0625, 0.125, 0.1875, 0.25, 0.3125, 0.375, 0.4375,
        0.5, 0.5625, 0.625, 0.6875, 0.75, 0.8125, 0.875, 0.9375, 1, 1, 1, 1, 1};
    std::vector<double> controlPointsEmpty = {};
    std::vector<double> controlPoints = {0, 0, 0, 0, 0.05650608, 0.236911941, 0.560672572,
        0.972236973, 1.357214323, 1.590212873, 1.590212873, 1.357214323, 0.972236973, 0.560672572,
        0.236911941, 0.05650608, 0, 0, 0, 0};
    double degree = 4;

    ASSERT_THROW(
        sut_.reset(new crf::math::geometricmethods::DeBoor(degree, knots, controlPointsEmpty)),
        std::runtime_error);

    ASSERT_THROW(
        sut_.reset(new crf::math::geometricmethods::DeBoor(degree, knotsEmpty, controlPoints)),
        std::runtime_error);

    ASSERT_THROW(
        sut_.reset(new crf::math::geometricmethods::DeBoor(0, knots, controlPoints)),
        std::runtime_error);
}

TEST_F(DeBoorShould, returnCorrectResults) {
    std::vector<double> knots = {0, 0, 0, 0, 0, 0.0625, 0.125, 0.1875, 0.25, 0.3125, 0.375, 0.4375,
        0.5, 0.5625, 0.625, 0.6875, 0.75, 0.8125, 0.875, 0.9375, 1, 1, 1, 1, 1};
    std::vector<double> controlPoints = {0, 0, 0, 0, 0.05650608, 0.236911941, 0.560672572,
        0.972236973, 1.357214323, 1.590212873, 1.590212873, 1.357214323, 0.972236973, 0.560672572,
        0.236911941, 0.05650608, 0, 0, 0, 0};
    double degree = 4;

    ASSERT_NO_THROW(sut_.reset(new crf::math::geometricmethods::DeBoor(degree, knots ,
        controlPoints)));

    std::optional<double> out = sut_->getRange();
    ASSERT_TRUE(out);
    ASSERT_EQ(out.value(), 1.0);
    std::optional<double> out0 = sut_->evaluate(0, 0);
    ASSERT_TRUE(out0);
    ASSERT_EQ(out0.value(), 0.0);
    std::optional<double> out1 = sut_->evaluate(0.2, 0);
    ASSERT_TRUE(out1);
    ASSERT_NEAR(out1.value(), 0.197268, 0.000001);
    std::optional<double> out2 = sut_->evaluate(0.4, 0);
    ASSERT_TRUE(out2);
    ASSERT_NEAR(out2.value(), 1.29448, 0.00001);
    std::optional<double> out3 = sut_->evaluate(0.6, 0);
    ASSERT_TRUE(out3);
    ASSERT_NEAR(out3.value(), 1.29448, 0.00001);
    std::optional<double> out4 = sut_->evaluate(0.8, 0);
    ASSERT_TRUE(out4);
    ASSERT_NEAR(out4.value(), 0.197268, 0.000001);
    std::optional<double> out5 = sut_->evaluate(1.0, 0);
    ASSERT_TRUE(out5);
    ASSERT_EQ(out5.value(), 0.0);
}

TEST_F(DeBoorShould, returnEmptyVectorIfInputEvalPointIsNotPositive) {
    std::vector<double> knots = {0, 0, 0, 0, 0, 0.0625, 0.125, 0.1875, 0.25, 0.3125, 0.375, 0.4375,
        0.5, 0.5625, 0.625, 0.6875, 0.75, 0.8125, 0.875, 0.9375, 1, 1, 1, 1, 1};
    std::vector<double> controlPoints = {0, 0, 0, 0, 0.05650608, 0.236911941, 0.560672572,
        0.972236973, 1.357214323, 1.590212873, 1.590212873, 1.357214323, 0.972236973, 0.560672572,
        0.236911941, 0.05650608, 0, 0, 0, 0};
    double degree = 4;

    ASSERT_NO_THROW(sut_.reset(new crf::math::geometricmethods::DeBoor(degree, knots,
        controlPoints)));

    ASSERT_EQ(sut_->evaluate(-0.5, 0), std::nullopt);
}

TEST_F(DeBoorShould, returnMaximunValueIfInputEvalPointIsNotWithinTheRange) {
    std::vector<double> knots = {0, 0, 0, 0, 0, 0.0625, 0.125, 0.1875, 0.25, 0.3125, 0.375, 0.4375,
        0.5, 0.5625, 0.625, 0.6875, 0.75, 0.8125, 0.875, 0.9375, 1, 1, 1, 1, 1};
    std::vector<double> controlPoints = {0, 0, 0, 0, 0.05650608, 0.236911941, 0.560672572,
        0.972236973, 1.357214323, 1.590212873, 1.590212873, 1.357214323, 0.972236973, 0.560672572,
        0.236911941, 0.05650608, 0, 0, 0, 0};
    double degree = 4;

    ASSERT_NO_THROW(sut_.reset(new crf::math::geometricmethods::DeBoor(degree, knots,
        controlPoints)));

    std::optional<double> out = sut_->evaluate(1.5, 0);
    ASSERT_TRUE(out);
    ASSERT_EQ(out.value(), 0.0);
}

TEST_F(DeBoorShould, returnEmptyVectorIfInputDerIsNot0) {
    std::vector<double> knots = {0, 0, 0, 0, 0, 0.0625, 0.125, 0.1875, 0.25, 0.3125, 0.375, 0.4375,
        0.5, 0.5625, 0.625, 0.6875, 0.75, 0.8125, 0.875, 0.9375, 1, 1, 1, 1, 1};
    std::vector<double> controlPoints = {0, 0, 0, 0, 0.05650608, 0.236911941, 0.560672572,
        0.972236973, 1.357214323, 1.590212873, 1.590212873, 1.357214323, 0.972236973, 0.560672572,
        0.236911941, 0.05650608, 0, 0, 0, 0};
    double degree = 4;

    ASSERT_NO_THROW(sut_.reset(new crf::math::geometricmethods::DeBoor(degree, knots,
        controlPoints)));

    ASSERT_EQ(sut_->evaluate(0.2, 1), std::nullopt);
}
