/* © Copyright CERN 2019.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "DigitalFilter/IIRFilter.hpp"
#include "crf/expected.hpp"

using crf::math::digitalfilter::DF2IIRFilter;
using crf::math::digitalfilter::direct2FormFilter;

class DigitalFilterShould : public ::testing::Test {
 protected:
    DigitalFilterShould() :
        logger_("DigitalFilterShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~DigitalFilterShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
};

TEST(DigitalFilterShould, failIfWrongSizeInput) {
    std::vector<double> a = {1, 1, 1, 1, 1};
    std::vector<double> b = {1, 1, 1, 1};
    double x = 1;

    // a.size() must be equal b.size()
    EXPECT_THROW(direct2FormFilter(x, a, b), std::invalid_argument);

    a = {1, 1, 1};
    EXPECT_THROW(direct2FormFilter(x, a, b), std::invalid_argument);

    // u.size() must equal to b.size() - 1
    b = {1, 1, 1};
    std::vector<double> u = {1, 1, 1, 1};
    EXPECT_THROW(direct2FormFilter(x, a, b, u), std::invalid_argument);

    u = {1, 1, 1};
    EXPECT_THROW(direct2FormFilter(x, a, b, u), std::invalid_argument);
}

TEST(DigitalFilterShould, failIfCoefficientA0IsNotOne) {
    std::vector<double> a = {2, 1, 1};
    std::vector<double> b = {1, 1, 1};
    double x = 1;
    EXPECT_THROW(direct2FormFilter(x, a, b), std::invalid_argument);
}

TEST(DigitalFilterShould, failIfEmptyCoefficients) {
    std::vector<double> a = {};
    std::vector<double> b = {1, 1, 1};
    double x = 1;

    // a empty
    EXPECT_THROW(direct2FormFilter(x, a, b), std::invalid_argument);

    // b empty
    a = {1, 1, 1};
    b = {};
    EXPECT_THROW(direct2FormFilter(x, a, b), std::invalid_argument);
}

TEST(DigitalFilterShould, Order2EmptyUInput) {
    std::vector<double> a = {1, 0.5, -1};
    std::vector<double> b = {0.5, -1, 0.5};
    double x = 1;

    crf::expected<DF2IIRFilter> result = direct2FormFilter(x, a, b);
    ASSERT_TRUE(result);

    DF2IIRFilter val = result.value();
    EXPECT_TRUE(std::vector<double>({2, 2}) == val.u);
    EXPECT_EQ(0.0, val.y);
}

TEST(DigitalFilterShould, Order2Input) {
    std::vector<double> a = {1, 0.5, -1};
    std::vector<double> b = {0.5, -1, 0.5};
    std::vector<double> u = {1, 2};
    double x = 1;

    crf::expected<DF2IIRFilter> result = direct2FormFilter(x, a, b, u);
    ASSERT_TRUE(result);

    DF2IIRFilter val = result.value();
    EXPECT_TRUE(std::vector<double>({2.5, 1}) == val.u);
    EXPECT_EQ(1.25, val.y);
}

TEST(DigitalFilterShould, Order3EmptyUInput) {
    std::vector<double> a = {1, 1, 1, 1};
    std::vector<double> b = {2, -1, 1, 1};
    double x = 5;

    crf::expected<DF2IIRFilter> result = direct2FormFilter(x, a, b);
    ASSERT_TRUE(result);

    DF2IIRFilter val = result.value();
    EXPECT_TRUE(std::vector<double>({1.25, 1.25, 1.25}) == val.u);
    EXPECT_EQ(3.75, val.y);
}

TEST(DigitalFilterShould, Order3Input) {
    std::vector<double> a = {1, 1, 1, 1};
    std::vector<double> b = {2, -1, 1, 1};
    std::vector<double> u = {1, -0.5, 1};
    double x = 5;

    crf::expected<DF2IIRFilter> result = direct2FormFilter(x, a, b, u);
    ASSERT_TRUE(result);

    DF2IIRFilter val = result.value();
    EXPECT_TRUE(std::vector<double>({3.5, 1, -0.5}) == val.u);
    EXPECT_EQ(6.5, val.y);
}
