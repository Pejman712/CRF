/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include "GeometricMethods/Sinusoid/Sinusoid.hpp"

class SinusoidShould: public ::testing::Test {
 protected:
    SinusoidShould(): logger_("SinusoidShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~SinusoidShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::math::geometricmethods::Sinusoid> sut_;
};

TEST_F(SinusoidShould, returnVariablesAreNotCorrectlyInitialized) {
    double start0thDerivative = 0.0;
    double end0thDerivative = 4.0;
    double limit1stDerivative = 0.0;

    ASSERT_THROW(sut_.reset(new crf::math::geometricmethods::Sinusoid(start0thDerivative,
        end0thDerivative, limit1stDerivative)), std::runtime_error);

    start0thDerivative = 0.0;
    end0thDerivative = 0.0;
    limit1stDerivative = 1.0;

    ASSERT_THROW(sut_.reset(new crf::math::geometricmethods::Sinusoid(start0thDerivative,
        end0thDerivative, limit1stDerivative)), std::runtime_error);
}

TEST_F(SinusoidShould, returnCorrectResults) {
    double start0thDerivative = 0.0;
    double end0thDerivative = 4.0;
    double limit1stDerivative = 1.0;

    ASSERT_NO_THROW(sut_.reset(new crf::math::geometricmethods::Sinusoid(start0thDerivative,
        end0thDerivative, limit1stDerivative)));

    std::optional<double> out = sut_->getRange();
    ASSERT_TRUE(out);
    ASSERT_DOUBLE_EQ(out.value(), 8.0);
    std::optional<double> out00 = sut_->evaluate(0.0, 0);
    ASSERT_TRUE(out00);
    ASSERT_DOUBLE_EQ(out00.value(), 0.0);
    std::optional<double> out01 = sut_->evaluate(0.0, 1);
    ASSERT_TRUE(out01);
    ASSERT_DOUBLE_EQ(out01.value(), 0.0);
    std::optional<double> out02 = sut_->evaluate(0.0, 2);
    ASSERT_TRUE(out02);
    ASSERT_DOUBLE_EQ(out02.value(), 0.0);
    std::optional<double> out10 = sut_->evaluate(1.6, 0);
    ASSERT_TRUE(out10);
    ASSERT_DOUBLE_EQ(out10.value(), 0.1367082569008261);
    std::optional<double> out11 = sut_->evaluate(1.6, 1);
    ASSERT_TRUE(out11);
    ASSERT_DOUBLE_EQ(out11.value(), 0.3064510716211361);
    std::optional<double> out12 = sut_->evaluate(1.6, 2);
    ASSERT_TRUE(out12);
    ASSERT_DOUBLE_EQ(out12.value(), 0.4522542485937368);
    std::optional<double> out20 = sut_->evaluate(3.2, 0);
    ASSERT_TRUE(out20);
    ASSERT_DOUBLE_EQ(out20.value(), 1.209988783993330);
    std::optional<double> out21 = sut_->evaluate(3.2, 1);
    ASSERT_TRUE(out21);
    ASSERT_DOUBLE_EQ(out21.value(), 0.9513653457281315);
    std::optional<double> out22 = sut_->evaluate(3.2, 2);
    ASSERT_TRUE(out22);
    ASSERT_DOUBLE_EQ(out22.value(), 0.1727457514062632);
    std::optional<double> out30 = sut_->evaluate(4.8, 0);
    ASSERT_TRUE(out30);
    ASSERT_DOUBLE_EQ(out30.value(), 2.790011216006670);
    std::optional<double> out31 = sut_->evaluate(4.8, 1);
    ASSERT_TRUE(out31);
    ASSERT_DOUBLE_EQ(out31.value(), 0.9513653457281315);
    std::optional<double> out32 = sut_->evaluate(4.8, 2);
    ASSERT_TRUE(out32);
    ASSERT_DOUBLE_EQ(out32.value(), -0.1727457514062631);
    std::optional<double> out40 = sut_->evaluate(6.4, 0);
    ASSERT_TRUE(out40);
    ASSERT_DOUBLE_EQ(out40.value(), 3.863291743099174);
    std::optional<double> out41 = sut_->evaluate(6.4, 1);
    ASSERT_TRUE(out41);
    ASSERT_DOUBLE_EQ(out41.value(), 0.3064510716211359);
    std::optional<double> out42 = sut_->evaluate(6.4, 2);
    ASSERT_TRUE(out42);
    ASSERT_DOUBLE_EQ(out42.value(), -0.4522542485937369);
    std::optional<double> out50 = sut_->evaluate(8.0, 0);
    ASSERT_TRUE(out50);
    ASSERT_DOUBLE_EQ(out50.value(), 4.0);
    std::optional<double> out51 = sut_->evaluate(8.0, 1);
    ASSERT_TRUE(out51);
    ASSERT_DOUBLE_EQ(out51.value(), 0.0);
    std::optional<double> out52 = sut_->evaluate(8.0, 2);
    ASSERT_TRUE(out52);
    ASSERT_DOUBLE_EQ(out52.value(), 0.0);
}

TEST_F(SinusoidShould, returnCorrectResultsNegativeCurveScalingFactor) {
    double start0thDerivative = 3.0;
    double end0thDerivative = -1.0;
    double limit1stDerivative = 1.0;

    ASSERT_NO_THROW(sut_.reset(new crf::math::geometricmethods::Sinusoid(start0thDerivative,
        end0thDerivative, limit1stDerivative)));

    std::optional<double> out = sut_->getRange();
    ASSERT_TRUE(out);
    ASSERT_DOUBLE_EQ(out.value(), 8.0);
    std::optional<double> out00 = sut_->evaluate(0.0, 0);
    ASSERT_TRUE(out00);
    ASSERT_DOUBLE_EQ(out00.value(), 3.0);
    std::optional<double> out01 = sut_->evaluate(0.0, 1);
    ASSERT_TRUE(out01);
    ASSERT_DOUBLE_EQ(out01.value(), 0.0);
    std::optional<double> out02 = sut_->evaluate(0.0, 2);
    ASSERT_TRUE(out02);
    ASSERT_DOUBLE_EQ(out02.value(), 0.0);
    std::optional<double> out10 = sut_->evaluate(1.6, 0);
    ASSERT_TRUE(out10);
    ASSERT_DOUBLE_EQ(out10.value(), 2.863291743099174);
    std::optional<double> out11 = sut_->evaluate(1.6, 1);
    ASSERT_TRUE(out11);
    ASSERT_DOUBLE_EQ(out11.value(), -0.3064510716211361);
    std::optional<double> out12 = sut_->evaluate(1.6, 2);
    ASSERT_TRUE(out12);
    ASSERT_DOUBLE_EQ(out12.value(), -0.4522542485937368);
    std::optional<double> out20 = sut_->evaluate(3.2, 0);
    ASSERT_TRUE(out20);
    ASSERT_DOUBLE_EQ(out20.value(), 1.790011216006670);
    std::optional<double> out21 = sut_->evaluate(3.2, 1);
    ASSERT_TRUE(out21);
    ASSERT_DOUBLE_EQ(out21.value(), -0.9513653457281315);
    std::optional<double> out22 = sut_->evaluate(3.2, 2);
    ASSERT_TRUE(out22);
    ASSERT_DOUBLE_EQ(out22.value(), -0.1727457514062632);
    std::optional<double> out30 = sut_->evaluate(4.8, 0);
    ASSERT_TRUE(out30);
    ASSERT_DOUBLE_EQ(out30.value(), 0.2099887839933299);
    std::optional<double> out31 = sut_->evaluate(4.8, 1);
    ASSERT_TRUE(out31);
    ASSERT_DOUBLE_EQ(out31.value(), -0.9513653457281315);
    std::optional<double> out32 = sut_->evaluate(4.8, 2);
    ASSERT_TRUE(out32);
    ASSERT_DOUBLE_EQ(out32.value(), 0.1727457514062631);
    std::optional<double> out40 = sut_->evaluate(6.4, 0);
    ASSERT_TRUE(out40);
    ASSERT_DOUBLE_EQ(out40.value(), -0.8632917430991736);
    std::optional<double> out41 = sut_->evaluate(6.4, 1);
    ASSERT_TRUE(out41);
    ASSERT_DOUBLE_EQ(out41.value(), -0.3064510716211360);
    std::optional<double> out42 = sut_->evaluate(6.4, 2);
    ASSERT_TRUE(out42);
    ASSERT_DOUBLE_EQ(out42.value(), 0.4522542485937369);
    std::optional<double> out50 = sut_->evaluate(8.0, 0);
    ASSERT_TRUE(out50);
    ASSERT_DOUBLE_EQ(out50.value(), -1.0);
    std::optional<double> out51 = sut_->evaluate(8.0, 1);
    ASSERT_TRUE(out51);
    ASSERT_DOUBLE_EQ(out51.value(), 0.0);
    std::optional<double> out52 = sut_->evaluate(8.0, 2);
    ASSERT_TRUE(out52);
    ASSERT_DOUBLE_EQ(out52.value(), 0.0);
}

TEST_F(SinusoidShould, returnCorrectResultsUsingEndEvalPointMethod) {
    double start0thDerivative = 0.0;
    double end0thDerivative = 4.0;
    double range = 10.0;

    ASSERT_NO_THROW(sut_.reset(new crf::math::geometricmethods::Sinusoid(start0thDerivative,
        end0thDerivative, range, crf::math::geometricmethods::ComputationMethod::SetRange)));

    std::optional<double> out = sut_->getRange();
    ASSERT_TRUE(out);
    ASSERT_DOUBLE_EQ(out.value(), 10.0);
    std::optional<double> out00 = sut_->evaluate(0.0, 0);
    ASSERT_TRUE(out00);
    ASSERT_DOUBLE_EQ(out00.value(), 0.0);
    std::optional<double> out01 = sut_->evaluate(0.0, 1);
    ASSERT_TRUE(out01);
    ASSERT_DOUBLE_EQ(out01.value(), 0.0);
    std::optional<double> out02 = sut_->evaluate(0.0, 2);
    ASSERT_TRUE(out02);
    ASSERT_DOUBLE_EQ(out02.value(), 0.0);
    std::optional<double> out10 = sut_->evaluate(2.0, 0);
    ASSERT_TRUE(out10);
    ASSERT_DOUBLE_EQ(out10.value(), 0.1367082569008260);
    std::optional<double> out11 = sut_->evaluate(2.0, 1);
    ASSERT_TRUE(out11);
    ASSERT_DOUBLE_EQ(out11.value(), 0.2451608572969089);
    std::optional<double> out12 = sut_->evaluate(2.0, 2);
    ASSERT_TRUE(out12);
    ASSERT_DOUBLE_EQ(out12.value(), 0.2894427190999916);
    std::optional<double> out20 = sut_->evaluate(4.0, 0);
    ASSERT_TRUE(out20);
    ASSERT_DOUBLE_EQ(out20.value(), 1.209988783993330);
    std::optional<double> out21 = sut_->evaluate(4.0, 1);
    ASSERT_TRUE(out21);
    ASSERT_DOUBLE_EQ(out21.value(), 0.7610922765825053);
    std::optional<double> out22 = sut_->evaluate(4.0, 2);
    ASSERT_TRUE(out22);
    ASSERT_DOUBLE_EQ(out22.value(), 0.1105572809000085);
    std::optional<double> out30 = sut_->evaluate(6.0, 0);
    ASSERT_TRUE(out30);
    ASSERT_DOUBLE_EQ(out30.value(), 2.79001121600667);
    std::optional<double> out31 = sut_->evaluate(6.0, 1);
    ASSERT_TRUE(out31);
    ASSERT_DOUBLE_EQ(out31.value(), 0.7610922765825050);
    std::optional<double> out32 = sut_->evaluate(6.0, 2);
    ASSERT_TRUE(out32);
    ASSERT_DOUBLE_EQ(out32.value(), -0.1105572809000084);
    std::optional<double> out40 = sut_->evaluate(8.0, 0);
    ASSERT_TRUE(out40);
    ASSERT_DOUBLE_EQ(out40.value(), 3.863291743099174);
    std::optional<double> out41 = sut_->evaluate(8.0, 1);
    ASSERT_TRUE(out41);
    ASSERT_DOUBLE_EQ(out41.value(), 0.2451608572969086);
    std::optional<double> out42 = sut_->evaluate(8.0, 2);
    ASSERT_TRUE(out42);
    ASSERT_DOUBLE_EQ(out42.value(), -0.2894427190999917);
    std::optional<double> out50 = sut_->evaluate(10.0, 0);
    ASSERT_TRUE(out50);
    ASSERT_DOUBLE_EQ(out50.value(), 4.0);
    std::optional<double> out51 = sut_->evaluate(10.0, 1);
    ASSERT_TRUE(out51);
    ASSERT_DOUBLE_EQ(out51.value(), 0.0);
    std::optional<double> out52 = sut_->evaluate(10.0, 2);
    ASSERT_TRUE(out52);
    ASSERT_DOUBLE_EQ(out52.value(), 0.0);
}

TEST_F(SinusoidShould, returnMaximunValueIfInputEvalPointIsNotWithinTheRange) {
    double start0thDerivative = 0.0;
    double end0thDerivative = 4.0;
    double limit1stDerivative = 1.0;

    ASSERT_NO_THROW(sut_.reset(new crf::math::geometricmethods::Sinusoid(start0thDerivative,
        end0thDerivative, limit1stDerivative)));

    std::optional<double> out00 = sut_->evaluate(-0.5, 0);
    ASSERT_TRUE(out00);
    ASSERT_DOUBLE_EQ(out00.value(), 0.0);
    std::optional<double> out01 = sut_->evaluate(-0.5, 1);
    ASSERT_TRUE(out01);
    ASSERT_DOUBLE_EQ(out01.value(), 0.0);
    std::optional<double> out02 = sut_->evaluate(-0.5, 2);
    ASSERT_TRUE(out02);
    ASSERT_DOUBLE_EQ(out02.value(), 0.0);
    std::optional<double> out10 = sut_->evaluate(9.5, 0);
    ASSERT_TRUE(out10);
    ASSERT_DOUBLE_EQ(out10.value(), 4.0);
    std::optional<double> out11 = sut_->evaluate(9.5, 1);
    ASSERT_TRUE(out11);
    ASSERT_DOUBLE_EQ(out11.value(), 0.0);
    std::optional<double> out12 = sut_->evaluate(9.5, 2);
    ASSERT_TRUE(out12);
    ASSERT_DOUBLE_EQ(out12.value(), 0.0);
}

TEST_F(SinusoidShould, returnEmptyVectorIfInputDerIsNot0) {
    double start0thDerivative = 0.0;
    double end0thDerivative = 4.0;
    double limit1stDerivative = 1.0;

    ASSERT_NO_THROW(sut_.reset(new crf::math::geometricmethods::Sinusoid(start0thDerivative,
        end0thDerivative, limit1stDerivative)));

    ASSERT_EQ(sut_->evaluate(0.2, 3), std::nullopt);
}
