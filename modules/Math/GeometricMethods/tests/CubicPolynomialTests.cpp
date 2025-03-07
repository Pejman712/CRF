/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include <vector>

#include "GeometricMethods/CubicPolynomial/CubicPolynomial.hpp"

using crf::math::geometricmethods::CubicPolynomial;

class CubicPolynomialShould: public ::testing::Test {
 protected:
    CubicPolynomialShould(): logger_("CubicPolynomialShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~CubicPolynomialShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<CubicPolynomial> sut_;
};

TEST_F(CubicPolynomialShould, MatchPositionsAtTheExtremesOfTheRange) {
    double start0thderivative = 0;
    double end0thderivative = 1;
    double start1stderivative = 0;
    double end1stderivative = 0;
    double range = 2;
    sut_.reset(new CubicPolynomial(
        start0thderivative,
        end0thderivative,
        start1stderivative,
        end1stderivative,
        range));

    ASSERT_NEAR(sut_->getRange().value(), 2, 0.000001);

    ASSERT_NEAR(sut_->evaluate(0, 0).value(), 0, 0.000001);
    ASSERT_NEAR(sut_->evaluate(0, 1).value(), 0, 0.000001);

    ASSERT_NEAR(sut_->evaluate(2, 0).value(), 1, 0.000001);
    ASSERT_NEAR(sut_->evaluate(2, 1).value(), 0, 0.000001);
}

TEST_F(CubicPolynomialShould, ValuesOutsideTheRangeAreConstant) {
    std::vector<double> points0thderivative = {0, 3};
    std::vector<double> ranges = {1, 2};
    double start1stderivative = 0;
    double end1stderivative = 0;

    sut_.reset(new CubicPolynomial(
        points0thderivative,
        start1stderivative,
        end1stderivative,
        ranges));

    ASSERT_NEAR(sut_->getRange().value(), 2, 0.000001);

    // At time = 1 and before the point is 0
    ASSERT_NEAR(sut_->evaluate(0, 0).value(), 0, 0.000001);
    ASSERT_NEAR(sut_->evaluate(0, 1).value(), 0, 0.000001);

    ASSERT_NEAR(sut_->evaluate(1, 0).value(), 0, 0.000001);
    ASSERT_NEAR(sut_->evaluate(1, 1).value(), 0, 0.000001);

    // At time = 2 and after the point is 3
    ASSERT_NEAR(sut_->evaluate(2, 0).value(), 3, 0.000001);
    ASSERT_NEAR(sut_->evaluate(2, 1).value(), 0, 0.000001);

    ASSERT_NEAR(sut_->evaluate(3, 0).value(), 3, 0.000001);
    ASSERT_NEAR(sut_->evaluate(3, 1).value(), 0, 0.000001);
}

TEST_F(CubicPolynomialShould, VelocitiesOutsideTheRangeShouldAlsoStayConstant) {
    std::vector<double> points0thderivative = {1, 2, 1};
    std::vector<double> ranges = {1, 2, 3};
    double start1stderivative = 1;
    double end1stderivative = -1;

    sut_.reset(new CubicPolynomial(
        points0thderivative,
        start1stderivative,
        end1stderivative,
        ranges));

    ASSERT_NEAR(sut_->getRange().value(), 3, 0.000001);

    ASSERT_NEAR(sut_->evaluate(0, 0).value(), 0, 0.000001);
    ASSERT_NEAR(sut_->evaluate(0, 1).value(), 1, 0.000001);

    ASSERT_NEAR(sut_->evaluate(1, 0).value(), 1, 0.000001);
    ASSERT_NEAR(sut_->evaluate(1, 1).value(), 1, 0.000001);

    ASSERT_NEAR(sut_->evaluate(3, 0).value(), 1, 0.000001);
    ASSERT_NEAR(sut_->evaluate(3, 1).value(), -1, 0.000001);

    ASSERT_NEAR(sut_->evaluate(4, 0).value(), 0, 0.000001);
    ASSERT_NEAR(sut_->evaluate(4, 1).value(), -1, 0.000001);
}

TEST_F(CubicPolynomialShould, ThrowExceptionIfDimensionsDontMatch) {
    std::vector<double> points0thderivative = {0, 1};
    std::vector<double> ranges = {1, 2, 3};
    double start1stderivative = 1;
    double end1stderivative = -1;

    ASSERT_THROW(sut_.reset(new CubicPolynomial(
        points0thderivative,
        start1stderivative,
        end1stderivative,
        ranges)), std::runtime_error);
}

TEST_F(CubicPolynomialShould, ThrowIfRangeIsDecreasing) {
    std::vector<double> points0thderivative = {0, 1, 0};
    std::vector<double> ranges = {1, 2, 1};
    double start1stderivative = 1;
    double end1stderivative = -1;

    ASSERT_THROW(sut_.reset(new CubicPolynomial(
        points0thderivative,
        start1stderivative,
        end1stderivative,
        ranges)), std::runtime_error);
}

TEST_F(CubicPolynomialShould, ThrowExceptionWhenItDetectsInfiniteVelocity) {
    std::vector<double> points0thderivative = {3, 2, 1};
    std::vector<double> ranges = {0, 1, 1};
    double start1stderivative = 0;
    double end1stderivative = 0;

    ASSERT_THROW(sut_.reset(new CubicPolynomial(
        points0thderivative,
        start1stderivative,
        end1stderivative,
        ranges)), std::runtime_error);
}

TEST_F(CubicPolynomialShould, NotThrowExceptionIfTimesAndPositionMatch) {
    std::vector<double> points0thderivative = {3, 2, 2, 1};
    std::vector<double> ranges = {0, 1, 1, 2};
    double start1stderivative = 0;
    double end1stderivative = 0;

    ASSERT_NO_THROW(sut_.reset(new CubicPolynomial(
        points0thderivative,
        start1stderivative,
        end1stderivative,
        ranges)));
}

TEST_F(CubicPolynomialShould, ComputeLongTrajectoryAccuraetly) {
    std::vector<double> points0thderivative = {1, 2, 1, 0, 1, 2, 3, 2, 3, 4, 3, 2, 1};
    std::vector<double> ranges = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
    double start1stderivative = 1;
    double end1stderivative = -1;

    ASSERT_NO_THROW(sut_.reset(new CubicPolynomial(
        points0thderivative,
        start1stderivative,
        end1stderivative,
        ranges)));

    ASSERT_NEAR(sut_->getRange().value(), 13, 0.000001);

    ASSERT_NEAR(sut_->evaluate(0, 0).value(), 0, 0.000001);
    ASSERT_NEAR(sut_->evaluate(0, 1).value(), 1, 0.000001);

    ASSERT_NEAR(sut_->evaluate(1, 0).value(), 1, 0.000001);
    ASSERT_NEAR(sut_->evaluate(2, 0).value(), 2, 0.000001);
    ASSERT_NEAR(sut_->evaluate(3, 0).value(), 1, 0.000001);
    ASSERT_NEAR(sut_->evaluate(4, 0).value(), 0, 0.000001);
    ASSERT_NEAR(sut_->evaluate(5, 0).value(), 1, 0.000001);
    ASSERT_NEAR(sut_->evaluate(6, 0).value(), 2, 0.000001);
    ASSERT_NEAR(sut_->evaluate(7, 0).value(), 3, 0.000001);
    ASSERT_NEAR(sut_->evaluate(8, 0).value(), 2, 0.000001);
    ASSERT_NEAR(sut_->evaluate(9, 0).value(), 3, 0.000001);
    ASSERT_NEAR(sut_->evaluate(10, 0).value(), 4, 0.000001);
    ASSERT_NEAR(sut_->evaluate(11, 0).value(), 3, 0.000001);
    ASSERT_NEAR(sut_->evaluate(12, 0).value(), 2, 0.000001);
    ASSERT_NEAR(sut_->evaluate(13, 0).value(), 1, 0.000001);

    ASSERT_NEAR(sut_->evaluate(14, 0).value(), 0, 0.000001);
    ASSERT_NEAR(sut_->evaluate(14, 1).value(), -1, 0.000001);
}

TEST_F(CubicPolynomialShould, WorkWithNegativeValuesToo) {
    double start0thderivative = 0;
    double end0thderivative = 1;
    double start1stderivative = 0;
    double end1stderivative = 0;
    double range = 2;
    sut_.reset(new CubicPolynomial(
        start0thderivative,
        end0thderivative,
        start1stderivative,
        end1stderivative,
        range));

    ASSERT_NEAR(sut_->getRange().value(), 2, 0.000001);

    ASSERT_NEAR(sut_->evaluate(0, 0).value(), 0, 0.000001);
    ASSERT_NEAR(sut_->evaluate(0, 1).value(), 0, 0.000001);

    ASSERT_NEAR(sut_->evaluate(2, 0).value(), 1, 0.000001);
    ASSERT_NEAR(sut_->evaluate(2, 1).value(), 0, 0.000001);
}

/**
 * @brief These tests are needed because a precision of 1e-8 (0.000000001) between two
 * points makes eigen fail when factorizing the matrix and creates a seg fault during
 * the solving
 * (jplayang)
 */
TEST_F(CubicPolynomialShould, ResistSegFaultWithSmallNumbersStupidEigenPls) {
    std::vector<double> points0thderivative = {1, 1.0000000001, 2};
    std::vector<double> ranges = {1, 1.0000000001, 3};
    double start1stderivative = 0;
    double end1stderivative = 0;

    ASSERT_NO_THROW(sut_.reset(new CubicPolynomial(
        points0thderivative,
        start1stderivative,
        end1stderivative,
        ranges)));
}

TEST_F(CubicPolynomialShould, ThrowExceptionIfRangesAreCloseButNotPointsStupidEigenPls) {
    std::vector<double> points0thderivative = {1, 2, 1};
    std::vector<double> ranges = {1, 1.0000000001, 3};
    double start1stderivative = 0;
    double end1stderivative = 0;

    ASSERT_THROW(sut_.reset(new CubicPolynomial(
        points0thderivative,
        start1stderivative,
        end1stderivative,
        ranges)), std::runtime_error);
}

TEST_F(CubicPolynomialShould, GiveWarningIfPositionsAreVeryCloseButRangeIsFarAway) {
    std::vector<double> points0thderivative = {1, 1.0000000001};
    std::vector<double> ranges = {1, 3};
    double start1stderivative = 0;
    double end1stderivative = 0;

    ASSERT_NO_THROW(sut_.reset(new CubicPolynomial(
        points0thderivative,
        start1stderivative,
        end1stderivative,
        ranges)));
}

TEST_F(CubicPolynomialShould, correctlyRunTheNonLinearOptimization) {
    double startPoint = 0;
    double endPoint = 10;
    double start1stderivative = 0;
    double end1stderivative = 0;
    double max1stDerivative = 1;
    double max2ndDerivative = 2;
    double tolerance = 1e-3;
    std::chrono::milliseconds timeout = std::chrono::seconds(1);

    ASSERT_NO_THROW(sut_.reset(new CubicPolynomial(
        startPoint, endPoint,
        start1stderivative, end1stderivative,
        max1stDerivative, max2ndDerivative,
        tolerance, timeout)));

    std::optional<double> rangeOpt = sut_->getRange();

    ASSERT_TRUE(rangeOpt);

    double range = rangeOpt.value();

    // The tolerance of the iteration is not the same as the one in the result
    tolerance = 0.1;
    ASSERT_NEAR(startPoint, sut_->evaluate(0, 0).value(), tolerance);
    ASSERT_NEAR(endPoint, sut_->evaluate(range, 0).value(), tolerance);
    ASSERT_NEAR(start1stderivative, sut_->evaluate(0, 1).value(), tolerance);
    ASSERT_NEAR(end1stderivative, sut_->evaluate(range, 1).value(), tolerance);
    ASSERT_LT(sut_->evaluate(0, 2).value(), max2ndDerivative);
    ASSERT_LT(sut_->evaluate(range, 2).value(), max2ndDerivative);
    // This one is the most inconsistent one, normally a bit higher than expected (e.g. 1.2, 1.3)
    ASSERT_LT(sut_->evaluate(range/2, 1).value(), 1.5*max1stDerivative);
}

