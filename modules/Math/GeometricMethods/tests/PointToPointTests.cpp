/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include <vector>

#include "GeometricMethods/PointToPoint/PointToPoint.hpp"

using crf::math::geometricmethods::PointToPoint;

class PointToPointShould: public ::testing::Test {
 protected:
    PointToPointShould(): logger_("PointToPointShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~PointToPointShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<PointToPoint> sut_;
};

TEST_F(PointToPointShould, ThrowExceptionIfMaxmimumFirstDerivativeOrSecondDerivativeAreZeroOrLess) {
    std::vector<double> points0thderivative = {0, 3};
    double max1stderivative = 0;
    double max2ndderivative = 0;

    ASSERT_THROW(sut_.reset(new PointToPoint(
        points0thderivative,
        max1stderivative,
        max2ndderivative)), std::runtime_error);
}

TEST_F(PointToPointShould, NotThrowExceptionWhenBuiltCorrectly) {
    std::vector<double> points0thderivative = {0, 3};
    double max1stderivative = 1;
    double max2ndderivative = 1;

    ASSERT_NO_THROW(sut_.reset(new PointToPoint(
        points0thderivative,
        max1stderivative,
        max2ndderivative)));
}

TEST_F(PointToPointShould, CorrectlyGetRangeOfFeasibleFunciton) {
    std::vector<double> points0thderivative = {0, 3, 0};
    double max1stderivative = 1;
    double max2ndderivative = 1;

    ASSERT_NO_THROW(sut_.reset(new PointToPoint(
        points0thderivative,
        max1stderivative,
        max2ndderivative)));

    ASSERT_NEAR(sut_->getRange().value(), 7.5, 0.000001);

    ASSERT_NEAR(sut_->evaluate(0, 0).value(), 0, 0.000001);
    ASSERT_NEAR(sut_->evaluate(0, 1).value(), 0, 0.000001);

    ASSERT_NEAR(sut_->evaluate(7.5, 0).value(), 0, 0.000001);
    ASSERT_NEAR(sut_->evaluate(7.5, 1).value(), 0, 0.000001);
}

TEST_F(PointToPointShould, VelocitiesInLinearPartAreTheMax) {
    std::vector<double> points0thderivative = {0, 3, 0};
    double max1stderivative = 1;
    double max2ndderivative = 1;

    ASSERT_NO_THROW(sut_.reset(new PointToPoint(
        points0thderivative,
        max1stderivative,
        max2ndderivative)));

    ASSERT_NEAR(sut_->getRange().value(), 7.5, 0.000001);

    ASSERT_NEAR(sut_->evaluate(0, 0).value(), 0, 0.000001);
    ASSERT_NEAR(sut_->evaluate(0, 1).value(), 0, 0.000001);

    ASSERT_NEAR(sut_->evaluate(1.5, 1).value(), 1, 0.000001);
    ASSERT_NEAR(sut_->evaluate(5.5, 1).value(), -1, 0.000001);

    ASSERT_NEAR(sut_->evaluate(7.5, 0).value(), 0, 0.000001);
    ASSERT_NEAR(sut_->evaluate(7.5, 1).value(), 0, 0.000001);
}

TEST_F(PointToPointShould, ThrowExceptionIfPathIsNotLongEnough) {
    std::vector<double> points0thderivative = {0};
    double max1stderivative = 1;
    double max2ndderivative = 1;

    ASSERT_THROW(sut_.reset(new PointToPoint(
        points0thderivative,
        max1stderivative,
        max2ndderivative)), std::runtime_error);
}

TEST_F(PointToPointShould, ThrowExceptionIfmaxVelMakesNoSense) {
    std::vector<double> points0thderivative = {0, 3, 0};
    double max1stderivative = 0;
    double max2ndderivative = 1;

    ASSERT_THROW(sut_.reset(new PointToPoint(
        points0thderivative,
        max1stderivative,
        max2ndderivative)), std::runtime_error);
}

TEST_F(PointToPointShould, ComputeLongTrajectoryAccuraetly) {
    std::vector<double> points0thderivative = {1, 2, 1, 0, 1, 2, 3, 2, 3, 4, 3, 2, 1};
    double max1stderivative = 0.5;
    double max2ndderivative = 2;

    ASSERT_NO_THROW(sut_.reset(new PointToPoint(
        points0thderivative,
        max1stderivative,
        max2ndderivative)));

    ASSERT_NEAR(sut_->getRange().value(), 24.375, 0.0001);

    ASSERT_NEAR(sut_->evaluate(0, 0).value(), 1, 0.000001);
    ASSERT_NEAR(sut_->evaluate(0, 1).value(), 0, 0.000001);

    ASSERT_NEAR(sut_->evaluate(24.5625, 0).value(), 1, 0.000001);
    ASSERT_NEAR(sut_->evaluate(24.5625, 1).value(), 0, 0.000001);
}

TEST_F(PointToPointShould, WorkWithOtherCTor) {
    std::vector<double> points0thderivative = {0, 1, 0};
    std::vector<double> ranges = {0, 1, 2};
    double max2ndderivative = 3;

    ASSERT_NO_THROW(sut_.reset(new PointToPoint(
        points0thderivative,
        ranges,
        max2ndderivative)));

    ASSERT_NEAR(sut_->getRange().value(), 2.5, 0.000001);
}

TEST_F(PointToPointShould, ThrowExceptionIfTimesAreNotFeasibleWithTheAccProvided) {
    std::vector<double> points0thderivative = {0, 1, 0};
    std::vector<double> ranges = {0, 1, 2};
    double max2ndderivative = 1;

    ASSERT_THROW(sut_.reset(new PointToPoint(
        points0thderivative,
        ranges,
        max2ndderivative)), std::runtime_error);
}

TEST_F(PointToPointShould, PlaceEqWhereNeeded) {
    std::vector<double> points0thderivative = {0, 1, 0};
    std::vector<double> ranges = {3, 4, 5};
    double max2ndderivative = 3;

    ASSERT_NO_THROW(sut_.reset(new PointToPoint(
        points0thderivative,
        ranges,
        max2ndderivative)));

    ASSERT_NEAR(sut_->getRange().value(), 2.5, 0.000001);
}

TEST_F(PointToPointShould, StartAndEndAccurate) {
    std::vector<double> points0thderivative = {0, 1, 0};
    std::vector<double> ranges = {3, 4, 5};
    double max2ndderivative = 3;

    ASSERT_NO_THROW(sut_.reset(new PointToPoint(
        points0thderivative,
        ranges,
        max2ndderivative)));

    ASSERT_NEAR(sut_->getRange().value(), 2.5, 0.000001);

    ASSERT_NEAR(sut_->evaluate(sut_->getStartPoint(), 0).value(), 0, 0.000001);
    ASSERT_NEAR(sut_->evaluate(sut_->getStartPoint(), 1).value(), 0, 0.000001);

    ASSERT_NEAR(sut_->evaluate(sut_->getStartPoint() + sut_->getRange().value(), 0).value(), 0, 0.000001);  // NOLINT
    ASSERT_NEAR(sut_->evaluate(sut_->getStartPoint() + sut_->getRange().value(), 1).value(), 0, 0.000001);  // NOLINT
}
