/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Chelsea Davidson CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include "GeometricMethods/CubicOrientationSpline/CubicOrientationSpline.hpp"

using crf::math::geometricmethods::CubicOrientationSpline;
using crf::math::rotation::areAlmostEqual;

class CubicOrientationSplineShould : public ::testing::Test {
 protected:
    CubicOrientationSplineShould() :
        logger_("CubicOrientationSplineShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~CubicOrientationSplineShould() {
        logger_->info(
            "{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<CubicOrientationSpline> sut_;
    double eps_ = 1e-11;
};

TEST_F(CubicOrientationSplineShould, MatchPositionsAtTheExtremesOfTheRange) {
    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)),
    };
    Eigen::Vector3d initialAngularVelocity(0, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0.25, 0, 0);
    std::vector<double> timeInstances = {0.0, 5.0, 10.0, 15.0};

    sut_.reset(new CubicOrientationSpline(
        orientationPoints, initialAngularVelocity, finalAngularVelocity, timeInstances));

    ASSERT_NEAR(sut_->getRange().value(), 15, eps_);

    // Orientation at the start
    Orientation orientationPointsStartEval = sut_->evaluateOrientation(0.0).value();
    ASSERT_TRUE(areAlmostEqual(orientationPointsStartEval, orientationPoints[0],
        eps_));

    // Angular velocity at the start
    Eigen::Vector3d initialAngularVelocityEval = sut_->evaluate(0.0, 1).value();
    ASSERT_NEAR(initialAngularVelocityEval[0], initialAngularVelocity[0], eps_);
    ASSERT_NEAR(initialAngularVelocityEval[1], initialAngularVelocity[1], eps_);
    ASSERT_NEAR(initialAngularVelocityEval[2], initialAngularVelocity[2], eps_);

    // Orientation at the end
    Orientation orientationPointsEndEval = sut_->evaluateOrientation(15.0).value();
    ASSERT_TRUE(areAlmostEqual(orientationPointsEndEval, orientationPoints[3],
        eps_));

    // Angular velocity at the end
    Eigen::Vector3d finalAngularVelocityEval = sut_->evaluate(15.0, 1).value();
    ASSERT_NEAR(finalAngularVelocityEval[0], finalAngularVelocity[0], eps_);
    ASSERT_NEAR(finalAngularVelocityEval[1], finalAngularVelocity[1], eps_);
    ASSERT_NEAR(finalAngularVelocityEval[2], finalAngularVelocity[2], eps_);
}

TEST_F(CubicOrientationSplineShould, ThrowExceptionIfDimensionsDontMatch) {
    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)),
    };
    std::vector<double> timeInstancesLess = {0.0, 5.0, 10.0};
    std::vector<double> timeInstancesMore = {0.0, 5.0, 10.0, 15.0, 20.0};
    Eigen::Vector3d initialAngularVelocity(0, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 0);

    std::stringstream expectedErrorMessageLess;
    expectedErrorMessageLess << "The number of input time instances does not match the one of "
        "orientations. The size of timeInstances was " << timeInstancesLess.size() <<
        " and the size of orientationPoints was " << orientationPoints.size();

    std::stringstream expectedErrorMessageMore;
    expectedErrorMessageMore << "The number of input time instances does not match the one of "
        "orientations. The size of timeInstances was " << timeInstancesMore.size() <<
        " and the size of orientationPoints was " << orientationPoints.size();
    // timeInstances have fewer input instances than orientationPoints
    ASSERT_ANY_THROW({
        try {
            sut_.reset(new CubicOrientationSpline(orientationPoints,
                initialAngularVelocity, finalAngularVelocity, timeInstancesLess));
        } catch (const std::runtime_error& e) {
            // Check the what() message (comment)
            ASSERT_STREQ(expectedErrorMessageLess.str().c_str(), e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    // timeInstances have more input instances than orientationPoints
    ASSERT_ANY_THROW({
        try {
            sut_.reset(new CubicOrientationSpline(orientationPoints,
            initialAngularVelocity, finalAngularVelocity, timeInstancesMore));
        } catch (const std::runtime_error& e) {
            // Check the what() message (comment)
            ASSERT_STREQ(expectedErrorMessageMore.str().c_str(), e.what());
            throw;  // Re-throw to propagate the exception
        }
    });
}

TEST_F(CubicOrientationSplineShould, NotThrowExceptionIfDimensionsMatch) {
    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)),
    };
    std::vector<double> timeInstances = {0.0, 5.0, 10.0, 15.0};
    Eigen::Vector3d initialAngularVelocity(0, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 0);

    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, initialAngularVelocity, finalAngularVelocity, timeInstances)));
}

TEST_F(CubicOrientationSplineShould, ThrowExceptionIfTimeIsDecreasing) {
    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)),
    };
    std::vector<double> timeInstances = {0.0, 5.0, 2.0, 6.0};
    Eigen::Vector3d initialAngularVelocity(0, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 0);

    ASSERT_ANY_THROW({
        try {
            sut_.reset(new CubicOrientationSpline(
                orientationPoints, initialAngularVelocity, finalAngularVelocity, timeInstances));
        } catch (const std::runtime_error& e) {
            // Check the what() message (comment)
            ASSERT_STREQ("Input timeInstances must be in ascending order", e.what());
            throw;  // Re-throw to propagate the exception
        }
    });
}

TEST_F(
    CubicOrientationSplineShould,
    NotThrowExceptionIfVerySmallConstantTimeIntervalsWithLargeOrientationChange) {
    Eigen::Vector3d initialAngularVelocity(0, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 0);

    // Check doesn't throw an error when times change by a small constant amount
    std::vector<double> timeInstances = {0.0, 0.00000001, 0.00000002, 0.00000003};
    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)),
    };

    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, initialAngularVelocity, finalAngularVelocity, timeInstances)));

    // Note - can be computed but has large angular rates.
}

TEST_F(
    CubicOrientationSplineShould,
    NotThrowExceptionIfVerySmallConstantTimeIntervalsWithVerySmallOrientationChange) {
    Eigen::Vector3d initialAngularVelocity(0, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 0);

    // Check doesn't throw an error when time intervals are constant and orientations
    // are changing by a small amount the whole duration
    std::vector<double> timeInstancesChanging = {0.0, 0.00000001, 0.00000002, 0.00000003};
    std::vector<Orientation> orientationPointsChanging = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, -0.000000015, 0.0)),
        Orientation(Eigen::Quaterniond(1.0, -0.000000015, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.000000015)),
        Orientation(Eigen::Quaterniond(1.0, 0.000000015, 0.0, 0.0)),
    };

    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPointsChanging,
        initialAngularVelocity,
        finalAngularVelocity,
        timeInstancesChanging)));
}

TEST_F(
    CubicOrientationSplineShould,
    ThrowExceptionIfSmallVaryingTimeIntervalsWithLargeOrientationChange) {
    Eigen::Vector3d initialAngularVelocity(0, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 0);

    // Check throws an error when times are changing inconsistently and orientations are changing
    std::vector<double> timeInstances = {0.0, 0.00000001, 0.00002, 0.003};
    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)),
    };

    double convergenceTol = 1e-12;
    std::size_t maxIter = 1000;

    std::stringstream expectedErrorMessage;
    expectedErrorMessage << "The algorithm took over " << maxIter << " iterations to converge, "
        "suggesting that the interpolation conditions may be unreasonable. These issues could be "
        "due to small time intervals or the combination of large time intervals with high "
        "initialAngularVelocity or finalAngularVelocity.";

    ASSERT_ANY_THROW({
        try {
            sut_.reset(new CubicOrientationSpline(
                orientationPoints,
                initialAngularVelocity,
                finalAngularVelocity,
                timeInstances,
                convergenceTol,
                maxIter));
        } catch (const std::runtime_error& e) {
            // Check the what() message (comment)
            ASSERT_STREQ(expectedErrorMessage.str().c_str(), e.what());
            throw;  // Re-throw to propagate the exception
        }
    });
}

TEST_F(
    CubicOrientationSplineShould,
    NotThrowExceptionIfSmallVaryingTimeIntervalsWithNoOrientationChange) {
    Eigen::Vector3d initialAngularVelocity(0, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 0);

    // Check doesn't throw an error when times are changing inconsistently
    // by a small amount but the quaternions stay the same
    std::vector<double> timeInstancesVarying = {0.0, 0.00000001, 0.00002, 0.003};

    std::vector<Orientation> orientationPointsConstant = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
    };

    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPointsConstant,
        initialAngularVelocity,
        finalAngularVelocity,
        timeInstancesVarying)));
}

TEST_F(CubicOrientationSplineShould,
    NotThrowExceptionIfProportionalTimeIntervalAndOrientationChange) {
    Eigen::Vector3d initialAngularVelocity(0, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 0);

    // Check doesn't throw an error when times and quaternions are both changing
    // by a small amount at a specific interval
    std::vector<double> timeInstancesSmallChangeAtEnd = {0.0, 5.0, 8.0, 10.0, 10.009};

    std::vector<Orientation> orientationPointsSmallChangeAtEnd = {
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.707106781187, 0.0, 0.0, 0.707106781187)),
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.996469131618467, 0.083959929320713, 0.0, 0.0)),
    };

    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPointsSmallChangeAtEnd,
        initialAngularVelocity,
        finalAngularVelocity,
        timeInstancesSmallChangeAtEnd)));
}

TEST_F(CubicOrientationSplineShould, ThrowExceptionIfHighAngularRateWithLargeTimeIntervals) {
    // Check throws an error when angular rates are high and time intervals are large.
    Eigen::Vector3d large1stDerivative(0, 2, 0);
    Eigen::Vector3d small1stDerivative(0, 0, 0);

    std::vector<double> timeInstances = {0.0, 45.0, 90.0, 135.0};
    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)),
    };

    double convergenceTol = 1e-12;
    std::size_t maxIter = 1000;

    std::stringstream expectedErrorMessage;
    expectedErrorMessage << "The algorithm took over " << maxIter << " iterations to converge, "
        "suggesting that the interpolation conditions may be unreasonable. These issues could be "
        "due to small time intervals or the combination of large time intervals with high "
        "initialAngularVelocity or finalAngularVelocity.";

    // Checking when initial angular rate is high
    ASSERT_ANY_THROW({
        try {
            sut_.reset(new CubicOrientationSpline(
                orientationPoints,
                large1stDerivative,
                small1stDerivative,
                timeInstances,
                convergenceTol,
                maxIter));
        } catch (const std::runtime_error& e) {
            // Check the what() message (comment)
            ASSERT_STREQ(expectedErrorMessage.str().c_str(), e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    // Checking when final angular rate is high
    ASSERT_ANY_THROW({
        try {
            sut_.reset(new CubicOrientationSpline(
                orientationPoints,
                small1stDerivative,
                large1stDerivative,
                timeInstances,
                convergenceTol,
                maxIter));
        } catch (const std::runtime_error& e) {
            // Check the what() message (comment)
            ASSERT_STREQ(expectedErrorMessage.str().c_str(), e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    // Checking when both are high
    ASSERT_ANY_THROW({
        try {
            sut_.reset(new CubicOrientationSpline(
                orientationPoints,
                large1stDerivative,
                large1stDerivative,
                timeInstances,
                convergenceTol,
                maxIter));
        } catch (const std::runtime_error& e) {
            // Check the what() message (comment)
            ASSERT_STREQ(expectedErrorMessage.str().c_str(), e.what());
            throw;  // Re-throw to propagate the exception
        }
    });
}

TEST_F(CubicOrientationSplineShould, NotThrowExceptionIfHighAngularRateWithSmallTimeIntervals) {
    // Check doesn't throw error when angular rates are high but time intervals are somewhat small.
    Eigen::Vector3d large1stDerivative(0, 2, 0);
    Eigen::Vector3d small1stDerivative(0, 0, 0);

    std::vector<double> timeInstancesMedium = {0.0, 5.0, 10.0, 15.0};
    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)),
    };
    // Checking when initial angular rate is high
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, large1stDerivative, small1stDerivative, timeInstancesMedium)));

    // Checking when final angular rate is high
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, small1stDerivative, large1stDerivative, timeInstancesMedium)));

    // Checking when both are high
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, large1stDerivative, large1stDerivative, timeInstancesMedium)));

    // Check doesn't throw an error when angular rates are high but time intervals are small.
    std::vector<double> timeInstancesSmall = {0.0, 0.1, 0.2, 0.3};
    // Checking when initial angular rate is high
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, large1stDerivative, small1stDerivative, timeInstancesSmall)));

    // Checking when final angular rate is high
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, small1stDerivative, large1stDerivative, timeInstancesSmall)));

    // Checking when both are high
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, large1stDerivative, large1stDerivative, timeInstancesSmall)));
}

TEST_F(CubicOrientationSplineShould, NotThrowExceptionIfHighAngularRateWithNoOrientationChange) {
    // Check doesn't throw an error when angular rates are high,
    // time intervals are large, but orientation doesn't change.
    Eigen::Vector3d large1stDerivative(0, 2, 0);
    Eigen::Vector3d small1stDerivative(0, 0, 0);

    std::vector<double> timeInstancesLarge = {0.0, 45.0, 90.0, 135.0};
    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
    };
    // Checking when initial angular rate is high
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, large1stDerivative, small1stDerivative, timeInstancesLarge)));

    // Checking when final angular rate is high
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, small1stDerivative, large1stDerivative, timeInstancesLarge)));

    // Checking when both are high
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, large1stDerivative, large1stDerivative, timeInstancesLarge)));

    // Check doesn't throw an error when angular rates are high,
    // time intervals are small, but orientation doesn't change.
    std::vector<double> timeInstancesSmall = {0.0, 0.1, 0.2, 0.3};
    // Checking when initial angular rate is high
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, large1stDerivative, small1stDerivative, timeInstancesSmall)));

    // Checking when final angular rate is high
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, small1stDerivative, large1stDerivative, timeInstancesSmall)));

    // Checking when both are high
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, large1stDerivative, large1stDerivative, timeInstancesSmall)));
}
TEST_F(CubicOrientationSplineShould, NotThrowExceptionWithTwoInputs) {
    std::vector<double> timeInstances = {1.0, 5.0};

    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0, 0.0)),
    };

    Eigen::Vector3d initialAngularVelocity(0.0, 0.0, 1.0);
    Eigen::Vector3d finalAngularVelocity(1.0, 0.0, 0.0);

    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, initialAngularVelocity, finalAngularVelocity, timeInstances)));

    // Checking two points are correct:
    ASSERT_TRUE(areAlmostEqual(sut_->evaluateOrientation(timeInstances[0]).value(),
        orientationPoints[0], eps_));
    ASSERT_TRUE(sut_->evaluate(timeInstances[0], 1).value().isApprox(initialAngularVelocity, eps_));

    ASSERT_TRUE(areAlmostEqual(sut_->evaluateOrientation(timeInstances[1]).value(),
        orientationPoints[1], eps_));
    ASSERT_TRUE(sut_->evaluate(timeInstances[1], 1).value().isApprox(finalAngularVelocity, eps_));
}


TEST_F(CubicOrientationSplineShould, ThrowExceptionWithLessThanTwoInputs) {
    std::vector<double> timeInstances1Input1 = {5.0};

    std::vector<Orientation> orientationPoints1Input = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0, 0.0)),
    };

    Eigen::Vector3d initialAngularVelocity(0, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 0);

    std::stringstream expectedErrorMessage1Input;
    expectedErrorMessage1Input <<  "Require at least 2 input instances to generate a Cubic "
        "Orientation Spline. Only 1 were given";

    // Check that it throws the error when there are less than 3 inputs
    ASSERT_ANY_THROW({
        try {
            sut_.reset(new CubicOrientationSpline(
                orientationPoints1Input,
                initialAngularVelocity,
                finalAngularVelocity,
                timeInstances1Input1));
        } catch (const std::runtime_error& e) {
            // Check the what() message (comment)
            ASSERT_STREQ(expectedErrorMessage1Input.str().c_str(), e.what());
            throw;  // Re-throw to propagate the exception
        }
    });

    // Check that it throws the error when the inputs are empty
    std::vector<double> timeInstances0Inputs = {};
    std::vector<Orientation> orientationPoints0Inputs = {};

    std::stringstream expectedErrorMessage0Inputs;
    expectedErrorMessage0Inputs << "Require at least 2 input instances to generate a Cubic "
        "Orientation Spline. Only 0 were given";

    ASSERT_ANY_THROW({
        try {
            sut_.reset(new CubicOrientationSpline(
                orientationPoints0Inputs,
                initialAngularVelocity,
                finalAngularVelocity,
                timeInstances0Inputs));
        } catch (const std::runtime_error& e) {
            // Check the what() message (comment)
            ASSERT_STREQ(expectedErrorMessage0Inputs.str().c_str(), e.what());
            throw;  // Re-throw to propagate the exception
        }
    });
}

TEST_F(CubicOrientationSplineShould, ForceOrientationEvaluationPointToBeInsideTheRange) {
    Eigen::Vector3d zeroInitialAngularVelocity(0, 0, 0);
    Eigen::Vector3d zeroFinalAngularVelocity(0, 0, 0);
    Eigen::Vector3d initialAngularVelocity(1, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 1);

    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)),
    };

    std::vector<double> timeInstances = {1.0, 6.0, 11.0, 16.0};

    //  ------ Checking when only initial velocity is zero -------
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, zeroInitialAngularVelocity, finalAngularVelocity, timeInstances)));

    // Checking when evaluation point is negative
    Orientation orientationPointsEvalNeg = sut_->evaluateOrientation(-100.0).value();
    ASSERT_TRUE(areAlmostEqual(orientationPointsEvalNeg, orientationPoints[0],
        eps_));

    // Checking when evaluation point is below minimum
    Orientation orientationPointsEvalBelow = sut_->evaluateOrientation(0.0).value();
    ASSERT_TRUE(areAlmostEqual(orientationPointsEvalBelow, orientationPoints[0],
        eps_));

    // Checking when evaluation point is above maximum
    ASSERT_EQ(sut_->evaluateOrientation(16.1), std::nullopt);

    //  ------ Checking when only final velocity is zero -------
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, initialAngularVelocity, zeroFinalAngularVelocity, timeInstances)));

    // Checking when evaluation point is negative
    ASSERT_EQ(sut_->evaluateOrientation(-100.0), std::nullopt);

    // Checking when evaluation point is below the minimum
    ASSERT_EQ(sut_->evaluateOrientation(0.0), std::nullopt);

    Orientation orientationPointsEvalAbove = sut_->evaluateOrientation(16.5).value();
    ASSERT_TRUE(areAlmostEqual(orientationPointsEvalAbove,
        orientationPoints[timeInstances.size() - 1], eps_));
}

TEST_F(CubicOrientationSplineShould, ThrowExceptionIfEvaluationDerivativeIsInvalid) {
    Eigen::Vector3d initialAngularVelocity(1, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 1);

    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)),
    };

    std::vector<double> timeInstances = {0.0, 5.0, 10.0, 15.0};

    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, initialAngularVelocity, finalAngularVelocity, timeInstances)));

    // Shouldn't accept derivative 0
    ASSERT_EQ(sut_->evaluate(8.0, 0), std::nullopt);

    // Shouldn't accept negative derivative
    ASSERT_EQ(sut_->evaluate(8.0, -1), std::nullopt);

    // Shouldn't accept derivatives above 2
    ASSERT_EQ(sut_->evaluate(8.0, 3), std::nullopt);
    ASSERT_EQ(sut_->evaluate(8.0, 100), std::nullopt);
}

TEST_F(CubicOrientationSplineShould, ForceEvaluationPointToBeInsideTheRange) {
    Eigen::Vector3d zeroInitialAngularVelocity(0, 0, 0);
    Eigen::Vector3d zeroFinalAngularVelocity(0, 0, 0);
    Eigen::Vector3d initialAngularVelocity(1, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 1);

    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)),
    };

    std::vector<double> timeInstances = {1.0, 6.0, 11.0, 16.0};

    //  ------ Checking when only initial velocity is zero -------
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, zeroInitialAngularVelocity, finalAngularVelocity, timeInstances)));

    // Checking when evaluation point is negative
    Eigen::Vector3d angularVelocityEvalNeg = sut_->evaluate(-100.0, 1).value();
    Eigen::Vector3d angularAccelerationEvalNeg = sut_->evaluate(-100.0, 2).value();
    Eigen::Vector3d angularAccelerationEvalMin = sut_->evaluate(timeInstances[0], 2).value();
    ASSERT_NEAR(angularVelocityEvalNeg[0], zeroInitialAngularVelocity[0], eps_);
    ASSERT_NEAR(angularVelocityEvalNeg[1], zeroInitialAngularVelocity[1], eps_);
    ASSERT_NEAR(angularVelocityEvalNeg[2], zeroInitialAngularVelocity[2], eps_);

    ASSERT_NEAR(angularAccelerationEvalNeg[0], angularAccelerationEvalMin[0], eps_);
    ASSERT_NEAR(angularAccelerationEvalNeg[1], angularAccelerationEvalMin[1], eps_);
    ASSERT_NEAR(angularAccelerationEvalNeg[2], angularAccelerationEvalMin[2], eps_);

    // Checking when evaluation point is below the minimum
    Eigen::Vector3d angularVelocityEvalBelow = sut_->evaluate(0.0, 1).value();
    Eigen::Vector3d angularAccelerationEvalBelow = sut_->evaluate(0.0, 2).value();
    ASSERT_NEAR(angularVelocityEvalBelow[0], zeroInitialAngularVelocity[0], eps_);
    ASSERT_NEAR(angularVelocityEvalBelow[1], zeroInitialAngularVelocity[1], eps_);
    ASSERT_NEAR(angularVelocityEvalBelow[2], zeroInitialAngularVelocity[2], eps_);

    ASSERT_NEAR(angularAccelerationEvalBelow[0], angularAccelerationEvalMin[0], eps_);
    ASSERT_NEAR(angularAccelerationEvalBelow[1], angularAccelerationEvalMin[1], eps_);
    ASSERT_NEAR(angularAccelerationEvalBelow[2], angularAccelerationEvalMin[2], eps_);

    // Checking when evaluation point is above maximum
    ASSERT_EQ(sut_->evaluate(16.1, 1), std::nullopt);
    ASSERT_EQ(sut_->evaluate(16.1, 2), std::nullopt);

    //  ------ Checking when only final velocity is zero -------
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, initialAngularVelocity, zeroFinalAngularVelocity, timeInstances)));

    // Checking when evaluation point is negative
    ASSERT_EQ(sut_->evaluate(-100.0, 1), std::nullopt);
    ASSERT_EQ(sut_->evaluate(-100.0, 2), std::nullopt);

    // Checking when evaluation point is below the minimum
    ASSERT_EQ(sut_->evaluate(0.0, 1), std::nullopt);
    ASSERT_EQ(sut_->evaluate(0.0, 2), std::nullopt);

    // Checking when evaluation point is above maximum
    Eigen::Vector3d angularVelocityEvalAbove = sut_->evaluate(16.1, 1).value();
    Eigen::Vector3d angularAccelerationEvalAbove = sut_->evaluate(16.1, 2).value();
    Eigen::Vector3d angularAccelerationEvalMax =
        sut_->evaluate(timeInstances[timeInstances.size() - 1], 2).value();
    ASSERT_NEAR(angularVelocityEvalAbove[0], zeroFinalAngularVelocity[0], eps_);
    ASSERT_NEAR(angularVelocityEvalAbove[1], zeroFinalAngularVelocity[1], eps_);
    ASSERT_NEAR(angularVelocityEvalAbove[2], zeroFinalAngularVelocity[2], eps_);

    ASSERT_NEAR(angularAccelerationEvalAbove[0], angularAccelerationEvalMax[0], eps_);
    ASSERT_NEAR(angularAccelerationEvalAbove[1], angularAccelerationEvalMax[1], eps_);
    ASSERT_NEAR(angularAccelerationEvalAbove[2], angularAccelerationEvalMax[2], eps_);
}

TEST_F(CubicOrientationSplineShould, InterpolateBetweenEulerZXZOrientations) {
    std::vector<double> timeInstances = {0.0, 45.0, 90.0, 135.0};

    std::vector<Orientation> orientationPoints = {
        Orientation(EulerZXZ({0, 0.03, 0})),
        Orientation(EulerZXZ({0.03, 0.28, 0.02})),
        Orientation(EulerZXZ({0.14, 0.14, 0.06})),
        Orientation(EulerZXZ({0.06, 0.28, 0.03}))};

    Eigen::Vector3d initialAngularVelocity(0, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 0);

    // Create the interpolation object
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, initialAngularVelocity, finalAngularVelocity, timeInstances)));

    // Orientation at the start
    EulerZXZ orientationPointsStartEval =
        sut_->evaluateOrientation(0.0).value().getEulerZXZ();
    EulerZXZ orientationPointsStart = orientationPoints[0].getEulerZXZ();

    ASSERT_NEAR(orientationPointsStartEval[0], orientationPointsStart[0], eps_);
    ASSERT_NEAR(orientationPointsStartEval[1], orientationPointsStart[1], eps_);
    ASSERT_NEAR(orientationPointsStartEval[2], orientationPointsStart[2], eps_);

    // Angular velocity at the start
    Eigen::Vector3d initialAngularVelocityEval = sut_->evaluate(0.0, 1).value();
    ASSERT_NEAR(initialAngularVelocityEval[0], initialAngularVelocity[0], eps_);
    ASSERT_NEAR(initialAngularVelocityEval[1], initialAngularVelocity[1], eps_);
    ASSERT_NEAR(initialAngularVelocityEval[2], initialAngularVelocity[2], eps_);

    // Orientation at second time instance
    EulerZXZ orientationPointsEval2 = sut_->evaluateOrientation(45.0).value().getEulerZXZ();
    EulerZXZ orientationPoints2 = orientationPoints[1].getEulerZXZ();

    ASSERT_NEAR(orientationPointsEval2[0], orientationPoints2[0], eps_);
    ASSERT_NEAR(orientationPointsEval2[1], orientationPoints2[1], eps_);
    ASSERT_NEAR(orientationPointsEval2[2], orientationPoints2[2], eps_);

    // Orientation at third time instance
    EulerZXZ orientationPointsEval3 = sut_->evaluateOrientation(90.0).value().getEulerZXZ();
    EulerZXZ orientationPoints3 = orientationPoints[2].getEulerZXZ();

    ASSERT_NEAR(orientationPointsEval3[0], orientationPoints3[0], eps_);
    ASSERT_NEAR(orientationPointsEval3[1], orientationPoints3[1], eps_);
    ASSERT_NEAR(orientationPointsEval3[2], orientationPoints3[2], eps_);

    // Orientation at the end
    EulerZXZ orientationPointsEndEval =
        sut_->evaluateOrientation(135.0).value().getEulerZXZ();
    EulerZXZ orientationPointsEnd = orientationPoints[3].getEulerZXZ();

    ASSERT_NEAR(orientationPointsEndEval[0], orientationPointsEnd[0], eps_);
    ASSERT_NEAR(orientationPointsEndEval[1], orientationPointsEnd[1], eps_);
    ASSERT_NEAR(orientationPointsEndEval[2], orientationPointsEnd[2], eps_);

    // Angular velocity at the end
    Eigen::Vector3d finalAngularVelocityEval = sut_->evaluate(135.0, 1).value();
    ASSERT_NEAR(finalAngularVelocityEval[0], finalAngularVelocity[0], eps_);
    ASSERT_NEAR(finalAngularVelocityEval[1], finalAngularVelocity[1], eps_);
    ASSERT_NEAR(finalAngularVelocityEval[2], finalAngularVelocity[2], eps_);
}

TEST_F(CubicOrientationSplineShould, InterpolateBetweenCardanXYZOrientations) {
    std::vector<double> timeInstances = {0.0, 10.0, 20.0, 30.0};

    std::vector<Orientation> orientationPoints = {
        Orientation(CardanXYZ({0, 0, 0.2})),
        Orientation(CardanXYZ({0.3, 0, 0.5})),
        Orientation(CardanXYZ({0, 0.3, 0})),
        Orientation(CardanXYZ({1.0, 0, 0.8}))};

    Eigen::Vector3d initialAngularVelocity(0, 0, -0.7);
    Eigen::Vector3d finalAngularVelocity(0, 0, 0);

    // Create the interpolation object
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, initialAngularVelocity, finalAngularVelocity, timeInstances)));

    // Orientation at the start
    CardanXYZ orientationPointsStartEval =
        sut_->evaluateOrientation(0.0).value().getCardanXYZ();
    CardanXYZ orientationPointsStart = orientationPoints[0].getCardanXYZ();

    ASSERT_NEAR(orientationPointsStartEval[0], orientationPointsStart[0], eps_);
    ASSERT_NEAR(orientationPointsStartEval[1], orientationPointsStart[1], eps_);
    ASSERT_NEAR(orientationPointsStartEval[2], orientationPointsStart[2], eps_);

    // Angular velocity at the start
    Eigen::Vector3d initialAngularVelocityEval = sut_->evaluate(0.0, 1).value();
    ASSERT_NEAR(initialAngularVelocityEval[0], initialAngularVelocity[0], eps_);
    ASSERT_NEAR(initialAngularVelocityEval[1], initialAngularVelocity[1], eps_);
    ASSERT_NEAR(initialAngularVelocityEval[2], initialAngularVelocity[2], eps_);

    // Orientation at second time instance
    CardanXYZ orientationPointsEval2 =
        sut_->evaluateOrientation(10.0).value().getCardanXYZ();
    CardanXYZ orientationPoints2 = orientationPoints[1].getCardanXYZ();

    ASSERT_NEAR(orientationPointsEval2[0], orientationPoints2[0], eps_);
    ASSERT_NEAR(orientationPointsEval2[1], orientationPoints2[1], eps_);
    ASSERT_NEAR(orientationPointsEval2[2], orientationPoints2[2], eps_);

    // Orientation at third time instance
    CardanXYZ orientationPointsEval3 =
        sut_->evaluateOrientation(20.0).value().getCardanXYZ();
    CardanXYZ orientationPoints3 = orientationPoints[2].getCardanXYZ();

    ASSERT_NEAR(orientationPointsEval3[0], orientationPoints3[0], eps_);
    ASSERT_NEAR(orientationPointsEval3[1], orientationPoints3[1], eps_);
    ASSERT_NEAR(orientationPointsEval3[2], orientationPoints3[2], eps_);

    // Orientation at the end
    CardanXYZ orientationPointsEndEval =
        sut_->evaluateOrientation(30.0).value().getCardanXYZ();
    CardanXYZ orientationPointsEnd = orientationPoints[3].getCardanXYZ();

    ASSERT_NEAR(orientationPointsEndEval[0], orientationPointsEnd[0], eps_);
    ASSERT_NEAR(orientationPointsEndEval[1], orientationPointsEnd[1], eps_);
    ASSERT_NEAR(orientationPointsEndEval[2], orientationPointsEnd[2], eps_);

    // Angular velocity at the end
    Eigen::Vector3d finalAngularVelocityEval = sut_->evaluate(30.0, 1).value();
    ASSERT_NEAR(finalAngularVelocityEval[0], finalAngularVelocity[0], eps_);
    ASSERT_NEAR(finalAngularVelocityEval[1], finalAngularVelocity[1], eps_);
    ASSERT_NEAR(finalAngularVelocityEval[2], finalAngularVelocity[2], eps_);
}

TEST_F(CubicOrientationSplineShould, InterpolateBetweenAngleAxisOrientations) {
    std::vector<double> timeInstances = {0.0, 90.0, 180.0, 270.0};

    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::AngleAxisd(0.2, Eigen::Vector3d(0.5, 0, 1).normalized())),
        Orientation(Eigen::AngleAxisd(0.5, Eigen::Vector3d(0.25, 1, 0).normalized())),
        Orientation(Eigen::AngleAxisd(1.0, Eigen::Vector3d(0.3, 0, 1).normalized())),
        Orientation(Eigen::AngleAxisd(0.8, Eigen::Vector3d(0.7, 0.2, 1).normalized()))};

    Eigen::Vector3d initialAngularVelocity(0, 0, -0.2);
    Eigen::Vector3d finalAngularVelocity(0, 0.2, 0);

    // Create the interpolation object
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, initialAngularVelocity, finalAngularVelocity, timeInstances)));

    // Orientation at the start
    Eigen::AngleAxisd orientationPointsStartEval =
        sut_->evaluateOrientation(0.0).value().getAngleAxis();
    Eigen::Vector3d axStartEval = orientationPointsStartEval.axis();
    double angStartEval = orientationPointsStartEval.angle();

    Eigen::AngleAxisd orientationPointsStart = orientationPoints[0].getAngleAxis();
    Eigen::Vector3d axStart = orientationPointsStart.axis();
    double angStart = orientationPointsStart.angle();

    ASSERT_NEAR(axStartEval[0], axStart[0], eps_);
    ASSERT_NEAR(axStartEval[1], axStart[1], eps_);
    ASSERT_NEAR(axStartEval[2], axStart[2], eps_);
    ASSERT_NEAR(angStartEval, angStart, eps_);

    // Angular velocity at the start
    Eigen::Vector3d initialAngularVelocityEval = sut_->evaluate(0.0, 1).value();
    ASSERT_NEAR(initialAngularVelocityEval[0], initialAngularVelocity[0], eps_);
    ASSERT_NEAR(initialAngularVelocityEval[1], initialAngularVelocity[1], eps_);
    ASSERT_NEAR(initialAngularVelocityEval[2], initialAngularVelocity[2], eps_);

    // Orientation at second time instance
    Eigen::AngleAxisd orientationPointsEval2 =
        sut_->evaluateOrientation(90.0).value().getAngleAxis();
    Eigen::Vector3d axEval2 = orientationPointsEval2.axis();
    double angEval2 = orientationPointsEval2.angle();

    Eigen::AngleAxisd orientationPoints2 = orientationPoints[1].getAngleAxis();
    Eigen::Vector3d ax2 = orientationPoints2.axis();
    double ang2 = orientationPoints2.angle();

    ASSERT_NEAR(axEval2[0], ax2[0], eps_);
    ASSERT_NEAR(axEval2[1], ax2[1], eps_);
    ASSERT_NEAR(axEval2[2], ax2[2], eps_);
    ASSERT_NEAR(angEval2, ang2, eps_);

    // Orientation at third time instance
    Eigen::AngleAxisd orientationPointsEval3 =
        sut_->evaluateOrientation(180.0).value().getAngleAxis();
    Eigen::Vector3d axEval3 = orientationPointsEval3.axis();
    double angEval3 = orientationPointsEval3.angle();

    Eigen::AngleAxisd orientationPoints3 = orientationPoints[2].getAngleAxis();
    Eigen::Vector3d ax3 = orientationPoints3.axis();
    double ang3 = orientationPoints3.angle();

    ASSERT_NEAR(axEval3[0], ax3[0], eps_);
    ASSERT_NEAR(axEval3[1], ax3[1], eps_);
    ASSERT_NEAR(axEval3[2], ax3[2], eps_);
    ASSERT_NEAR(angEval3, ang3, eps_);

    // Orientation at the end
    Eigen::AngleAxisd orientationPointsEndEval =
        sut_->evaluateOrientation(270.0).value().getAngleAxis();
    Eigen::Vector3d axEndEval = orientationPointsEndEval.axis();
    double angEndEval = orientationPointsEndEval.angle();

    Eigen::AngleAxisd orientationPointsEnd = orientationPoints[3].getAngleAxis();
    Eigen::Vector3d axEnd = orientationPointsEnd.axis();
    double angEnd = orientationPointsEnd.angle();

    ASSERT_NEAR(axEndEval[0], axEnd[0], eps_);
    ASSERT_NEAR(axEndEval[1], axEnd[1], eps_);
    ASSERT_NEAR(axEndEval[2], axEnd[2], eps_);
    ASSERT_NEAR(angEndEval, angEnd, eps_);

    // Angular velocity at the end
    Eigen::Vector3d finalAngularVelocityEval = sut_->evaluate(270.0, 1).value();
    ASSERT_NEAR(finalAngularVelocityEval[0], finalAngularVelocity[0], eps_);
    ASSERT_NEAR(finalAngularVelocityEval[1], finalAngularVelocity[1], eps_);
    ASSERT_NEAR(finalAngularVelocityEval[2], finalAngularVelocity[2], eps_);
}

TEST_F(CubicOrientationSplineShould, ComputeLongTrajectoryAccurately) {
    Eigen::Vector3d initialAngularVelocity(0.8, 0.0, 0.2);
    Eigen::Vector3d finalAngularVelocity(0.0, 0.7, 1.0);

    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(
            0.4424523389628854, -0.722945740390823, -0.5284688155067477, -0.04802182039377063)),
        Orientation(Eigen::Quaterniond(
            -0.38682065281097566, 0.3457778383036485, 0.6383778698830466, -0.5685781954494302)),
        Orientation(Eigen::Quaterniond(
            0.09606638809543289, -0.5963645276055518, 0.490839826046745, 0.6278509890559271)),
        Orientation(Eigen::Quaterniond(
            0.36469568540920805, -0.33789577126525105, -0.8390705966122345, -0.22087109069717106)),
        Orientation(Eigen::Quaterniond(
            0.29891582906444936, 0.7179205275004454, -0.29027037358512, 0.5576580973562115)),
        Orientation(Eigen::Quaterniond(
            0.32079760874470775, -0.21058371632377448, 0.14374393135336236, -0.9121847810844167)),
        Orientation(Eigen::Quaterniond(
            0.6306912077045483, -0.36260792817409104, 0.38442492103506726, 0.56829707991213)),
        Orientation(Eigen::Quaterniond(
            0.5561075370534756, 0.02317431113687185, -0.5808535116063304, 0.5939836332679912)),
        Orientation(Eigen::Quaterniond(
            -0.6545248038649207, -0.30577456995679664, -0.172417160436229, 0.6696054930184122)),
        Orientation(Eigen::Quaterniond(
            0.2931366882390998, 0.11746076153512042, 0.7719200760763123, 0.5517365745155404)),
    };

    std::vector<double> timeInstances = {0.0, 0.5, 1.0, 1.3, 1.5, 1.8, 2.3, 3.0, 4.5, 5.0};
    std::size_t numInputs = timeInstances.size();

    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, initialAngularVelocity, finalAngularVelocity, timeInstances)));

    // Check interpolation is correct at each input instance
    for (std::size_t i = 0; i < numInputs; i++) {
        // Orientation
        Orientation orientationPointsEval =
            sut_->evaluateOrientation(timeInstances[i]).value();
        Orientation orientationPointsExpected = orientationPoints[i];
        ASSERT_TRUE(
            areAlmostEqual(orientationPointsEval, orientationPointsExpected, eps_));

        if (i == 0) {
            // Angular velocity at the start
            Eigen::Vector3d initialAngularVelocityEval = sut_->evaluate(0.0, 1).value();
            ASSERT_NEAR(initialAngularVelocityEval[0], initialAngularVelocity[0], eps_);
            ASSERT_NEAR(initialAngularVelocityEval[1], initialAngularVelocity[1], eps_);
            ASSERT_NEAR(initialAngularVelocityEval[2], initialAngularVelocity[2], eps_);
        }

        if (i == numInputs - 1) {
            // Angular velocity at the end
            Eigen::Vector3d finalAngularVelocityEval =
                sut_->evaluate(timeInstances[numInputs - 1], 1).value();
            ASSERT_NEAR(finalAngularVelocityEval[0], finalAngularVelocity[0], eps_);
            ASSERT_NEAR(finalAngularVelocityEval[1], finalAngularVelocity[1], eps_);
            ASSERT_NEAR(finalAngularVelocityEval[2], finalAngularVelocity[2], eps_);
        }
    }
}

TEST_F(CubicOrientationSplineShould, BeAbleToBeCombined) {
    // Whole trajectory we want to evaluate
    std::vector<double> trajectoryTimes = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0,
                                           1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0};
    std::size_t numEvalInstances = trajectoryTimes.size();
    std::vector<Orientation> taskTrajectory(numEvalInstances);
    std::vector<Eigen::Vector3d> taskTrajectoryAngularRate(numEvalInstances);

    // Create interpolation which covers first half of desired trajectory
    Eigen::Vector3d initialAngularVelocity1(0, 0, 0);
    Eigen::Vector3d finalAngularVelocity1(0, 0, 0);

    std::vector<Orientation> orientationPoints1 = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
    };
    std::vector<double> timeInstances1 = {0.0, 0.5, 1.0};

    // Making first interpolation
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints1, initialAngularVelocity1, finalAngularVelocity1, timeInstances1)));

    // Evaluating first half of trajectory using first interpolation
    for (std::size_t i = 0; i < 10; i++) {
        taskTrajectory[i] = sut_->evaluateOrientation(trajectoryTimes[i]).value();
        taskTrajectoryAngularRate[i] = sut_->evaluate(trajectoryTimes[i], 1).value();
    }

    // Create interpolation which covers second half of desired trajectory
    // Using second last instance in the first interpolation to start the next one
    Eigen::Vector3d currentVelocity = sut_->evaluate(trajectoryTimes[9], 1).value();
    Orientation currentOrientation = sut_->evaluateOrientation(trajectoryTimes[9]).value();

    std::vector<Orientation> orientationPoints2 = {
        currentOrientation,
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
    };

    std::vector<double> timeInstances2 = {0.9, 1.5, 2.0};

    // New endpoint angular rates
    Eigen::Vector3d initialAngularVelocity2(currentVelocity);
    Eigen::Vector3d finalAngularVelocity2(0, 0, 0);

    // Making second interpolation
    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints2, initialAngularVelocity2, finalAngularVelocity2, timeInstances2)));

    // Evaluating second half of trajectory using second interpolation
    for (std::size_t i = 10; i < numEvalInstances; i++) {
        taskTrajectory[i] = sut_->evaluateOrientation(trajectoryTimes[i]).value();
        taskTrajectoryAngularRate[i] = sut_->evaluate(trajectoryTimes[i], 1).value();
    }

    // Check that the point in the trajectory that would have been the last point in the first
    // trajectory was instead evaluated by the second one
    ASSERT_FALSE(
        areAlmostEqual(taskTrajectory[10], orientationPoints1[2], eps_));  // at time 1.0

    // Check continuous - first part of second trajectory matches what was evaluated using first
    Orientation orientationChangeInstanceEval = sut_->evaluateOrientation(0.9).value();
    // task trajectory at t=0.9 found using 1st interp.
    ASSERT_TRUE(areAlmostEqual(orientationChangeInstanceEval, taskTrajectory[9], eps_));

    Eigen::Vector3d angularRateChangeInstanceEval = sut_->evaluate(0.9, 1).value();
    // task trajectory angular rate at t=0.9 found using 1st interp.
    Eigen::Vector3d angularRateChangeInstance = taskTrajectoryAngularRate[9];
    ASSERT_NEAR(angularRateChangeInstanceEval[0], angularRateChangeInstance[0], eps_);
    ASSERT_NEAR(angularRateChangeInstanceEval[1], angularRateChangeInstance[1], eps_);
    ASSERT_NEAR(angularRateChangeInstanceEval[2], angularRateChangeInstance[2], eps_);
}

TEST_F(CubicOrientationSplineShould, ThrowExceptionIfInfiniteVelocity) {
    Eigen::Vector3d initialAngularVelocity(0, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 0);

    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
    };

    std::vector<double> timeInstances = {0.0, 1.0, 1.0};

    double convergenceTol = 1e-12;
    std::size_t maxIter = 1000;

    std::stringstream expectedErrorMessage;
    expectedErrorMessage << "The algorithm took over " << maxIter << " iterations to converge, "
        "suggesting that the interpolation conditions may be unreasonable. These issues could be "
        "due to small time intervals or the combination of large time intervals with high "
        "initialAngularVelocity or finalAngularVelocity.";

    ASSERT_ANY_THROW({
        try {
            sut_.reset(new CubicOrientationSpline(
                orientationPoints,
                initialAngularVelocity,
                finalAngularVelocity,
                timeInstances,
                convergenceTol,
                maxIter));
        } catch (const std::runtime_error& e) {
            // Check the what() message (comment)
            ASSERT_STREQ(expectedErrorMessage.str().c_str(), e.what());
            throw;  // Re-throw to propagate the exception
        }
    });
}

TEST_F(CubicOrientationSplineShould, MatchOriginalImplementationResults) {
    // Check that it matches James McEnnan's implementation's results (James McEnnan wrote
    // the paper that this algorithm was based on).

    // Expected results - found by putting the same input values into the origin C implementation
    std::vector<double> evaluationTimes = {
        0.0, 1.0/3.0, 2.0/3.0, 1.0, 4.0/3.0, 5.0/3.0, 2.0, 7.0/3.0, 8.0/3.0, 3.0};
    std::vector<Orientation> expectedOrientations = {
        Orientation(Eigen::Quaterniond(
            1.000000000000000, 0.000000000000000, 0.000000000000000, 0.000000000000000)),
        Orientation(Eigen::Quaterniond(
            0.929113323161600, 0.335074460517890, -0.139027885163718, 0.071727162073769)),
        Orientation(Eigen::Quaterniond(
            0.467819515624578, 0.844310653125814, -0.232227876209037, 0.119810831440597)),
        Orientation(Eigen::Quaterniond(
            0.000000000000000, 1.000000000000000, 0.000000000000000, 0.000000000000000)),
        Orientation(Eigen::Quaterniond(
            -0.136105144626202, 0.810737889326950, 0.516956515617495, -0.238611452735573)),
        Orientation(Eigen::Quaterniond(
            -0.075242451885673, 0.322947201638637, 0.900424525750149, -0.281565892493435)),
        Orientation(Eigen::Quaterniond(
            -0.000000000000000, 0.000000000000001, 1.000000000000000, -0.000000000000000)),
        Orientation(Eigen::Quaterniond(
            0.020724838306603, -0.068087654258810, 0.815400779390656, 0.574505109971886)),
        Orientation(Eigen::Quaterniond(
            0.008323526327233, -0.027345418786799, 0.299292331432133, 0.953733216013143)),
        Orientation(Eigen::Quaterniond(
            0.000000000000000, 0.000000000000000, 0.000000000000000, 1.000000000000000))
    };

    std::vector<Eigen::Vector3d> expected1stDerivatives = {
        Eigen::Vector3d(0.000000000000000, 0.000000000000000, 0.000000000000000),
        Eigen::Vector3d(3.603899989039259, -1.334924552771291, 0.598730811851923),
        Eigen::Vector3d(3.962293892270830, -1.059559571264024, -0.457604865451444),
        Eigen::Vector3d(1.728541476744382, -1.262883080115868, -2.447830902008596),
        Eigen::Vector3d(0.185071132327977, -1.915420142044018, -3.640641365241616),
        Eigen::Vector3d(-0.972252117674624, -1.166516715693201, -2.909533794196386),
        Eigen::Vector3d(-2.773548054584515, -0.298382175528870, -0.980279899116428),
        Eigen::Vector3d(-4.173884701099883, -0.302465271276515, -0.033702794126412),
        Eigen::Vector3d(-3.254082899830941, -0.301510113558915, 0.093365209663309),
        Eigen::Vector3d(0.000000000000000, 0.000000000000000, 0.000000000000000)
    };

    std::vector<Eigen::Vector3d> expected2ndDerivatives = {
        Eigen::Vector3d(15.392472968049995, -7.690087578980281, 3.967464206834861),
        Eigen::Vector3d(6.142040769645449, -0.703132579053302, -0.672479457293512),
        Eigen::Vector3d(-3.986788532858526, 1.230560304936713, -5.080220911894504),
        Eigen::Vector3d(-5.976770656373885, -3.661825872197508, -7.097672515271523),
        Eigen::Vector3d(-3.504901751665936, 0.301761646761315, -0.534594590441307),
        Eigen::Vector3d(-3.913455839484628, 3.399956752494238, 4.887985444753275),
        Eigen::Vector3d(-7.755363703201605, 1.193528702115348, 3.921119596466254),
        Eigen::Vector3d(-0.682281097494748, -0.517811955926938, 1.415880344048167),
        Eigen::Vector3d(6.224462871017000, 0.593933730853770, -0.190346565045036),
        Eigen::Vector3d(13.302459812369726, 1.248131132464031, -0.379911985327467)
    };


    // Making CubicOrientationSpline object using same inputs that were used to obtain above results
    std::vector<Orientation> orientationPoints = {
        Orientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0)),
        Orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)),
    };

    Eigen::Vector3d initialAngularVelocity(0, 0, 0);
    Eigen::Vector3d finalAngularVelocity(0, 0, 0);

    std::vector<double> timeInstances = {0.0, 1.0, 2.0, 3.0};

    ASSERT_NO_THROW(sut_.reset(new CubicOrientationSpline(
        orientationPoints, initialAngularVelocity, finalAngularVelocity, timeInstances)));

    // Check interpolation is correct at each evaluation instance
    for (std::size_t i = 0; i < evaluationTimes.size(); i++) {
        // Orientation
        Orientation orientationPointsEval =
            sut_->evaluateOrientation(evaluationTimes[i]).value();
        Orientation orientationPointsExpected = expectedOrientations[i];
        ASSERT_TRUE(
            areAlmostEqual(orientationPointsEval, orientationPointsExpected, eps_));

        // Angular rate
        Eigen::Vector3d angularVelocityEval = sut_->evaluate(evaluationTimes[i], 1).value();
        Eigen::Vector3d angularVelocityExpected = expected1stDerivatives[i];
        ASSERT_NEAR(angularVelocityEval[0], angularVelocityExpected[0], eps_);
        ASSERT_NEAR(angularVelocityEval[1], angularVelocityExpected[1], eps_);
        ASSERT_NEAR(angularVelocityEval[2], angularVelocityExpected[2], eps_);

        // Angular acceleration
        Eigen::Vector3d angularAccelerationEval = sut_->evaluate(evaluationTimes[i], 2).value();
        Eigen::Vector3d angularAccelerationExpected = expected2ndDerivatives[i];
        ASSERT_NEAR(angularAccelerationEval[0], angularAccelerationExpected[0], eps_);
        ASSERT_NEAR(angularAccelerationEval[1], angularAccelerationExpected[1], eps_);
        ASSERT_NEAR(angularAccelerationEval[2], angularAccelerationExpected[2], eps_);
    }
}
