/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include <vector>

#include "InputShaper/CubicPolynomialShaper/CubicPolynomialShaper.hpp"

using crf::math::inputshaper::CubicPolynomialShaper;

class CubicPolynomialShaperShould: public ::testing::Test {
 protected:
    CubicPolynomialShaperShould(): logger_("CubicPolynomialShaperShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~CubicPolynomialShaperShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<CubicPolynomialShaper> sut_;
};

TEST_F(CubicPolynomialShaperShould, MatchPositionsAtTheExtremesOfTheRange) {
    double start0thDer = 0;
    double max1stDer = 1;
    sut_.reset(new CubicPolynomialShaper(
        start0thDer,
        max1stDer));

    sut_->setReference(1);

    ASSERT_NEAR(sut_->getInputPoint(0), 0, 0.000001);
    ASSERT_NEAR(sut_->getInputPoint(2), 1, 0.000001);
}

TEST_F(CubicPolynomialShaperShould, ThrowIfThePointGoesBack) {
    double start0thDer = 0;
    double max1stDer = 1;
    sut_.reset(new CubicPolynomialShaper(
        start0thDer,
        max1stDer));

    sut_->setReference(1);

    ASSERT_NEAR(sut_->getInputPoint(0), 0, 0.000001);
    ASSERT_NEAR(sut_->getInputPoint(2), 1, 0.000001);
    ASSERT_THROW(sut_->getInputPoint(0), std::runtime_error);
}

TEST_F(CubicPolynomialShaperShould, BeAbleToGoNegative) {
    double start0thDer = 0;
    double max1stDer = 1;
    sut_.reset(new CubicPolynomialShaper(
        start0thDer,
        max1stDer));

    sut_->setReference(-1);

    ASSERT_NEAR(sut_->getInputPoint(0), 0, 0.000001);
    ASSERT_NEAR(sut_->getInputPoint(2), -1, 0.000001);
}

TEST_F(CubicPolynomialShaperShould, BeAbleToChangeReferences) {
    double start0thDer = 0;
    double max1stDer = 1;
    sut_.reset(new CubicPolynomialShaper(
        start0thDer,
        max1stDer));

    sut_->setReference(-1);

    ASSERT_NEAR(sut_->getInputPoint(0), 0, 0.000001);
    ASSERT_NEAR(sut_->getInputPoint(2), -1, 0.000001);

    sut_->setReference(1);

    ASSERT_NEAR(sut_->getInputPoint(5), 1, 0.000001);
}

TEST_F(CubicPolynomialShaperShould, ChangeResponsivenessFactor) {
    double start0thDer = 0;
    double max1stDer = 1;
    sut_.reset(new CubicPolynomialShaper(
        start0thDer,
        max1stDer));

    sut_->setReference(-1);

    ASSERT_NEAR(sut_->getInputPoint(0), 0, 0.000001);
    ASSERT_NEAR(sut_->getInputPoint(2), -1, 0.000001);

    sut_->setResponsivenessFactor(2);
    sut_->setReference(1);

    ASSERT_NEAR(sut_->getInputPoint(8), 1, 0.000001);
}

TEST_F(CubicPolynomialShaperShould, NotWorkIfResponsivenessFactorIsUnderOne) {
    double start0thDer = 0;
    double max1stDer = 1;
    sut_.reset(new CubicPolynomialShaper(
        start0thDer,
        max1stDer));

    ASSERT_THROW(sut_->setResponsivenessFactor(0.5), std::runtime_error);
}

TEST_F(CubicPolynomialShaperShould, NotWorkIfTimeIsNegative) {
    double start0thDer = 0;
    double max1stDer = 1;
    sut_.reset(new CubicPolynomialShaper(
        start0thDer,
        max1stDer));

    ASSERT_THROW(sut_->getInputPoint(-1), std::runtime_error);
}
