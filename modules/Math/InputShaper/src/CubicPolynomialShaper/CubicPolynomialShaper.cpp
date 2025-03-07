/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
 */

#include <vector>
#include <optional>

#include "InputShaper/CubicPolynomialShaper/CubicPolynomialShaper.hpp"

namespace crf::math::inputshaper {

CubicPolynomialShaper::CubicPolynomialShaper(
    const double& startingPoint,
    const double& max1stDerivative):
    startingPoint_(startingPoint),
    max1stDerivative_(max1stDerivative),
    reference_(0.0),
    responsivenessFactor_(1.0),
    lastEvaluationPoint_(0.0),
    referenceSet_(false),
    logger_("CubicPolynomialShaper") {
}

CubicPolynomialShaper::CubicPolynomialShaper(const CubicPolynomialShaper& other):
    startingPoint_(other.startingPoint_),
    max1stDerivative_(other.max1stDerivative_),
    reference_(other.reference_.load()),
    responsivenessFactor_(other.responsivenessFactor_.load()),
    lastEvaluationPoint_(other.lastEvaluationPoint_.load()),
    referenceSet_(other.referenceSet_.load()),
    logger_("CubicPolynomialShaper") {
}

CubicPolynomialShaper::CubicPolynomialShaper(CubicPolynomialShaper&& other):
    startingPoint_(std::move(other.startingPoint_)),
    max1stDerivative_(std::move(other.max1stDerivative_)),
    reference_(std::move(other.reference_.load())),
    responsivenessFactor_(std::move(other.responsivenessFactor_).load()),
    lastEvaluationPoint_(std::move(other.lastEvaluationPoint_).load()),
    referenceSet_(std::move(other.referenceSet_).load()),
    logger_("CubicPolynomialShaper") {
}

CubicPolynomialShaper::~CubicPolynomialShaper() {
}

void CubicPolynomialShaper::reset() {
    reference_ = 0.0;
    responsivenessFactor_ = 1.0;
    lastEvaluationPoint_ = 0.0;
    referenceSet_ = false;
}

void CubicPolynomialShaper::setReference(const double& reference) {
    if (reference == reference_) return;
    reference_ = reference;

    if (!referenceSet_) {
        double time = std::fabs(reference_ - startingPoint_)*(3/2)/max1stDerivative_ + 1;
        double endTime = lastEvaluationPoint_ + time*responsivenessFactor_;

        referenceSet_ = true;
        std::scoped_lock<std::mutex> lck(polyMtx_);
        polynomial_ = std::make_unique<crf::math::geometricmethods::CubicPolynomial>(
            std::vector<double>({startingPoint_, reference}), 0, 0,
            std::vector<double>({lastEvaluationPoint_, endTime}));
        return;
    }

    double current0thDer = polynomial_->evaluate(lastEvaluationPoint_, 0).value();
    double current1stDer = polynomial_->evaluate(lastEvaluationPoint_, 1).value();
    double time = std::fabs(reference_ - current0thDer)*(3/2)/max1stDerivative_ + 1;
    double endTime = lastEvaluationPoint_ + time*responsivenessFactor_;

    std::scoped_lock<std::mutex> lck(polyMtx_);
    polynomial_ = std::make_unique<crf::math::geometricmethods::CubicPolynomial>(
        std::vector<double>({current0thDer, reference}), current1stDer, 0,
        std::vector<double>({lastEvaluationPoint_, endTime}));
}

void CubicPolynomialShaper::setResponsivenessFactor(const double& responsivenessFactor) {
    if (responsivenessFactor < 1.0)
        throw std::runtime_error(
            "CubicPolynomialShaper - Responsiveness cannot be less than 1");
    responsivenessFactor_ = responsivenessFactor;
}

double CubicPolynomialShaper::getInputPoint(const double& evaluationPoint) {
    if (lastEvaluationPoint_ > evaluationPoint) {
        logger_->error("Last Point {} vs current {}", lastEvaluationPoint_, evaluationPoint);
        throw std::runtime_error(
            "CubicPolynomialShaper - The evaluation point cannot go backwards");
    }
    lastEvaluationPoint_ = evaluationPoint;
    if (!referenceSet_) return startingPoint_;
    std::scoped_lock<std::mutex> lck(polyMtx_);
    return polynomial_->evaluate(evaluationPoint, 0).value();
}

}  // namespace crf::math::inputshaper
