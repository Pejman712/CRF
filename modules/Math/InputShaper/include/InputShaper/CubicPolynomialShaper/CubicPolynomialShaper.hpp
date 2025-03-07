/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
 */

#pragma once

#include <atomic>
#include <mutex>
#include <memory>

#include "GeometricMethods/CubicPolynomial/CubicPolynomial.hpp"

#include "EventLogger/EventLogger.hpp"
#include "InputShaper/IInputShaper.hpp"

namespace crf::math::inputshaper {

/**
 * @ingroup group_cubic_polynomial_shaper
 * @brief Implementation of IInputShaper.
 * This implemementation uses cubic polynomials to shape the reference
 * into an smooth sign.
 *
 * @param startingPoint Initial point of the system when it's static.
 * @param max1stDerivative Value of the maximum first derivative allowed to the
 * resulting signal after the shaping
 * @{
 */

class CubicPolynomialShaper: public IInputShaper {
 public:
    CubicPolynomialShaper(const double& startingPoint, const double& max1stDerivative);
    CubicPolynomialShaper(const CubicPolynomialShaper& other);
    CubicPolynomialShaper(CubicPolynomialShaper&& other);
    ~CubicPolynomialShaper() override;

    void reset() override;
    void setReference(const double& reference) override;
    void setResponsivenessFactor(const double& responsiveness) override;
    double getInputPoint(const double& evaluationPoint) override;

 private:
    double startingPoint_;
    double max1stDerivative_;
    std::atomic<double> reference_;
    std::atomic<double> responsivenessFactor_;
    std::atomic<double> lastEvaluationPoint_;
    std::atomic<bool> referenceSet_;

    std::unique_ptr<crf::math::geometricmethods::CubicPolynomial> polynomial_;
    std::mutex polyMtx_;  // CubicPolynomial is not thread safe

    crf::utility::logger::EventLogger logger_;
};
/**@}*/

}  // namespace crf::math::inputshaper
