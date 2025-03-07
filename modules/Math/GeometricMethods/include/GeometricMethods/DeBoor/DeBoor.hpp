/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2021
 *         Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *
 *  ================================================================================================================
 */

#pragma once

#include <vector>
#include <optional>

#include "EventLogger/EventLogger.hpp"
#include "GeometricMethods/IGeometricMethods.hpp"

namespace crf {
namespace math {
namespace geometricmethods {

/**
 * @ingroup group_de_boor
 * @brief The order of a NURBS curve defines the number of nearby control points that influence
 *        any given point on the curve. The curve is represented mathematically by a polynomial
 *        of degree one less than the order of the curve.
 * 
 * @param The control points determine the shape of the curve.
 * @param The knot vector is a sequence of parameter values that determines where and how the
 *        control points affect the NURBS curve.
 */
class DeBoor: public IGeometricMethods {
 public:
    DeBoor(unsigned int degree, std::vector<double> knots, std::vector<double> controlPoints);
    ~DeBoor() override;

    std::optional<double> getRange() const override;
    std::optional<double> evaluate(double evaluationPoint, unsigned int derivative) override;

 private:
    crf::utility::logger::EventLogger logger_;
    int degree_;
    std::vector<double> knots_;
    std::vector<double> controlPoints_;
    int nKnots_, nControlPoints_;
    std::vector<std::vector<double>> Pk_, a_;
};

}  // namespace geometricmethods
}  // namespace math
}  // namespace crf
