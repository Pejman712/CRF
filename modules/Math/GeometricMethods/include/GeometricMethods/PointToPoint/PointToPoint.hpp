/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2022
 *         Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
 */

#pragma once

#include <vector>
#include <optional>

#include "EventLogger/EventLogger.hpp"
#include "GeometricMethods/IGeometricMethods.hpp"

namespace crf::math::geometricmethods {

/**
 * @ingroup group_point_to_point
 * @brief The point to point method is conformed of two parts:
 *
 * A first equation representing a linear velocity part between the points.
 *
 *                        q(x) = a_0 + a_1*x
 *
 * And a cuadratic part that represents the parabolic blends that connect
 * the different linear velocities between the points:
 *
 *                   q(x) = a_0 + a_1*x + a_2*x^2
 *
 * Two constructors are given depending on the desired behaviour. The first
 * constructor takes the points, the ranges and the maximum acceleration. It
 * will try to connect the points at the specified time with the acceleration
 * given. If it's not possible to folllow the path then an exception will be thrown.
 *
 * The second constructor takes the points, the maximum velocity, and the maximum
 * acceleration and connects them trying to achieve the maximum velocity.
 * If the maximum velocity cannot be acheived with the given acceleration,
 * then a lower velocity will be used.
 *
 * @param points0thDerivative refers to the all the points of the 0th
 * derivative that the functions needs to go through.
 * @param ranges must be a set of the same size as points0thDerivative that
 * describes at which value of x the correspondent point is reached.
 * @param max1stDerivative maximum first derivative allowed to connect all the
 * points.
 * @param max2ndDerivative maximum second derivative allowed to generate the
 * parabolic blends to connect the linear parts.
 */
class PointToPoint: public IGeometricMethods {
 public:
    PointToPoint(
        const std::vector<double>& points0thDerivative,
        const std::vector<double>& ranges,
        const double& max2ndDerivative);
    PointToPoint(
        const std::vector<double>& points0thDerivative,
        const double& max1stDerivative, const double& max2ndDerivative);
    ~PointToPoint() override;

    std::optional<double> getRange() const override;
    std::optional<double> evaluate(double evaluationPoint, unsigned int derivative) override;

    double getStartPoint() const;

 private:
    std::vector<double> points0thDerivative_;
    std::vector<double> velocities_;
    std::vector<double> timeIntervals_;
    std::vector<double> accelerationTimes_;
    std::vector<double> totalTime_;

    crf::utility::logger::EventLogger logger_;

    int sign(const double& num);
    double sum(const std::vector<double>& vec) const;
};

}  // namespace crf::math::geometricmethods
