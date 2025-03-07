/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
 */

#pragma once

#include <vector>
#include <optional>

#include <Eigen/Sparse>

#include <gsl/gsl_multiroots.h>

#include "EventLogger/EventLogger.hpp"
#include "GeometricMethods/IGeometricMethods.hpp"

namespace crf::math::geometricmethods {

/**
 * @ingroup group_cubic_polynomial
 * @brief The cubic polinomial is an equation of the form:
 *
 *          q(x) = a_0 + a_1*x + a_2*x^2 + a_3*x^3
 *
 * As there are four incognitas, four conditions are needed to define
 * it's parameters. Two constructors are provided to generate the necessary
 * equations. The first one is a handy tool to create one cubic polinomial
 * with the neccessary restrictions that calls the second construtor. The
 * second constructor accepts a set of points and a set of ranges and
 * connects them with cubic polinomials.
 *
 * @param points0thDerivative refers to all the points of the 0th
 * derivative that the functions needs to go through.
 * @param start1stDerivative refers to the value of the 1st derivative in
 * the first point of points0thderivative.
 * @param end1stDerivative refers to the value of the 1st derivative in
 * the last point of points0thderivative.
 * @param ranges must be a set of the same size as points0thDerivative that
 * describes at which value of x the correspondent point is reached.
 */
class CubicPolynomial: public IGeometricMethods {
 public:
    CubicPolynomial() = delete;
    CubicPolynomial(
        const double& start0thDerivative, const double& end0thDerivative,
        const double& start1stDerivative, const double& end1stDerivative, const double& range);
    CubicPolynomial(
        const std::vector<double>& points0thDerivative, const double& start1stDerivative,
        const double& end1stDerivative, const std::vector<double>& ranges);
    CubicPolynomial(
        const double& start0thDerivative, const double& end0thDerivative,
        const double& start1stDerivative, const double& end1stDerivative,
        const double& max1stDerivative, const double& max2ndDerivative,
        const double& tolerance, const std::chrono::milliseconds& timeout);
    CubicPolynomial(const CubicPolynomial&);
    CubicPolynomial(CubicPolynomial&&);
    ~CubicPolynomial() override = default;
    CubicPolynomial& operator=(const CubicPolynomial&);

    std::optional<double> getRange() const override;
    std::optional<double> evaluate(double evaluationPoint, unsigned int derivative) override;

 private:
    std::vector<double> points0thDerivative_;
    double start1stDerivative_;
    double end1stDerivative_;
    std::vector<double> ranges_;

    std::vector<double> a_n_;
    uint64_t tripletsCounter_;

    crf::utility::logger::EventLogger logger_;

    /**
     * @brief This constant is needed to make sure values are not too
     * small. If they are too small they can produce problems in the
     * calculation of the matrix and create segfaults.
     * (jplayang)
     */
    const double EigenSparseLUMargin_ = 0.00000001;  // 1e-7

    void insertEquation(
        std::vector<Eigen::Triplet<double>>& triplets, const int& fromRow,  // NOLINT
        const int& fromCol, const std::vector<double>& eq);
};

}  // namespace crf::math::geometricmethods
