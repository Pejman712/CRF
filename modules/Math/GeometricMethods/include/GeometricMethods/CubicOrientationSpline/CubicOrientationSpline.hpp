/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Chelsea Davidson CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */
#pragma once

#include "EventLogger/EventLogger.hpp"
#include "GeometricMethods/IOrientationMethods.hpp"

#include <cmath>
#include <vector>

namespace crf::math::geometricmethods {
/**
 * @ingroup group_cubic_orientation_spline
 * @brief Constructs a Cubic Orientation Spline object which interpolates between Orientations. The
 *        interpolation passes through these Orientations at specified time instances and begins and
 *        ends with specified angular rates. It is continuous to the second derivative.
 *
 * @param orientationPoints refers to all the Orientations that the interpolation needs to pass
 *        through.
 * @param initialAngularVelocity refers to the angular velocity at the beginning of the
 *        interpolation (the 1st derivative of the first orientation in orientationPoints).
 * @param finalAngularVelocity refers to the angular velocity at the end of the interpolation
 *        (the 1st derivative of the last orientation in orientationPoints).
 * @param timeInstances refers to the time at which the interpolation needs to pass through the
 *        Orientations in orientationPoints.
 * @param convergenceTol refers to the maximum allowable error when checking for the convergence of
 *        the intermediate rates solution. Default: 1e-12.
 * @param maxIter refers to the maximum number of iterations allowed. when solving for the
 *        intermediate rates. Default: 1000.
 */
class CubicOrientationSpline : public IOrientationMethods {
 public:
    CubicOrientationSpline(
        const std::vector<Orientation>& orientationPoints,
        const Eigen::Vector3d& initialAngularVelocity,
        const Eigen::Vector3d& finalAngularVelocity,
        const std::vector<double>& timeInstances,
        const double& convergenceTol = 1e-12,
        const std::size_t& maxIter = 1000);
    CubicOrientationSpline() = delete;

    CubicOrientationSpline(const CubicOrientationSpline&);
    CubicOrientationSpline(CubicOrientationSpline&&);
    ~CubicOrientationSpline() override;
    CubicOrientationSpline& operator=(const CubicOrientationSpline&);

    std::optional<double> getRange() const override;
    std::optional<Orientation> evaluateOrientation(double evaluationPoint) override;
    std::optional<Eigen::Vector3d> evaluate(double evaluationPoint,
        unsigned int derivative) override;

 private:
    std::vector<Orientation> orientationPoints_;
    Eigen::Vector3d initialAngularVelocity_;
    Eigen::Vector3d finalAngularVelocity_;
    std::vector<double> timeInstances_;
    double convergenceTol_;  // Tolerance for checking if rates in getRates has converged
    std::size_t maxIter_;  // Max num of iterations for the getRates algorithm

    const double epsilon_ = 1e-6;  // Defines small change. Used to check angle approaching 0 etc.
    std::size_t numInputs_;

    // Rates coeffient variables
    std::vector<double> bRatesCoeffs_;
    std::vector<double> cRatesCoeffs_;
    std::vector<Eigen::Vector3d> dRatesCoeffs_;

    // Interpolation interval variables
    std::vector<Eigen::Vector3d> eList_;
    std::vector<double> deltaThetaList_;
    std::vector<double> deltaTimeList_;

    // Parameterisations of the interpolation
    std::vector<std::array<Eigen::Vector3d, 4>> coeffsTheta_;
    std::vector<std::array<Eigen::Vector3d, 3>> coeffsDTheta_;
    std::vector<std::array<Eigen::Vector3d, 2>> coeffsDDTheta_;

    crf::utility::logger::EventLogger logger_;

    /**
     * @brief Determines the coefficients of the polynomial curves used to describe
     *        theta and its derivatives over this interval.
     * @post sets the coefficient variables, coeffsTheta_, coeffsDTheta_, coeffsDDTheta_.
     */
    void setPolynomCoeffs();

    /**
     * @brief Solves for the angular velocities (rates) at each input instance such that continuity
     *        and initial/final conditions are satisfied. Create vector of all angular rates
     *        (.incl. initialAngularVelocity_, finalAngularVelocity_)
     * @return A vector containing the angular rate at each input instance.
     * @throws Throws std::runtime_error if rates solution does not converge and interpolation
     *         cannot be defined.
     */
    std::vector<Eigen::Vector3d> getRates();

    /**
     * @brief Determines the coefficients of the intermediate rate matrix equation on page 7 of the
     *        paper. Used by the getRates() function.
     * @param angularRatesPrev the angular rate vectors calculated in the previous interation within
     *        the getRates function.
     * @post sets the variables b, c, d.
     */
    void setRatesCoeffs(const std::vector<Eigen::Vector3d>& angularRatesPrev);

    /**
     * @brief Computes the 'a' coefficient used in the intermediate angular rate matrix equation at
     *        a specific input instance (page 7 of the paper).
     * @param k input instance index.
     * @return the 'a' coefficient at this input instance.
     * @throws Throws std::runtime_error if k is outside of the range over which 'a' is defined.
     */
    double getRatesCoeffA(const std::size_t& k);

    /**
     * @brief Computes the 'b' coefficient used in the intermediate angular rate matrix equation at
     *        a specific input instance (page 7 of the paper).
     * @param k input instance index.
     * @return the 'b' coefficient at this input instance.
     * @throws Throws std::runtime_error if k is outside of the range over which 'b' is defined.
     */
    double getRatesCoeffB(const std::size_t& k);

    /**
     * @brief Computes the 'c' coefficient used in the intermediate angular rate matrix equation at
     *        a specific input instance (page 7 of the paper).
     * @param k input instance index.
     * @return the 'c' coefficient at this input instance.
     * @throws Throws std::runtime_error if k is outside of the range over which 'c' is defined.
     */
    double getRatesCoeffC(const std::size_t& k);

    /**
     * @brief Computes the 'd' coefficient used in the intermediate angular rate matrix equation at
     *        a specific input instance (page 7 of the paper).
     * @param k input instance index.
     * @param angularRatesPrev the angular rate vectors calculated in the previous interation within
     *        the getRates function.
     * @return the 'd' coefficient at this input instance.
     * @throws Throws std::runtime_error if k is outside of the range over which 'd' is defined.
     */
    Eigen::Vector3d getRatesCoeffD(const std::size_t& k,
        const std::vector<Eigen::Vector3d>& angularRatesPrev);

    /**
     * @brief Helper function for the getRates() algorithm which calculates the non-linear component
     *        of the angular rate vector.
     * @param e slew axis between Orientations at this interval.
     * @param deltaTheta slew angle between Orientations at this interval.
     * @param angularRatePrev end angular rate vector of this interval calculated in the
     *        previous interation of the getRates() algorithm.
     * @return non-linear component of the angular rate vector.
     */
    Eigen::Vector3d getNonLinearRateTerm(const Eigen::Vector3d& e, const double& deltaTheta,
        const Eigen::Vector3d& angularRatePrev);

    /**
     * @brief Executes equation 26 from the paper. Typically used transform the coefficient vector
     *        to the angular rate vector.
     * @param e slew axis
     * @param deltaTheta slew angle
     * @param x input vector
     * @return transformed output vector
     */
    Eigen::Vector3d bDotX(const Eigen::Vector3d& e, const double& deltaTheta,
        const Eigen::Vector3d& x);

    /**
     * @brief Executes equation 24 from the paper. Typically used transform the angular rate vector
     *        to the coefficient vector.
     * @param e slew axis
     * @param deltaTheta slew angle
     * @param x input vector
     * @return transformed output vector
     */
    Eigen::Vector3d bInvDotX(const Eigen::Vector3d& e, const double& deltaTheta,
        const Eigen::Vector3d& x);
};

}  // namespace crf::math::geometricmethods
