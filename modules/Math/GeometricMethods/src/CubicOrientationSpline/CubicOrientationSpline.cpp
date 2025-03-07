/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Chelsea Davidson CERN BE/CEM/MRO 2024
 *
 * This code was based on James McEnnan's paper, Quaternion Cubic Spline, which was published on May 28, 2003. The
 * original C code and documentation can be found at https://qspline.sourceforge.net/
 *
 *  ================================================================================================================
 */

#include "GeometricMethods/CubicOrientationSpline/CubicOrientationSpline.hpp"

namespace crf::math::geometricmethods {

CubicOrientationSpline::CubicOrientationSpline(
    const std::vector<Orientation>& orientationPoints,
    const Eigen::Vector3d& initialAngularVelocity,
    const Eigen::Vector3d& finalAngularVelocity,
    const std::vector<double>& timeInstances,
    const double& convergenceTol,
    const std::size_t& maxIter) :
    orientationPoints_(orientationPoints),
    initialAngularVelocity_(initialAngularVelocity),
    finalAngularVelocity_(finalAngularVelocity),
    timeInstances_(timeInstances),
    convergenceTol_(convergenceTol),
    maxIter_(maxIter),
    numInputs_(timeInstances.size()),
    bRatesCoeffs_(numInputs_, 0.0),
    cRatesCoeffs_(numInputs_, 0.0),
    dRatesCoeffs_(numInputs_, Eigen::Vector3d::Zero()),
    eList_(numInputs_, Eigen::Vector3d::Zero()),
    deltaThetaList_(numInputs_, 0.0),
    deltaTimeList_(numInputs_, 0.0),
    coeffsTheta_(),
    coeffsDTheta_(),
    coeffsDDTheta_(),
    logger_("CubicOrientationSpline") {
    // Check for dimension mismatch
    if (timeInstances_.size() != orientationPoints_.size()) {
        throw std::runtime_error("The number of input time instances does not match the one of "
            "orientations. The size of timeInstances was " + std::to_string(timeInstances_.size()) +
            " and the size of orientationPoints was " + std::to_string(orientationPoints_.size()));
    }

    // Check has min number of input instances required to create interpolation
    if (numInputs_ < 2) {
        throw std::runtime_error("Require at least 2 input instances to generate a Cubic "
            "Orientation Spline. Only " + std::to_string(numInputs_) + " were given");
    }

    // Calculate the value of the interval variables at each interval
    for (std::size_t i = 0; i <= numInputs_ - 2; i++) {
        deltaTimeList_[i+1] = timeInstances_[i+1] - timeInstances_[i];

        // Check for times in timeInstances_ being in decreasing order
        if (deltaTimeList_[i+1] < 0) {
            throw std::runtime_error("Input timeInstances must be in ascending order");
        }

        // Calculating slew angle and axis - Equation 17 in documentation
        // Find relative rotation from start to end orientation
        Eigen::AngleAxisd relativeRotation = multiply(invert(orientationPoints[i]),
            orientationPoints[i+1]).getAngleAxis();

        // Angle of rotation from start to end orientation
        deltaThetaList_[i+1] = relativeRotation.angle();

        // Axis of rotation - axis about which the angle rotates
        eList_[i+1] = relativeRotation.axis();
    }

    // Set the coeffs that parameterize the functions of theta and its derivatives for each interval
    setPolynomCoeffs();
}

CubicOrientationSpline::CubicOrientationSpline(const CubicOrientationSpline& other):
    orientationPoints_(other.orientationPoints_),
    initialAngularVelocity_(other.initialAngularVelocity_),
    finalAngularVelocity_(other.finalAngularVelocity_),
    timeInstances_(other.timeInstances_),
    convergenceTol_(other.convergenceTol_),
    maxIter_(other.maxIter_),
    numInputs_(other.numInputs_),
    bRatesCoeffs_(other.bRatesCoeffs_),
    cRatesCoeffs_(other.cRatesCoeffs_),
    dRatesCoeffs_(other.dRatesCoeffs_),
    eList_(other.eList_),
    deltaThetaList_(other.deltaThetaList_),
    deltaTimeList_(other.deltaTimeList_),
    coeffsTheta_(other.coeffsTheta_),
    coeffsDTheta_(other.coeffsDTheta_),
    coeffsDDTheta_(other.coeffsDDTheta_),
    logger_("CubicOrientationSpline") {
}

CubicOrientationSpline::CubicOrientationSpline(CubicOrientationSpline&& other):
    orientationPoints_(std::move(other.orientationPoints_)),
    initialAngularVelocity_(std::move(other.initialAngularVelocity_)),
    finalAngularVelocity_(std::move(other.finalAngularVelocity_)),
    timeInstances_(std::move(other.timeInstances_)),
    convergenceTol_(std::move(other.convergenceTol_)),
    maxIter_(std::move(other.maxIter_)),
    numInputs_(std::move(other.numInputs_)),
    bRatesCoeffs_(std::move(other.bRatesCoeffs_)),
    cRatesCoeffs_(std::move(other.cRatesCoeffs_)),
    dRatesCoeffs_(std::move(other.dRatesCoeffs_)),
    eList_(std::move(other.eList_)),
    deltaThetaList_(std::move(other.deltaThetaList_)),
    deltaTimeList_(std::move(other.deltaTimeList_)),
    coeffsTheta_(std::move(other.coeffsTheta_)),
    coeffsDTheta_(std::move(other.coeffsDTheta_)),
    coeffsDDTheta_(std::move(other.coeffsDDTheta_)),
    logger_("CubicOrientationSpline") {
}

CubicOrientationSpline::~CubicOrientationSpline() {
}

CubicOrientationSpline& CubicOrientationSpline::operator=(const CubicOrientationSpline& other) {
    orientationPoints_ = other.orientationPoints_;
    initialAngularVelocity_ = other.initialAngularVelocity_;
    finalAngularVelocity_ = other.finalAngularVelocity_;
    timeInstances_ = other.timeInstances_;
    convergenceTol_ = other.convergenceTol_;
    maxIter_ = other.maxIter_;
    numInputs_ = other.numInputs_,
    bRatesCoeffs_ = other.bRatesCoeffs_,
    cRatesCoeffs_ = other.cRatesCoeffs_,
    dRatesCoeffs_ = other.dRatesCoeffs_,
    eList_ = other.eList_,
    deltaThetaList_ = other.deltaThetaList_,
    deltaTimeList_ = other.deltaTimeList_,
    coeffsTheta_ = other.coeffsTheta_,
    coeffsDTheta_ = other.coeffsDTheta_,
    coeffsDDTheta_ = other.coeffsDDTheta_,
    logger_ = other.logger_;
    return *this;
}

std::optional<double> CubicOrientationSpline::getRange() const {
    return timeInstances_[numInputs_ - 1] - timeInstances_[0];
}

std::optional<Orientation> CubicOrientationSpline::evaluateOrientation(double evaluationPoint) {
    // Forcing the evaluationPoint to be within the range
    if (evaluationPoint < timeInstances_[0] && initialAngularVelocity_.isZero(epsilon_)) {
        evaluationPoint = timeInstances_[0];
    } else if (evaluationPoint > timeInstances_[numInputs_ - 1] &&
        finalAngularVelocity_.isZero(epsilon_)) {
        evaluationPoint = timeInstances_[numInputs_ - 1];
    } else if (
        evaluationPoint < timeInstances_[0] || evaluationPoint > timeInstances_[numInputs_ - 1 ]) {
        logger_->error("The evaluation point, t = {}, exceeded the range: [{},{}]. Cannot evaluate "
            "orientation at this time since the endpoint angular velocity is non-zero.",
            evaluationPoint, timeInstances_[0], timeInstances_[numInputs_ - 1]);
        return std::nullopt;
    }

    // Finding which interval this evaluationPoint belongs to
    // - Find first element in timeInstances_ that's greater than evaluationPoint
    auto it = std::upper_bound(timeInstances_.begin(), timeInstances_.end(), evaluationPoint);

    // intervalIndx is index of element before the first one greater than evaluationPoint
    std::size_t intervalIndx = std::distance(timeInstances_.begin(), it - 1);

    // Include the last point in the last polynomial function because there are only numInputs_ - 1
    // polynomials for numInputs_ points
    if (intervalIndx == numInputs_ - 1 &&
        abs(evaluationPoint - timeInstances_[numInputs_ - 1]) <= epsilon_) {
        intervalIndx -= 1;
    }

    // Time variables
    double deltaTime = deltaTimeList_[intervalIndx + 1];  // Interval var defined at end of interval
    double x = (evaluationPoint - timeInstances_[intervalIndx]) / deltaTime;

    // Corresponding initial orientation for the interval of interest
    Orientation startOrientation = orientationPoints_[intervalIndx];

    // Finding the corresponding coefficients of the polynomial defined over that interval
    // - Naming matches Equation 11 in the documentation where number here is superscript in doc.
    std::array<Eigen::Vector3d, 4> a3 = coeffsTheta_[intervalIndx];

    // Cubic polynomial representation of vector function theta - Equation 11 in documentation
    Eigen::Vector3d thetaVect = a3[0] * std::pow((x - 1.0), 3) + a3[1] * x * std::pow((x - 1.0), 2)
        + a3[2] * std::pow(x, 2) * (x - 1.0) + a3[3] * std::pow(x, 3);

    // Solve for deltaOrientation (delta q in doc., change in orientation from start to end orient.)
    double theta = thetaVect.norm();  // Value of theta(t) - Equation 2 in documentation
    Eigen::Vector3d u(Eigen::Vector3d::Zero());
    if (theta > epsilon_) {  // Singularity as theta approaches 0 - if theta 0, u remains [0,0,0]
        u = thetaVect / theta;  // Equation 2 in documentation
    }

    // Form [w,x,y,z]
    Orientation deltaOrientation = Orientation(Eigen::Quaterniond(cos(theta / 2.0),
        u[0] * sin(theta / 2.0), u[1] * sin(theta / 2.0), u[2] * sin(theta / 2.0)));

    // Solving for interpolated orientation - Equation 1 in documentation
    return multiply(startOrientation, deltaOrientation);
}

std::optional<Eigen::Vector3d> CubicOrientationSpline::evaluate(double evaluationPoint,
    unsigned int derivative) {
    // Checking the derivative can be calculated
    if (derivative == 0) {
        logger_->error("0th derivative cannot be computed using this function. Use "
            "CubicOrientationSpline::evaluateOrientation(double evaluationPoint) instead.");
        return std::nullopt;
    } else if (derivative != 1 && derivative != 2) {
        logger_->error("Can only compute 1st or 2nd derivative. Cannot compute derivative {}.",
            derivative);
        return std::nullopt;
    }

    // Forcing the evaluationPoint to be within the range
    if (evaluationPoint < timeInstances_[0] && initialAngularVelocity_.isZero(epsilon_)) {
        evaluationPoint = timeInstances_[0];
    } else if (evaluationPoint > timeInstances_[numInputs_ - 1] &&
        finalAngularVelocity_.isZero(epsilon_)) {
        evaluationPoint = timeInstances_[numInputs_ - 1];
    } else if (
        evaluationPoint < timeInstances_[0] || evaluationPoint > timeInstances_[numInputs_ - 1 ]) {
        logger_->error("The evaluation point, t = {}, exceeded the range: [{},{}]. Cannot evaluate "
            "derivative at this time since the endpoint angular velocity is non-zero.",
            evaluationPoint, timeInstances_[0], timeInstances_[numInputs_ - 1]);
        return std::nullopt;
    }

    // Finding which interval this evaluationPoint belongs to
    // - Find first element in timeInstances_ that's greater than evaluationPoint
    auto it = std::upper_bound(timeInstances_.begin(), timeInstances_.end(), evaluationPoint);

    // intervalIndx is index of element before the first one greater than evaluationPoint
    std::size_t intervalIndx = std::distance(timeInstances_.begin(), it - 1);

    // Include the last point in the last polynomial function because there are only numInputs_ - 1
    // polynomials for numInputs_ points
    if (intervalIndx == numInputs_ - 1 &&
        abs(evaluationPoint - timeInstances_[numInputs_ - 1]) <= epsilon_) {
        intervalIndx -= 1;
    }

    // Time variables
    double deltaTime = deltaTimeList_[intervalIndx + 1];
    double x = (evaluationPoint - timeInstances_[intervalIndx]) / deltaTime;

    // Finding the corresponding coefficients of the polynomial defined over that interval
    // - Naming matches Equation 11-13 in documentation where number here is superscript in doc.
    std::array<Eigen::Vector3d, 4> a3 = coeffsTheta_[intervalIndx];
    std::array<Eigen::Vector3d, 3> a2 = coeffsDTheta_[intervalIndx];
    std::array<Eigen::Vector3d, 2> a1;

    // Cubic polynomial representation of the vector function theta and dtheta
    // - Equation 11-12 in documentation
    Eigen::Vector3d thetaVect = a3[0] * std::pow((x - 1.0), 3) + a3[1] * x * std::pow((x - 1.0), 2)
        + a3[2] * std::pow(x, 2) * (x - 1.0) + a3[3] * std::pow(x, 3);
    Eigen::Vector3d dThetaVect = a2[0] * std::pow((x - 1.0), 2) + a2[1] * x * (x - 1.0)
        + a2[2] * std::pow(x, 2);
    Eigen::Vector3d ddThetaVect;

    // Variables only needed to find the second derivative
    if (derivative == 2) {
        a1 = coeffsDDTheta_[intervalIndx];
        ddThetaVect = a1[0] * (x - 1.0) + a1[1] * x;  // Equation 13 in documentation
    }

    double theta = thetaVect.norm();  // Value of theta(t) - Equation 2 in documentation
    if (theta <= epsilon_) {  // Singularity as theta approaches 0
        if (derivative == 1) {
            return dThetaVect;  // Equation 9 in documentation
        }
        return ddThetaVect;  // Derivative = 2, Equation 10 in documentation
    }
    // Solving for the angular rate/ angular acceleration
    Eigen::Vector3d u = thetaVect / theta;  // Equation 2 in documentation
    Eigen::Vector3d v = u.cross(dThetaVect) / theta;  // Equation 7 in documentation
    double dTheta = u.dot(dThetaVect);  // Equation 5 in documentation

    // Shorthand for sin and inverse cos
    double c1 = 1.0 - cos(theta);
    double s = sin(theta);

    Eigen::Vector3d evaluatedAngularVelocity = u * dTheta + s * v.cross(u) - c1 * v;  // Equation 3

    if (derivative == 1) {
        return evaluatedAngularVelocity;
    }

    // Derivative = 2
    Eigen::Vector3d dv = (thetaVect.cross(ddThetaVect) - 2.0 * thetaVect.dot(dThetaVect) * v) /
        std::pow(theta, 2);  // Derivative of v (Equation 8 in documentation)
    double ddTheta = (v.cross(u)).dot(dThetaVect) + u.dot(ddThetaVect);  // Equation 6 in doc.

    return u * ddTheta + s * dv.cross(u) - c1 * dv + dTheta * v.cross(u) +
        evaluatedAngularVelocity.cross(u * dTheta - v);  // Equation 4 in documentation
}

void CubicOrientationSpline::setPolynomCoeffs() {
    std::vector<Eigen::Vector3d> angularVelocities = getRates();

    // Setting the coefficient vectors that are used describe each of the polynomials for theta
    // and its derivatives for each interval
    for (std::size_t i = 0; i <= numInputs_ - 2; i++) {
        // Initial state
        Eigen::Vector3d startAngularVelocity = angularVelocities[i];

        // Interval variables
        Eigen::Vector3d eInterval = eList_[i + 1];  // Interval variables defined at end of interval
        double deltaThetaInterval = deltaThetaList_[i + 1];
        double deltaTimeInterval = deltaTimeList_[i + 1];

        // Final state
        Eigen::Vector3d endAngularVelocity = angularVelocities[i + 1];

        // Setting coefficient values based on Equation 16 in documentation
        // Note: number corresponds to superscript in doc. and index is the subscript minus 1 due
        //       to 0 indexing (ie a3[1] is (a_2)^3 in doc.)
        std::array<Eigen::Vector3d, 4> a3;
        std::array<Eigen::Vector3d, 3> a2;
        std::array<Eigen::Vector3d, 2> a1;

        // Variable to make equations easier
        Eigen::Vector3d bInvDotEndAngularVelocity = bInvDotX(eInterval, deltaThetaInterval,
            endAngularVelocity);

        // Theta equation coefficients
        a3[0] = Eigen::Vector3d::Zero();  // as stated in documentation
        a3[1] = deltaTimeInterval * startAngularVelocity;
        a3[2] = deltaTimeInterval * bInvDotEndAngularVelocity -
            3.0 * eInterval * deltaThetaInterval;
        a3[3] = eInterval * deltaThetaInterval;

        // dTheta equation coefficients
        a2[0] = startAngularVelocity;
        a2[1] = 2.0 * startAngularVelocity + 2.0 * bInvDotEndAngularVelocity -
            (6.0 * eInterval * deltaThetaInterval) / deltaTimeInterval;
        a2[2] = bInvDotEndAngularVelocity;

        // ddTheta equation coefficients
        a1[0] = (4.0 * startAngularVelocity + 2.0 * bInvDotEndAngularVelocity -
            6.0 * eInterval * deltaThetaInterval / deltaTimeInterval) / deltaTimeInterval;
        a1[1] = (2.0 * startAngularVelocity + 4.0 * bInvDotEndAngularVelocity -
            6.0 * eInterval * deltaThetaInterval / deltaTimeInterval) / deltaTimeInterval;

        // Store the coefficients that describe the polynomial at this interval
        coeffsTheta_.push_back(a3);  // Interval coeffs found in order so push_back can be used.
        coeffsDTheta_.push_back(a2);
        coeffsDDTheta_.push_back(a1);
    }
}

std::vector<Eigen::Vector3d> CubicOrientationSpline::getRates() {
    // Initialise variables. Assume all angular rates are 0 to start with
    std::vector<Eigen::Vector3d> angularRatesPrev(numInputs_, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> angularRates(numInputs_, Eigen::Vector3d::Zero());

    // Setting the initial and final conditions
    angularRates[0] = initialAngularVelocity_;
    angularRates[numInputs_ - 1] = finalAngularVelocity_;

    // Don't need to solve for intermediate rates if there are only 2 input instances
    if (numInputs_ == 2) {
        return angularRates;
    }

    // Setup iteration variables
    std::size_t itr = 0;
    bool converged = false;

    // Iterate until convergence or number of iterations exceeds maxIter_
    while (!converged && itr < maxIter_) {
        itr++;
        // Set coefficients of angular rate matrix equation - Equation 22-26 in documentation
        setRatesCoeffs(angularRatesPrev);

        // Calculate intermediate rates using back substitution
        // Finding the last intermediate rate (numInputs_ - 1) but considering 0 based
        angularRates[numInputs_ - 2] = dRatesCoeffs_[numInputs_ - 2] /
            bRatesCoeffs_[numInputs_ - 2];

        // Calculate intermediate rates iterating backward because eqn uses result at higher k value
        for (std::size_t k = numInputs_ - 3; k >= 1; k--) {
            Eigen::Vector3d bInvDotAngularRateNext = bInvDotX(eList_[k + 1], deltaThetaList_[k + 1],
                angularRates[k + 1]);
            angularRates[k] = (dRatesCoeffs_[k] - (cRatesCoeffs_[k] * bInvDotAngularRateNext)) /
                bRatesCoeffs_[k];
        }

        // Checking for convergence - note start and end angular velocities don't change
        double err = 0.0;
        for (std::size_t k = 1; k <= numInputs_ - 2; k++) {
            Eigen::Vector3d diff = angularRates[k] - angularRatesPrev[k];
            err += diff.squaredNorm();
        }
        double l2Error = std::sqrt(err);
        if (l2Error <= convergenceTol_) {
            converged = true;
        }

        // Storing this iteration's angularRate values as the next iteration's previous values.
        angularRatesPrev = angularRates;
    }

    // If reached max iterations, rates didn't converge and hence interpolation cannot be computed
    if (itr == maxIter_) {
        throw std::runtime_error("The algorithm took over " + std::to_string(maxIter_) +
            " iterations to converge, suggesting that the interpolation conditions may be "
            "unreasonable. These issues could be due to small time intervals or the combination of "
            "large time intervals with high initialAngularVelocity or finalAngularVelocity.");
    }
    logger_->info("Algorithm to find intermediate angular rates converged in {} iterations.", itr);

    return angularRates;
}

void CubicOrientationSpline::setRatesCoeffs(const std::vector<Eigen::Vector3d>& angularRatesPrev) {
    // Create an 'a' coeff vector since it won't be used outside of the function but needs to be
    // stored in same way as other coeff variables for eqns to match up
    std::vector<double> a(numInputs_, 0.0);

    // Reset rates coeffs
    std::fill(bRatesCoeffs_.begin(), bRatesCoeffs_.end(), 0.0);
    std::fill(cRatesCoeffs_.begin(), cRatesCoeffs_.end(), 0.0);
    std::fill(dRatesCoeffs_.begin(), dRatesCoeffs_.end(), Eigen::Vector3d::Zero());

    // Finding the coeffs used at each row (k value) of the matrix when the matrix is in
    // the tridiagonal matrix form
    for (std::size_t k = 1; k <= numInputs_ - 2; k++) {
        a[k] = getRatesCoeffA(k);
        bRatesCoeffs_[k] = getRatesCoeffB(k);
        cRatesCoeffs_[k] = getRatesCoeffC(k);
        dRatesCoeffs_[k] = getRatesCoeffD(k, angularRatesPrev);
    }

    // Finding coeffs at each k input instance when the matrix is in upper triangular matrix form
    // Note: first row in matrix (k=1 in C++, k=2 in doc.) stays same and only coeffs b and d change
    for (std::size_t k = 2; k <= numInputs_ - 2; k++) {
        bRatesCoeffs_[k] = bRatesCoeffs_[k] - (cRatesCoeffs_[k - 1] * a[k]) / bRatesCoeffs_[k - 1];

        Eigen::Vector3d bDotDKPrev = bDotX(eList_[k], deltaThetaList_[k], dRatesCoeffs_[k - 1]);
        dRatesCoeffs_[k] = dRatesCoeffs_[k] - (a[k] / bRatesCoeffs_[k - 1]) * bDotDKPrev;
    }
}

double CubicOrientationSpline::getRatesCoeffA(const std::size_t& k) {
    // Checking 'a' is defined at this k value
    if (k <= 0 || k >= numInputs_ - 1) {
        throw std::runtime_error("Rates coeff 'a' is only defined within the range of k: [1, " +
            std::to_string(numInputs_ - 2) + "]. Function called with k = " + std::to_string(k));
    }
    return 2.0 / deltaTimeList_[k];
}

double CubicOrientationSpline::getRatesCoeffB(const std::size_t& k) {
    // Checking 'b' is defined at this k value
    if (k <= 0 || k >= numInputs_ - 1) {
        throw std::runtime_error("Rates coeff 'b' is only defined within the range of k: [1, " +
            std::to_string(numInputs_ - 2) + "]. Function called with k = " + std::to_string(k));
    }
    return (4.0 / deltaTimeList_[k]) + (4.0 / deltaTimeList_[k + 1]);
}

double CubicOrientationSpline::getRatesCoeffC(const std::size_t& k) {
    // Checking 'c' is defined at this k value
    if (k <= 0 || k >= numInputs_ - 1) {
        throw std::runtime_error("Rates coeff 'c' is only defined within the range of k: [1, " +
            std::to_string(numInputs_ - 2) + "]. Function called with k = " + std::to_string(k));
    }
    return 2.0 / deltaTimeList_[k + 1];
}

Eigen::Vector3d CubicOrientationSpline::getRatesCoeffD(
    const std::size_t& k,
    const std::vector<Eigen::Vector3d>& angularRatesPrev) {
    // Checking 'd' is defined at this k value
    if (k <= 0 || k >= numInputs_ - 1) {
        throw std::runtime_error("Rates coeff 'd' is only defined within the range of k: [1, " +
            std::to_string(numInputs_ - 2) + "]. Function called with k = " + std::to_string(k));
    }

    // Finding the terms of the d coefficient equation - Equation 26 in documentation
    Eigen::Vector3d d0 = (6.0 * eList_[k] * deltaThetaList_[k]) /
        (deltaTimeList_[k] * deltaTimeList_[k]);
    Eigen::Vector3d d1 = (6.0 * eList_[k + 1] * deltaThetaList_[k + 1]) /
        (deltaTimeList_[k + 1] * deltaTimeList_[k + 1]);
    Eigen::Vector3d d2 = getNonLinearRateTerm(eList_[k], deltaThetaList_[k], angularRatesPrev[k]);
    // Only have the d3 term for the 2nd and 2nd last 'd' rate variable equations
    Eigen::Vector3d d3 = Eigen::Vector3d::Zero();

    // Calculating d3
    if (k == 1) {  // k = 2 in eqns in documentation (C++ is 0 based)
        d3 = getRatesCoeffA(k) * bDotX(eList_[k], deltaThetaList_[k], initialAngularVelocity_);
    } else if (k == numInputs_ - 2) {  // k = N-1 in eqns in documentation (C++ is 0 based)
        d3 = getRatesCoeffC(k) * bInvDotX(eList_[k + 1], deltaThetaList_[k + 1],
            finalAngularVelocity_);
    }

    return d0 + d1 - d2 - d3;
}

Eigen::Vector3d CubicOrientationSpline::getNonLinearRateTerm(
    const Eigen::Vector3d& e,
    const double& deltaTheta,
    const Eigen::Vector3d& angularRatePrev) {
    if (deltaTheta <= epsilon_) {  // singularity when deltaTheta approaches 0
        return Eigen::Vector3d::Zero();
    }
    // Terms to simplify the equations
    double s = sin(deltaTheta);
    double c1 = 1.0 - cos(deltaTheta);
    double angularRateSqrd = angularRatePrev.dot(angularRatePrev);  // sum of squares
    double eDotAngularRate = e.dot(angularRatePrev);
    double eDotAngularRateSqrd = eDotAngularRate * eDotAngularRate;
    Eigen::Vector3d eCrossAngularRate = e.cross(angularRatePrev);

    // Terms in the equation - Equation 21 in documentation
    double r0 = (deltaTheta - s) / (2.0 * c1);
    double r1 = (deltaTheta * s - 2.0 * c1) / (deltaTheta * c1);
    // double r2 = 0 as written in documentation

    return r0 * (angularRateSqrd - eDotAngularRateSqrd) * e + r1 * eDotAngularRate *
        eCrossAngularRate.cross(e);
}

Eigen::Vector3d CubicOrientationSpline::bDotX(const Eigen::Vector3d& e, const double& deltaTheta,
    const Eigen::Vector3d& x) {
    if (deltaTheta <= epsilon_) {  // Singularity as deltaTheta approaches 0
        return x;  // B reduces to the identity matrix when deltaTheta goes to 0
    }
    // Term to simplify equations
    Eigen::Vector3d eCrossX = e.cross(x);

    // Finding terms in the B dot x equation - Equation 27 in documentation
    Eigen::Vector3d b0 = x.dot(e) * e;
    Eigen::Vector3d b1 = (sin(deltaTheta) / deltaTheta) * eCrossX.cross(e);
    Eigen::Vector3d b2 = ((1.0 - cos(deltaTheta)) / deltaTheta) * eCrossX;

    return b0 + b1 - b2;
}

Eigen::Vector3d CubicOrientationSpline::bInvDotX(const Eigen::Vector3d& e, const double& deltaTheta,
    const Eigen::Vector3d& x) {
    if (deltaTheta <= epsilon_) {  // singularity as deltaTheta approaches 0
        return x;  // B reduces to identity matrix when deltaTheta goes to 0 (hence so does BInv)
    }
    // Terms to simplify equations
    double c1 = 1.0 - cos(deltaTheta);
    Eigen::Vector3d eCrossX = e.cross(x);

    // Finding the terms in the B^-1 dot x equation - Equation 18 in documentation
    Eigen::Vector3d b0 = e.dot(x) * e;
    Eigen::Vector3d b1 = ((deltaTheta * sin(deltaTheta)) / (2.0 * c1)) * eCrossX.cross(e);
    Eigen::Vector3d b2 = 0.5 * deltaTheta * eCrossX;

    return b0 + b1 + b2;
}

}  // namespace crf::math::geometricmethods
