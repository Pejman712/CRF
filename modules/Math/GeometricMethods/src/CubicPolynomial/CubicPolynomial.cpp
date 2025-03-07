/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
 */

#include <vector>
#include <optional>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <nlopt.hpp>

#include "GeometricMethods/CubicPolynomial/CubicPolynomial.hpp"

namespace crf::math::geometricmethods {

// Declare functions and variables so that they are only visible in this translation unit
namespace {

struct PolynomialParams {
    double y0;
    double yf;
    double yd0;
    double ydf;
    double ydmax;
    double yddmax;
    double t0;
};

}  // anonymous namespace

static double objectiveFunction(
    const std::vector<double> &x, std::vector<double> &grad, void *data);  // NOLINT
static double pointAtT0(
    const std::vector<double> &x, std::vector<double> &grad, void *data);  // NOLINT
static double pointAtTf(
    const std::vector<double> &x, std::vector<double> &grad, void *data);  // NOLINT
static double derivativeAtT0(
    const std::vector<double> &x, std::vector<double> &grad, void *data);  // NOLINT
static double derivativeAtTf(
    const std::vector<double> &x, std::vector<double> &grad, void *data);  // NOLINT
static double maxFirstDerivative(
    const std::vector<double> &x, std::vector<double> &grad, void *data);  // NOLINT
static double secondDerivativeAtT0(
    const std::vector<double> &x, std::vector<double> &grad, void *data);  // NOLINT
static double secondDerivativeAtTf(
    const std::vector<double> &x, std::vector<double> &grad, void *data);  // NOLINT

CubicPolynomial::CubicPolynomial(const double& start0thDerivative, const double& end0thDerivative,
    const double& start1stDerivative, const double& end1stDerivative, const double& range):
    CubicPolynomial({start0thDerivative, end0thDerivative}, start1stDerivative, end1stDerivative,
        {0, range}) {
}

CubicPolynomial::CubicPolynomial(const std::vector<double>& points0thDerivative,
    const double& start1stDerivative, const double& end1stDerivative,
    const std::vector<double>& ranges):
    points0thDerivative_(points0thDerivative),
    start1stDerivative_(start1stDerivative),
    end1stDerivative_(end1stDerivative),
    ranges_(ranges),
    tripletsCounter_(0),
    logger_("CubicPolynomial") {
    if (points0thDerivative_.size() != ranges_.size()) {
        throw std::runtime_error("The size of the vector of points and ranges don't match");
    }

    for (uint64_t i = 0; i < ranges_.size()-1; i++) {
        if (ranges_[i] > ranges_[i+1]) {
            throw std::runtime_error("Range cannot decrease through the duration of the function");
        }
        if (std::fabs(points0thDerivative_[i]-points0thDerivative_[i+1]) < EigenSparseLUMargin_) {
            if (std::fabs(ranges_[i] - ranges_[i+1]) < EigenSparseLUMargin_) {
                // Both the points and the ranges are lower than the margin. So we can erase both.
                ranges_.erase(ranges_.begin() + i);
                points0thDerivative_.erase(points0thDerivative_.begin() + i);
                i--;
                continue;
            }
        }
        if (ranges_[i] - EigenSparseLUMargin_ < ranges_[i+1] &&
            ranges_[i] + EigenSparseLUMargin_ > ranges_[i+1]) {
            throw std::runtime_error("Impossible to perform a Cubic Polynomial between two "
                "points with difference in range close to 0");
        }
    }

    logger_->debug("CTor - from {} at range {}, to {} at range {}", points0thDerivative_.front(),
        ranges_.front(), points0thDerivative_.back(), ranges_.back());

    Eigen::SparseMatrix<double> A(3, 4);
    // N of variables -> 4*4 + 8*3 * viaPoints
    std::vector<Eigen::Triplet<double>> triplets(16 + (ranges_.size()-2)*24);

    Eigen::VectorXd B(3);
    B << points0thDerivative_[0], points0thDerivative_[1], start1stDerivative_;

    // Initial Equations
    insertEquation(triplets, 0, 0,
        {1, ranges_[0], ranges_[0]*ranges_[0], ranges_[0]*ranges_[0]*ranges_[0]});
    insertEquation(triplets, 1, 0,
        {1, ranges_[1], ranges_[1]*ranges_[1], ranges_[1]*ranges_[1]*ranges_[1]});
    insertEquation(triplets, 2, 0,
        {0, 1, 2*ranges_[0], 3*ranges_[0]*ranges_[0]});

    // Iterate for every via point
    for (uint64_t n = 2; n < ranges_.size(); n++) {
        A.conservativeResize(A.rows() + 4, A.cols() + 4);
        B.conservativeResize(B.rows() + 4, B.cols());

        // qdn-1(tn-1) - qdn(tn-1) = 0
        insertEquation(triplets, A.rows()-4, A.cols()-8,
            {0, 1, 2*ranges_[n-1], 3*ranges_[n-1]*ranges_[n-1], 0, -1, -2*ranges_[n-1], -3*ranges_[n-1]*ranges_[n-1]});  // NOLINT
        B(B.rows()-4, 0) = 0;

        // qddn(tn-1) - qddn-1(tn-1) = 0
        insertEquation(triplets, A.rows()-3, A.cols()-8,
            {0, 0, 2, 6*ranges_[n-1], 0, 0, -2, -6*ranges_[n-1]});
        B(B.rows()-3, 0) = 0;

        // qn(tn-1) = P(tn-1)
        insertEquation(triplets, A.rows()-2, A.cols()-4,
            {1, ranges_[n-1], ranges_[n-1]*ranges_[n-1], ranges_[n-1]*ranges_[n-1]*ranges_[n-1]});
        B(B.rows()-2, 0) = points0thDerivative_[n-1];

        // qn(tn) = P(tn)
        insertEquation(triplets, A.rows()-1, A.cols()-4,
            {1, ranges_[n], ranges_[n]*ranges_[n], ranges_[n]*ranges_[n]*ranges_[n]});
        B(B.rows()-1, 0) = points0thDerivative_[n];
    }

    // Final Equation
    B.conservativeResize(B.rows() + 1, B.cols());
    B(B.rows()-1, 0) = end1stDerivative_;

    A.conservativeResize(A.rows() + 1, A.cols());
    insertEquation(triplets, A.rows()-1, A.cols()-4,
        {0, 1, 2*ranges_.back(), 3*ranges_.back()*ranges_.back()});

    // Solve Matrix
    A.setFromTriplets(triplets.begin(), triplets.end());
    A.makeCompressed();
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(A);
    Eigen::VectorXd X = solver.solve(B);

    a_n_ = std::vector<double>(4*(ranges_.size()-1));
    for (size_t i = 0; i < 4*(ranges_.size()-1); i++) {
        a_n_[i] = X(i);
    }
}

CubicPolynomial::CubicPolynomial(
    const double& start0thDerivative, const double& end0thDerivative,
    const double& start1stDerivative, const double& end1stDerivative,
    const double& max1stDerivative, const double& max2ndDerivative,
    const double& tolerance, const std::chrono::milliseconds& timeout):
    points0thDerivative_({start0thDerivative, end0thDerivative}),
    start1stDerivative_(start1stDerivative),
    end1stDerivative_(end1stDerivative),
    logger_("CubicPolynomial") {
    logger_->debug("CTor - With non-linear constraints");

    // Load parameters
    PolynomialParams params;
    params.y0 = start0thDerivative;
    params.yf = end0thDerivative;
    params.yd0 = start1stDerivative;
    params.ydf = end1stDerivative;
    params.ydmax = max1stDerivative;
    params.yddmax = max2ndDerivative;
    params.t0 = 0;

    // Initial guess for a, b, c, d, tf
    std::vector<double> x(5);
    x[0] = 1;
    x[1] = 1;
    x[2] = params.yd0;
    x[3] = params.y0;
    x[4] = params.t0 + 3;

    nlopt::opt opt(nlopt::GN_ISRES, 5);  // Using Sequential Quadratic Programming method

    // Equations to meet start and end pos and vel
    opt.set_min_objective(objectiveFunction, &params);  // We don't have an objective function
    opt.add_equality_constraint(pointAtT0, &params, 1e-4);
    opt.add_equality_constraint(pointAtTf, &params, 1e-4);
    opt.add_equality_constraint(derivativeAtT0, &params, 1e-8);
    opt.add_equality_constraint(derivativeAtTf, &params, 1e-8);

    // Equations to respect max vel and acc
    opt.add_inequality_constraint(maxFirstDerivative, &params, 1e-4);
    if (max2ndDerivative > 0) {
        opt.add_inequality_constraint(secondDerivativeAtT0, &params, 1e-4);
        opt.add_inequality_constraint(secondDerivativeAtTf, &params, 1e-4);
    }

    // Set optimization bounds for a, b, c, d, tf
    std::vector<double> lb({-1e4, -1e4, -1e4, -1e4, params.t0});
    std::vector<double> ub({1e4, 1e4, 1e4, 1e4, 1e4});
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    // Set stopping criteria
    opt.set_xtol_abs(tolerance);
    opt.set_maxtime(timeout.count()/1e3);

    // Optimize
    double minf;
    nlopt::result result;
    auto start = std::chrono::high_resolution_clock::now();
    result = opt.optimize(x, minf);
    auto end = std::chrono::high_resolution_clock::now();

    // Output results
    if (result < 0) {
        throw std::runtime_error(
            "CubicPolynomial - CTor - Optimization problem failed, result: " + result);
    }

    logger_->info("Solution found in {}ms",
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count());

    a_n_ = std::vector<double>(4);
    a_n_[0] = x[3];
    a_n_[1] = x[2];
    a_n_[2] = x[1];
    a_n_[3] = x[0];
    ranges_ = std::vector<double>({params.t0, x[4]});
}

CubicPolynomial::CubicPolynomial(const CubicPolynomial& other):
    points0thDerivative_(other.points0thDerivative_),
    start1stDerivative_(other.start1stDerivative_),
    end1stDerivative_(other.end1stDerivative_),
    ranges_(other.ranges_),
    a_n_(other.a_n_),
    tripletsCounter_(other.tripletsCounter_),
    logger_("CubicPolynomial") {
}

CubicPolynomial::CubicPolynomial(CubicPolynomial&& other):
    points0thDerivative_(std::move(other.points0thDerivative_)),
    start1stDerivative_(std::move(other.start1stDerivative_)),
    end1stDerivative_(std::move(other.end1stDerivative_)),
    ranges_(std::move(other.ranges_)),
    a_n_(std::move(other.a_n_)),
    tripletsCounter_(std::move(other.tripletsCounter_)),
    logger_("CubicPolynomial") {
}

CubicPolynomial& CubicPolynomial::operator=(const CubicPolynomial& other) {
    points0thDerivative_ = other.points0thDerivative_;
    start1stDerivative_ = other.start1stDerivative_;
    end1stDerivative_ = other.end1stDerivative_;
    ranges_ = other.ranges_;
    a_n_ = other.a_n_;
    tripletsCounter_ = other.tripletsCounter_;
    logger_ = other.logger_;
    return *this;
}

std::optional<double> CubicPolynomial::getRange() const {
    return ranges_[ranges_.size()-1];
}

std::optional<double> CubicPolynomial::evaluate(double evaluationPoint, unsigned int derivative) {
    if (evaluationPoint < ranges_[0]) {
        if (derivative == 0) return points0thDerivative_[0] + start1stDerivative_*(evaluationPoint-ranges_[0]);  // NOLINT
        if (derivative == 1) return start1stDerivative_;
        if (derivative == 2) return 0;
        if (derivative == 3) return 0;
    }
    if (evaluationPoint > ranges_.back()) {
        if (derivative == 0) return points0thDerivative_.back() +end1stDerivative_*(evaluationPoint-ranges_.back());  // NOLINT
        if (derivative == 1) return end1stDerivative_;
        if (derivative == 2) return 0;
        if (derivative == 3) return 0;
    }
    int equationNumber = 0;
    while (evaluationPoint > ranges_[equationNumber]) {
        equationNumber++;
    }
    if (equationNumber != 0) equationNumber--;
    double a0 = a_n_[(equationNumber*4)];
    double a1 = a_n_[(equationNumber*4)+1];
    double a2 = a_n_[(equationNumber*4)+2];
    double a3 = a_n_[(equationNumber*4)+3];

    if (derivative == 0) return a0 + a1*evaluationPoint + a2*evaluationPoint*evaluationPoint + a3*evaluationPoint*evaluationPoint*evaluationPoint;  // NOLINT
    if (derivative == 1) return a1 + 2*a2*evaluationPoint + 3*a3*evaluationPoint*evaluationPoint;  // NOLINT
    if (derivative == 2) return 2*a2 + 6*a3*evaluationPoint;
    if (derivative == 3) return 6*a3;
    throw std::runtime_error(
        "Request of a derivative outside of working margin. Only derivatives between 0 - 3 are accepted");  // NOLINT
}

// Private

void CubicPolynomial::insertEquation(
    std::vector<Eigen::Triplet<double>>& triplets,
    const int& fromRow, const int& fromCol, const std::vector<double>& eq) {
    for (uint64_t i = fromCol; i < static_cast<uint64_t>(fromCol + eq.size()); i++) {
        triplets[tripletsCounter_] = Eigen::Triplet<double>(fromRow, i, eq[i - fromCol]);
        tripletsCounter_++;
    }
}

static double objectiveFunction(
    const std::vector<double> &x, std::vector<double> &grad, void *data) {  // NOLINT
    return 0.0;  // Dummy objective
}

// Define the system of equations
static double pointAtT0(
    const std::vector<double> &x, std::vector<double> &grad, void *data) {  // NOLINT
    PolynomialParams* params = static_cast<PolynomialParams*>(data);

    const double y0 = params->y0;
    const double t0 = params->t0;

    double a = x[0];
    double b = x[1];
    double c = x[2];
    double d = x[3];

    return a*t0*t0*t0 + b*t0*t0 + c*t0 + d - y0;
}

static double pointAtTf(
    const std::vector<double> &x, std::vector<double> &grad, void *data) {  // NOLINT
    PolynomialParams* params = static_cast<PolynomialParams*>(data);

    const double yf = params->yf;

    double a = x[0];
    double b = x[1];
    double c = x[2];
    double d = x[3];
    double tf = x[4];

    return a*tf*tf*tf + b*tf*tf + c*tf + d - yf;
}

static double derivativeAtT0(
    const std::vector<double> &x, std::vector<double> &grad, void *data) {  // NOLINT
    PolynomialParams* params = static_cast<PolynomialParams*>(data);

    const double yd0 = params->yd0;
    const double t0 = params->t0;

    double a = x[0];
    double b = x[1];
    double c = x[2];

    return 3*a*t0*t0 + 2*b*t0 + c - yd0;
}

static double derivativeAtTf(
    const std::vector<double> &x, std::vector<double> &grad, void *data) {  // NOLINT
    PolynomialParams* params = static_cast<PolynomialParams*>(data);

    const double ydf = params->ydf;

    double a = x[0];
    double b = x[1];
    double c = x[2];
    double tf = x[4];

    return 3*a*tf*tf + 2*b*tf + c - ydf;
}

static double maxFirstDerivative(
    const std::vector<double> &x, std::vector<double> &grad, void *data) {  // NOLINT
    PolynomialParams* params = static_cast<PolynomialParams*>(data);

    const double ydmax = params->ydmax;

    double a = x[0];
    double b = x[1];
    double c = x[2];

    return abs(c - (b*b) / (3*a)) - ydmax;
}

static double secondDerivativeAtT0(
    const std::vector<double> &x, std::vector<double> &grad, void *data) {  // NOLINT
    PolynomialParams* params = static_cast<PolynomialParams*>(data);

    const double yddmax = params->yddmax;
    const double t0 = params->t0;

    double a = x[0];
    double b = x[1];

    return abs(6*a*t0 + 2*b) - yddmax;
}

static double secondDerivativeAtTf(
    const std::vector<double> &x, std::vector<double> &grad, void *data) {  // NOLINT
    PolynomialParams* params = static_cast<PolynomialParams*>(data);

    const double yddmax = params->yddmax;

    double a = x[0];
    double b = x[1];
    double tf = x[2];

    return abs(6*a*tf + 2*b) - yddmax;
}

}  // namespace crf::math::geometricmethods
