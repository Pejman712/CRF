/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2022
 *         Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
 */

#include <vector>
#include <optional>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "GeometricMethods/PointToPoint/PointToPoint.hpp"

namespace crf::math::geometricmethods {

PointToPoint::PointToPoint(
    const std::vector<double>& points0thDerivative,
    const std::vector<double>& ranges,
    const double& max2ndDerivative):
    points0thDerivative_(points0thDerivative),
    velocities_(points0thDerivative_.size(), 0),
    timeIntervals_(points0thDerivative_.size(), 0),
    accelerationTimes_(points0thDerivative_.size(), 0),
    totalTime_(points0thDerivative_.size(), 0),
    logger_("PointToPoint") {
    if (max2ndDerivative <= 0) {
        throw std::runtime_error(
            "Maximum first derivative or maximum second derivative cannot be less or equal to 0");
    }
    if (points0thDerivative_.size() != ranges.size()) {
        throw std::runtime_error(
            "The size of the vector of points and ranges don't match");
    }
    if (points0thDerivative.size() <= 1) {
        throw std::runtime_error(
            "Not enough points to generate a well defined function");
    }
    for (uint64_t i = 0; i < ranges.size()-1; i++) {
        if (ranges[i] > ranges[i+1]) {
            throw std::runtime_error(
                "Range cannot decrease through the duration of the function");
        }
    }

    // Calculate times and velocities
    for (uint64_t i = 0; i < points0thDerivative_.size(); i++) {
        if (i == 0) {  // First point
            velocities_[i] =
                (points0thDerivative_[i+1]-points0thDerivative_[i])/(ranges[i+1] - ranges[i]);
            accelerationTimes_[i] = 0.75*std::fabs(velocities_[i])/max2ndDerivative;
            timeIntervals_[i] = ranges[1] - ranges[0];
            totalTime_[i] = ranges[0] - accelerationTimes_[i]/2;
        } else if (i == points0thDerivative_.size()-1) {  // Final point
            velocities_[i] = 0;
            accelerationTimes_[i] = 0.75*std::fabs(-velocities_[i-1])/max2ndDerivative;
            timeIntervals_[i] = accelerationTimes_[i];
            totalTime_[i] = ranges[0] + accelerationTimes_[0] + sum(timeIntervals_) - 1.5*accelerationTimes_[i] - timeIntervals_[i];  // NOLINT
        } else {  // Middle point
            velocities_[i] =
                (points0thDerivative_[i+1]-points0thDerivative_[i])/(ranges[i+1] - ranges[i]);
            accelerationTimes_[i] =
                0.75*std::fabs(velocities_[i] - velocities_[i-1])/max2ndDerivative;
            timeIntervals_[i] = ranges[i+1] - ranges[i];
            totalTime_[i] = ranges[0] + accelerationTimes_[0] + sum(timeIntervals_) - 1.5*accelerationTimes_[i] - timeIntervals_[i];  // NOLINT
        }
        if (i != points0thDerivative_.size()-1 && timeIntervals_[i] < accelerationTimes_[i] + accelerationTimes_[i+1]) {  // NOLINT
            logger_->critical("Acceleration not high enough, time interval {}, vs acc time {}",
                timeIntervals_[i], accelerationTimes_[i] + accelerationTimes_[i+1]);
            throw std::runtime_error(
                "The acceleration is not high enough to follow a well defined path. Can't arrive to the points in the specified time");  // NOLINT
        }
    }
}

PointToPoint::PointToPoint(
    const std::vector<double>& points0thDerivative,
    const double& max1stDerivative, const double& max2ndDerivative):
    points0thDerivative_(points0thDerivative),
    velocities_(points0thDerivative_.size()),
    timeIntervals_(points0thDerivative_.size()),
    accelerationTimes_(points0thDerivative_.size()),
    totalTime_(points0thDerivative_.size()),
    logger_("PointToPoint") {
    if (max1stDerivative <= 0 || max2ndDerivative <= 0) {
        throw std::runtime_error(
            "Maximum first derivative or maximum second derivative cannot be less or equal to 0");
    }
    if (points0thDerivative.size() <= 1) {
        throw std::runtime_error(
            "Not enough points to generate a well defined function");
    }

    // Calculate times and velocities
    double Tpeek = 0;
    double Vpeek = 0;
    bool correctedVelocity = false;
    uint64_t i = 0;
    while (i < points0thDerivative_.size()) {
        if (i == 0) {  // First point
            if (!correctedVelocity) velocities_[i] = max1stDerivative*sign(points0thDerivative_[i+1]-points0thDerivative_[i]);  // NOLINT
            if (i < points0thDerivative_.size()-2) {
                Vpeek = max1stDerivative*sign(points0thDerivative_[i+2]-points0thDerivative_[i+1]);
                Tpeek = 0.75*std::fabs(Vpeek-velocities_[i])/max2ndDerivative;
            }
            accelerationTimes_[i] = 0.75*std::fabs(velocities_[i])/max2ndDerivative;
            timeIntervals_[i] = (points0thDerivative_[i+1]-points0thDerivative_[i])/velocities_[i];
            totalTime_[i] = 0;
        } else if (i == points0thDerivative_.size()-1) {  // Final point
            velocities_[i] = 0;
            accelerationTimes_[i] = 0.75*std::fabs(-velocities_[i-1])/max2ndDerivative;
            timeIntervals_[i] = accelerationTimes_[i];
            totalTime_[i] = accelerationTimes_[0] + sum(timeIntervals_) - accelerationTimes_[i] - timeIntervals_[i];  // NOLINT
            Tpeek = 0;
        } else {  // Middle point
            if (!correctedVelocity) velocities_[i] = max1stDerivative*sign(points0thDerivative_[i+1]-points0thDerivative_[i]);  // NOLINT
            if (i < points0thDerivative_.size()-2) {
                Vpeek = max1stDerivative*sign(points0thDerivative_[i+2]-points0thDerivative_[i+1]);
            } else {
                Vpeek = 0;
            }
            accelerationTimes_[i] =
                0.75*std::fabs(velocities_[i] - velocities_[i-1])/max2ndDerivative;
            Tpeek = 0.75*std::fabs(Vpeek - velocities_[i])/max2ndDerivative;
            timeIntervals_[i] =
                (points0thDerivative_[i+1] - points0thDerivative_[i])/velocities_[i];
            totalTime_[i] =accelerationTimes_[0] + sum(timeIntervals_) - accelerationTimes_[i] - timeIntervals_[i];  // NOLINT
        }
        correctedVelocity = false;
        if (i != points0thDerivative_.size() && timeIntervals_[i] < accelerationTimes_[i] + Tpeek) {
            logger_->warn("the acceleration is not high enough to follow a well defined path at i = {}.", i);  // NOLINT
            logger_->warn("Computing trajectory with reduced velocity.");

            double hmin = accelerationTimes_[i]+Tpeek;
            velocities_[i] = (points0thDerivative_[i+1]-points0thDerivative_[i])/hmin;
            correctedVelocity = true;
        } else {
            i = i + 1;
        }
    }
}

PointToPoint::~PointToPoint() {
}

std::optional<double> PointToPoint::getRange() const {
    return accelerationTimes_.front() + sum(timeIntervals_) +
        accelerationTimes_.back() - timeIntervals_.back();
}

std::optional<double> PointToPoint::evaluate(double evaluationPoint, unsigned int derivative) {
    if (evaluationPoint < totalTime_[0]) {
        if (derivative == 0) return points0thDerivative_[0];
        if (derivative == 1) return 0;
        if (derivative == 2) return 0;
        if (derivative == 3) return 0;
    } else if (evaluationPoint > totalTime_[0] + getRange().value()) {
        if (derivative == 0) return points0thDerivative_.back();
        if (derivative == 1) return 0;
        if (derivative == 2) return 0;
        if (derivative == 3) return 0;
    } else {
        // Find the segment k we are in
        uint64_t k = 0;
        while (k < totalTime_.size() && totalTime_[k] <= evaluationPoint) {
            k++;
        }
        k--;

        // evaluate the function
        if (evaluationPoint < totalTime_[k] + 2*accelerationTimes_[k]) {
            // quadratic / acceleration part
            if (k == 0) {
                if (derivative == 0) return points0thDerivative_[k] - 0.0625*std::pow(accelerationTimes_[k], -3)*(std::pow(evaluationPoint-totalTime_[k], 3))*(evaluationPoint-totalTime_[k]-4*accelerationTimes_[k])*velocities_[k];  // NOLINT
                if (derivative == 1) return -0.25*std::pow(accelerationTimes_[k], -3)*(std::pow(evaluationPoint-totalTime_[k], 2))*(evaluationPoint-totalTime_[k]-3*accelerationTimes_[k])*velocities_[k];  // NOLINT
                if (derivative == 2) return -0.75*std::pow(accelerationTimes_[k], -3)*(evaluationPoint-totalTime_[k])*(evaluationPoint-totalTime_[k]-2*accelerationTimes_[k])*velocities_[k];  // NOLINT
                if (derivative == 3) return -0.75*std::pow(accelerationTimes_[k], -3)*(2*evaluationPoint-2*totalTime_[k]-2*accelerationTimes_[k])*velocities_[k];  // NOLINT
            } else {
                if (derivative == 0) return points0thDerivative_[k] - 0.0625*std::pow(accelerationTimes_[k], -3)*(std::pow(evaluationPoint-totalTime_[k], 3))*(evaluationPoint-totalTime_[k]-4*accelerationTimes_[k])*(velocities_[k]-velocities_[k-1])+(evaluationPoint-totalTime_[k]-accelerationTimes_[k])*velocities_[k-1];  // NOLINT
                if (derivative == 1) return velocities_[k-1]-(0.25)*std::pow(accelerationTimes_[k], -3)*(std::pow(evaluationPoint-totalTime_[k], 2))*(evaluationPoint-totalTime_[k]-3*accelerationTimes_[k])*(velocities_[k]-velocities_[k-1]);  // NOLINT
                if (derivative == 2) return -0.75*std::pow(accelerationTimes_[k], -3)*(evaluationPoint-totalTime_[k])*(evaluationPoint-totalTime_[k]-2*accelerationTimes_[k])*(velocities_[k]-velocities_[k-1]);  // NOLINT
                if (derivative == 3) return -0.75*std::pow(accelerationTimes_[k], -3)*(2*evaluationPoint-2*totalTime_[k]-2*accelerationTimes_[k])*(velocities_[k]-velocities_[k-1]);  // NOLINT
            }
        } else {
            // linear / zero acceleration part
            if (derivative == 0) return (evaluationPoint-totalTime_[k]-accelerationTimes_[k])*velocities_[k]+points0thDerivative_[k];  // NOLINT
            if (derivative == 1) return velocities_[k];
            if (derivative == 2) return 0;
            if (derivative == 3) return 0;
        }
    }
    throw std::runtime_error(
        "Request of a derivative outside of working margin. Only derivatives between 0 - 3 are accepted");  // NOLINT
}

double PointToPoint::getStartPoint() const {
    return totalTime_[0];
}

// Private

int PointToPoint::sign(const double& num) {
    return num/std::fabs(num);
}

double PointToPoint::sum(const std::vector<double>& vec) const {
    double sum = 0;
    for (auto& value : vec) {
        sum += value;
    }
    return sum;
}

}  // namespace crf::math::geometricmethods
