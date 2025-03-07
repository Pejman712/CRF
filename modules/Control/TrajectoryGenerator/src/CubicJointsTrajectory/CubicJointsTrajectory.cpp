/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <vector>
#include <map>

#include "TrajectoryGenerator/CubicJointsTrajectory/CubicJointsTrajectory.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

namespace crf {
namespace control {
namespace trajectorygenerator {

CubicJointsTrajectory::CubicJointsTrajectory(
    const JointVelocities& maxVelocity, const JointAccelerations& maxAcceleration):
    maxVelocity_(maxVelocity),
    maxAcceleration_(maxAcceleration),
    initialPosition_(),
    ranges_({0}),
    firstTrajectory_(true),
    initialPositionSet_(false),
    lastEvaluationPoint_(0),
    endTrajectoryRange_(-1),
    logger_("CubicJointsTrajectory") {
    logger_->debug("CTor");
    if (maxVelocity_.size() != maxAcceleration_.size()) {
        throw std::runtime_error(
            "Dimension between maximum acceleration and velocity don't match");
    }
    dimensions_ = maxVelocity_.size();
}

CubicJointsTrajectory::~CubicJointsTrajectory() {
    logger_->debug("DTor");
}

void CubicJointsTrajectory::setInitialPosition(const JointPositions& initialPosition) {
    logger_->debug("setInitialPosition");
    if (!firstTrajectory_) {
        throw std::runtime_error(
            "Cannot set initial position as a trajectory has already been defined."
            " To add points to the existing trajectory, use append(). To start the trajectory "
            "again, call reset() then setInitialPose().");
    }

    initialPosition_ = initialPosition;
    initialPositionSet_ = true;
}

void CubicJointsTrajectory::append(const std::vector<JointPositions>& path) {
    logger_->debug("append");
    if (!initialPositionSet_) {
        throw std::runtime_error(
            "Initial position has not been set, call it first to define the start point of the"
            " trajectory");
    }
    std::vector<JointPositions> pathPoints(path);

    for (uint64_t i = 0; i < pathPoints.size(); i++) {
        if (dimensions_ != pathPoints[i].size()) {
            throw std::runtime_error(
                "Path dimensions don't match with the specified dimensions in vel and acc");
        }
    }
    std::unique_lock<std::mutex> lck(mtx_);
    std::vector<double> Vinitial(dimensions_);
    double initialTime = 0;
    if (firstTrajectory_) {
        // First Trajectory: initial time and vel 0
        for (uint64_t i = 0; i < dimensions_; i++) {
            Vinitial[i] = 0;
        }
        initialTime = lastEvaluationPoint_;
        pathPoints.insert(pathPoints.begin(), initialPosition_);
    } else if (lastEvaluationPoint_ <= ranges_[ranges_.size()-2]) {
        // Before the end of the last trajectory
        // initial time and vel equal to second to last point to avoid
        // deceleration and make a smooth addition of the points
        for (uint64_t i = 0; i < dimensions_; i++) {
            Vinitial[i] = currentTrajectory_[i].evaluate(ranges_[ranges_.size()-2], 1).value();
        }
        initialTime = ranges_[ranges_.size()-2];
        pathPoints.insert(pathPoints.begin(), pathPoints_[pathPoints_.size()-1]);
        pathPoints.insert(pathPoints.begin(), pathPoints_[pathPoints_.size()-2]);
    } else {
        // Already past the point to append smoothly
        // initial time and velocity are those of the current point
        // if trajectory finished already then vel 0 and pos static
        JointPositions currentPoint(dimensions_);
        for (uint64_t i = 0; i < dimensions_; i++) {
            currentPoint[i] = currentTrajectory_[i].evaluate(lastEvaluationPoint_, 0).value();
            Vinitial[i] = currentTrajectory_[i].evaluate(lastEvaluationPoint_, 1).value();
        }
        initialTime = lastEvaluationPoint_;
        pathPoints.insert(pathPoints.begin(), pathPoints_.back());
        pathPoints.insert(pathPoints.begin(), currentPoint);
    }
    lck.unlock();

    // Eliminate consecutive duplicates in every dimension
    for (uint64_t i = 0; i < pathPoints.size()-1; i++) {
        uint64_t counter = 0;
        for (uint64_t j = 0; j < dimensions_; j++) {
            if (std::fabs(pathPoints[i+1][j] - pathPoints[i][j]) < 1e-7) counter++;
        }
        if (counter == dimensions_) {
            pathPoints.erase(pathPoints.begin() + i);
            logger_->info("Erased duplicated point in position {}", i);
            i--;
        }
    }

    uint64_t numberOfPoints = pathPoints.size();
    std::vector<double> newRanges(numberOfPoints);
    newRanges[0] = initialTime;

    // Calculate times
    // For every point calculate the most restrictive dimension and add it to ranges
    std::unique_lock<std::mutex> lckProfile(profileValuesMtx_);
    for (uint64_t i = 0; i < numberOfPoints-1; i++) {
        double maxTime = 0;
        for (uint64_t j = 0; j < dimensions_; j++) {
            double time1 = std::fabs(
                pathPoints[i+1][j] - pathPoints[i][j])*1.5/maxVelocity_[j] + 1;
            double time2 = std::sqrt(
                std::fabs(pathPoints[i+1][j] - pathPoints[i][j])/(6.0/maxAcceleration_[j])) + 1;
            if (time1 > maxTime) maxTime = time1;
            if (time2 > maxTime) maxTime = time2;
        }
        newRanges[i+1] = newRanges[i] + maxTime;
    }
    lckProfile.unlock();

    logger_->info("append {} points in time range ({}, {})",
        pathPoints.size(), initialTime, newRanges.back());
    endTrajectoryRange_ = newRanges.back();

    // Generate a cubic trajectory for each dimension
    std::vector<crf::math::geometricmethods::CubicPolynomial> calculatedTrajectory;
    for (uint64_t i = 0; i < dimensions_; i++) {
        std::vector<double> pathOfDimensionN(numberOfPoints);
        for (uint64_t j = 0; j < numberOfPoints; j++) {
            pathOfDimensionN[j] = pathPoints[j][i];
        }
        calculatedTrajectory.push_back(crf::math::geometricmethods::CubicPolynomial(
            pathOfDimensionN, Vinitial[i], 0, newRanges));
    }

    // Add to map with the starting time as key
    lck.lock();
    if (firstTrajectory_) {
        currentTrajectory_ = calculatedTrajectory;
        firstTrajectory_ = false;
    }
    trajectories_.insert({newRanges[0], calculatedTrajectory});

    // Update total ranges and points
    ranges_.insert(ranges_.end(), newRanges.begin(), newRanges.end());
    pathPoints_.insert(pathPoints_.end(), pathPoints.begin(), pathPoints.end());
}

void CubicJointsTrajectory::setProfileVelocity(const JointVelocities& vel) {
    logger_->debug("setProfileVelocity");
    if (dimensions_ != vel.size()) {
        throw std::runtime_error(
            "CubicJointsTrajectory - Wrong dimensions used for the profile velocity");
    }
    std::scoped_lock<std::mutex> lckProfile(profileValuesMtx_);
    maxVelocity_ = vel;
}

void CubicJointsTrajectory::setProfileAcceleration(const JointAccelerations& acc) {
    if (dimensions_ != acc.size()) {
        throw std::runtime_error(
            "CubicJointsTrajectory - Wrong dimensions used for the profile acceleration");
    }
    std::scoped_lock<std::mutex> lckProfile(profileValuesMtx_);
    maxAcceleration_ = acc;
}

void CubicJointsTrajectory::reset() {
    logger_->debug("reset");
    firstTrajectory_ = true;
    initialPositionSet_ = false;
    std::scoped_lock<std::mutex> lck(mtx_);
    trajectories_.clear();
    ranges_.clear();
    pathPoints_.clear();
    currentTrajectory_.clear();
    lastEvaluationPoint_ = 0;
}

void CubicJointsTrajectory::clearMemory() {
    logger_->debug("clearMemory");
    std::scoped_lock<std::mutex> lck(mtx_);
    if (trajectories_.size() < 2) {
        logger_->info("One or less trajectories are computed");
        return;
    }
    auto it = trajectories_.lower_bound(lastEvaluationPoint_);
    if (it == trajectories_.begin()) {
        logger_->warn(
            "The time has not reached the first trajectory yet, cannot clear memory");
        return;
    }
    it--;
    if (it == trajectories_.begin()) {
        logger_->warn(
            "The time is still inside the first trajectory, no previous trajectories to clear");
        return;
    }
    // Erase from the beginning to the previous traj
    uint64_t pre = trajectories_.size();
    trajectories_.erase(trajectories_.begin(), it--);
    logger_->info("Cleared {} old trajectories", pre - trajectories_.size());
}

bool CubicJointsTrajectory::isTrajectoryRunning() {
    logger_->debug("isTrajectoryRunning");
    return endTrajectoryRange_ > lastEvaluationPoint_;
}

JointSignals CubicJointsTrajectory::getTrajectoryPoint(double Tp) {
    logger_->debug("getTrajectoryPoint({})", Tp);
    std::scoped_lock<std::mutex> lck(mtx_);
    if (trajectories_.empty()) {
        throw std::runtime_error(
            "No trajectories have been generated, cannot retrieve trajectory point");
    }
    if (lastEvaluationPoint_ > Tp) {
        throw std::runtime_error(
            "Going backwards in a trajectory is not permitted");
    }
    lastEvaluationPoint_ = Tp;
    // Returns the first value that is not less than Tp, so key > Tp, we want the previous one
    auto it = trajectories_.lower_bound(Tp);
    it--;
    currentTrajectory_ = it->second;

    // Now we have an iterator to the traj we are in
    JointSignals result;
    JointPositions evaluation0thOfDimension(dimensions_);
    JointVelocities evaluation1stOfDimension(dimensions_);
    JointAccelerations evaluation2ndOfDimension(dimensions_);

    for (uint64_t j = 0; j < dimensions_; j++) {
        evaluation0thOfDimension[j] = currentTrajectory_[j].evaluate(Tp, 0).value();
        evaluation1stOfDimension[j] = currentTrajectory_[j].evaluate(Tp, 1).value();
        evaluation2ndOfDimension[j] = currentTrajectory_[j].evaluate(Tp, 2).value();
    }

    result.positions = evaluation0thOfDimension;
    result.velocities = evaluation1stOfDimension;
    result.accelerations = evaluation2ndOfDimension;
    return result;
}

}  // namespace trajectorygenerator
}  // namespace control
}  // namespace crf
