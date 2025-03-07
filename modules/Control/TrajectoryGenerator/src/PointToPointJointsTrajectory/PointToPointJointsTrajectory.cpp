/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <vector>
#include <map>

#include "TrajectoryGenerator/PointToPointJointsTrajectory/PointToPointJointsTrajectory.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;
using crf::utility::types::stdVectorFromEigenVector;

namespace crf::control::trajectorygenerator {

PointToPointJointsTrajectory::PointToPointJointsTrajectory(
    const JointVelocities& maxVelocity, const JointAccelerations& maxAcceleration):
    maxVelocity_(maxVelocity),
    maxAcceleration_(maxAcceleration),
    initialPosition_(),
    ranges_({0}),
    firstTrajectory_(true),
    initialPositionSet_(false),
    lastEvaluationPoint_(0),
    endTrajectoryRange_(-1),
    logger_("PointToPointJointsTrajectory") {
    logger_->debug("CTor");
    if (maxVelocity_.size() != maxAcceleration_.size()) {
        throw std::runtime_error("Dimension between maximum acceleration and velocity don't match");
    }
    dimensions_ = maxVelocity_.size();
}

PointToPointJointsTrajectory::~PointToPointJointsTrajectory() {
    logger_->debug("DTor");
}

void PointToPointJointsTrajectory::setInitialPosition(const JointPositions& initialPosition) {
    logger_->debug("setInitialPosition");
    if (!firstTrajectory_) {
        throw std::runtime_error("Cannot set initial position as a trajectory has already been "
            "defined. To add points to the existing trajectory, use append(). To start the "
            "trajectory again, call reset() then setInitialPose().");
    }

    initialPosition_ = initialPosition;
    initialPositionSet_ = true;
}

void PointToPointJointsTrajectory::append(const std::vector<JointPositions>& path) {
    if (!initialPositionSet_) {
        throw std::runtime_error("Initial position has not been set, call it first to define the "
            "start point of the trajectory");
    }
    std::vector<JointPositions> pathPoints(path);

    for (uint64_t i = 0; i < pathPoints.size(); i++) {
        if (dimensions_ != pathPoints[i].size()) {
            throw std::runtime_error("Path dimensions don't match with the specified dimensions in "
                "velocity and acceleration");
        }
    }

    std::unique_lock<std::mutex> lck(mtx_);
    double initialTime = 0;
    bool beforeTheEndOfPreviousTraj = false;
    if (firstTrajectory_) {
        // First Trajectory: initial time and vel 0
        initialTime = lastEvaluationPoint_;
        pathPoints.insert(pathPoints.begin(), initialPosition_);
    } else if (lastEvaluationPoint_ <= ranges_[ranges_.size()-2]) {
        // Before the end of the last trajectory
        // initial time equal to second to last point to avoid
        // deceleration and make a smooth addition of the points
        initialTime = ranges_[ranges_.size()-2];
        beforeTheEndOfPreviousTraj = true;
        pathPoints.insert(pathPoints.begin(), pathPoints_[pathPoints_.size()-1]);
        pathPoints.insert(pathPoints.begin(), pathPoints_[pathPoints_.size()-2]);
    } else {
        // Already past the point to append smoothly
        // We calculate the last point in time of the current trajectory
        double lastPoint = 0;
        for (uint64_t j = 0; j < dimensions_; j++) {
            if (currentTrajectory_[j].getStartPoint() +
            currentTrajectory_[j].getRange().value() > lastPoint) {
                lastPoint = currentTrajectory_[j].getStartPoint() +
                    currentTrajectory_[j].getRange().value();
            }
        }
        // If we haven't finished yet we finish the trajectory, otherwise
        // we add where we are
        if (lastEvaluationPoint_ < lastPoint) {
            logger_->warn("Appending too late, will end the current trajectory and append after");
            initialTime = lastPoint;
            pathPoints.insert(pathPoints.begin(), pathPoints_.back());
        } else {
            initialTime = lastEvaluationPoint_;
            pathPoints.insert(pathPoints.begin(), pathPoints_.back());
        }
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

    int64_t numberOfPoints = static_cast<int64_t>(pathPoints.size());
    std::vector<double> newRanges(numberOfPoints);
    newRanges[0] = initialTime;
    std::unique_lock<std::mutex> lckProfile(profileValuesMtx_);
    std::vector<double> trajectoryVelocity(stdVectorFromEigenVector(maxVelocity_.raw()));

    // Calculate times
    // For every point calculate the most restrictive dimension and add it to ranges
    // We also have to obtain the maximum acceleration with the velocity of each
    // joint and add it to the initial range.
    for (int64_t i = 0; i < numberOfPoints-1; i++) {
        double maxTime = 0;
        double maxAccTime = 0;
        for (uint64_t j = 0; j < dimensions_; j++) {
            double time = std::fabs(pathPoints[i+1][j] - pathPoints[i][j])/trajectoryVelocity[j];
            if (time > maxTime) maxTime = time;
        }
        for (uint64_t j = 0; j < dimensions_; j++) {
            double time = std::fabs(pathPoints[i+1][j] - pathPoints[i][j])/trajectoryVelocity[j];
            double accTime = 0.75*(time/maxTime)*trajectoryVelocity[j]/maxAcceleration_[j];
            if (accTime > maxAccTime) maxAccTime = accTime;
        }
        // Check to see if we have enough time to accelerate to the desired velocity
        // otherwise we reduce the velocity and calulcate all points again (i = -1 so
        // so that in the first iteration it's back to 0 and we start again)
        if (maxTime < maxAccTime*2) {
            logger_->debug("Not enough acceleration to follow the velocity ({} vs {}), "
                "reducing max velocity by 5%", maxTime, maxAccTime*2);
            for (uint64_t j = 0; j < dimensions_; j++) {
                trajectoryVelocity[j] = trajectoryVelocity[j]*0.95;
            }
            i = -1;
            continue;
        }
        // We only care about the acceleration time if we are going to use it,
        // in the case we append before the end of the prev traj we don't use it
        // This avoids the starting traj to start in a negative value
        if (i == 0 && !beforeTheEndOfPreviousTraj) {
            newRanges[0] += maxAccTime;
        }
        newRanges[i+1] = newRanges[i] + maxTime;
    }
    lckProfile.unlock();

    logger_->debug("append {} points in time range ({}, {})",
        pathPoints.size(), initialTime, newRanges.back());
    endTrajectoryRange_ = newRanges.back();

    // Generate a point to point trajectory for each dimension
    std::vector<crf::math::geometricmethods::PointToPoint> calculatedTrajectory;
    for (uint64_t i = 0; i < dimensions_; i++) {
        std::vector<double> pathOfDimensionN(numberOfPoints);
        for (int64_t j = 0; j < numberOfPoints; j++) {
            pathOfDimensionN[j] = pathPoints[j][i];
        }
        calculatedTrajectory.push_back(crf::math::geometricmethods::PointToPoint(
            pathOfDimensionN, newRanges, maxAcceleration_[i]));
    }

    lck.lock();
    if (firstTrajectory_) {
        currentTrajectory_ = calculatedTrajectory;
        firstTrajectory_ = false;
    }

    // Add to map with the starting time as key
    if (beforeTheEndOfPreviousTraj) {
        // In this case we need to append where the functions collide,
        // we add them on the linear part to asure a smooth connexion,
        // this is between the ranges on the middle
        trajectories_.insert({
            (newRanges[0] + (newRanges[1] - newRanges[0])/2.0), calculatedTrajectory});
    } else {
        // otherwise on the start
        trajectories_.insert({initialTime, calculatedTrajectory});
    }

    // Update total ranges and points
    ranges_.insert(ranges_.end(), newRanges.begin(), newRanges.end());
    pathPoints_.insert(pathPoints_.end(), pathPoints.begin(), pathPoints.end());
}

void PointToPointJointsTrajectory::setProfileVelocity(const JointVelocities& vel) {
    logger_->debug("setProfileVelocity");
    if (dimensions_ != vel.size()) {
        throw std::runtime_error(
            "PointToPointJointsTrajectory - Wrong dimensions used for the profile velocity");
    }
    std::scoped_lock<std::mutex> lckProfile(profileValuesMtx_);
    maxVelocity_ = vel;
}

void PointToPointJointsTrajectory::setProfileAcceleration(const JointAccelerations& acc) {
    if (dimensions_ != acc.size()) {
        throw std::runtime_error(
            "PointToPointJointsTrajectory - Wrong dimensions used for the profile acceleration");
    }
    std::scoped_lock<std::mutex> lckProfile(profileValuesMtx_);
    maxAcceleration_ = acc;
}

void PointToPointJointsTrajectory::reset() {
    logger_->debug("reset");
    firstTrajectory_ = true;
    initialPositionSet_ = false;
    std::scoped_lock<std::mutex> lck(mtx_);
    trajectories_.clear();
    ranges_.clear();
    pathPoints_.clear();
    lastEvaluationPoint_ = 0;
}

void PointToPointJointsTrajectory::clearMemory() {
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

bool PointToPointJointsTrajectory::isTrajectoryRunning() {
    logger_->debug("isTrajectoryRunning");
    return endTrajectoryRange_ > lastEvaluationPoint_;
}

JointSignals PointToPointJointsTrajectory::getTrajectoryPoint(double Tp) {
    logger_->debug("getTrajectoryPoint {}", Tp);
    std::scoped_lock<std::mutex> lck(mtx_);
    if (!initialPositionSet_) {
        throw std::runtime_error(
            "No trajectories have been generated, cannot retrieve trajectory point");
    }
    if (lastEvaluationPoint_ > Tp) {
        throw std::runtime_error(
            "Going backwards in a trajectory is not permitted");
    }

    // Now we have an iterator to the traj we are in
    JointSignals result;
    JointPositions evaluation0thOfDimension(dimensions_);
    JointVelocities evaluation1stOfDimension(dimensions_);
    JointAccelerations evaluation2ndOfDimension(dimensions_);

    // No trajectory has been defined but initial pose has been set
    if (trajectories_.empty()) {
        result.positions = initialPosition_;
        result.velocities = evaluation1stOfDimension;
        result.accelerations = evaluation2ndOfDimension;
        return result;
    }

    lastEvaluationPoint_ = Tp;
    // Returns the first value that is not less than Tp, so key > Tp, we want the previous one
    auto it = trajectories_.lower_bound(Tp);
    it--;
    currentTrajectory_ = it->second;

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

}  // namespace crf::control::trajectorygenerator
