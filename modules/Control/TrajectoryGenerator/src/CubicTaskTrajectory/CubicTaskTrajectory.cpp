/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Chelsea Davidson CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "TrajectoryGenerator/CubicTaskTrajectory/CubicTaskTrajectory.hpp"

namespace crf::control::trajectorygenerator {

CubicTaskTrajectory::CubicTaskTrajectory(
    TaskVelocity maxVelocity,
    TaskAcceleration maxAcceleration) :
    maxVelocity_(maxVelocity),
    maxAcceleration_(maxAcceleration),
    lastEvaluationTime_(0.0),
    isFirstTrajectory_(true),
    isInitialPoseSet_(false),
    logger_("CubicTaskTrajectory") {
    logger_->debug("CTor");
}

CubicTaskTrajectory::~CubicTaskTrajectory() {
    logger_->debug("DTor");
}

void CubicTaskTrajectory::setInitialPose(const TaskPose& initialPose) {
    logger_->debug("setInitialPose");
    if (!isFirstTrajectory_) {
        throw std::runtime_error(
            "Cannot set initial pose as a trajectory has already been defined."
            " To add points to the existing trajectory, use append(). To start the trajectory "
            "again, call reset() then setInitialPose().");
    }

    initialPose_ = initialPose;

    if (isInitialPoseSet_) {
        logger_->info(
            "Initial pose has been reset. Note that if the trajectory has already been "
            "evaluated, this reset action will cause a sudden jump in the evaluated trajectory.");
    }

    isInitialPoseSet_ = true;
}

void CubicTaskTrajectory::append(const std::vector<TaskPose>& path) {
    logger_->debug("append");

    // Variable definitions
    std::vector<TaskPose> appendPathPoses(path);  // Pre-fill with the new path poses
    double startTime;                             // Start time of new TaskTrajectory
    std::size_t numNewPathPoints = path.size();
    std::size_t numStartPoints;  // No. points from existing trajectory used to make the new one
    TaskVelocity startVelocity;  // Start velocity of the new trajectory
    std::unique_lock<std::mutex> lck(mtx_);  // Ensures only one thread accesses this data at a time

    // Check for invalid append calls
    if (numNewPathPoints < 1) {
        throw std::runtime_error(
            "Require at least 1 point in the append path to continue the trajectory.");
    }
    if (!isInitialPoseSet_) {  // No trajectory or initial pose defined
        throw std::runtime_error(
            "Must set an initial pose before defining a trajectory. Please "
            "call setInitialPose() before calling append().");
    }

    // Determine which append condition this is
    if (isFirstTrajectory_) {  // No trajectory defined
        // Set starting pose of new trajectory to be the initialPose_
        numStartPoints = 1;
        appendPathPoses.insert(appendPathPoses.begin(), initialPose_);
        startTime = lastEvaluationTime_;
        // NOTE: ^lastEvaluationTime_ should be 0.0 but could also be another time if it was
        //       evaluated after the initial pose was set but before a trajectory was made
        startVelocity = zeroTaskVelocity_;  // Start stationary

    } else if (lastEvaluationTime_ <= ranges_[ranges_.size() - 2]) {
        // Before the end of the last trajectory - set initial time and vel equal to second last
        // global path point to avoid deceleration and make a smooth addition of the points

        numStartPoints = 2;

        // Put second last and last global path point as the first points of this new trajectory
        // - since using insert(...begin()), insert in reverse order
        TaskPose lastPose = poses_[ranges_.size() - 1];
        appendPathPoses.insert(appendPathPoses.begin(), lastPose);

        TaskPose secondLastPose = poses_[ranges_.size() - 2];
        appendPathPoses.insert(appendPathPoses.begin(), secondLastPose);
        startTime = ranges_[ranges_.size() - 2];

        // Set starting velocity to be velocity of second last point
        startVelocity = evaluateTaskVelocity(lastTrajectory_, ranges_[ranges_.size() - 2]);

    } else if (lastEvaluationTime_ < ranges_[ranges_.size() - 1]) {
        // Already past the point to append smoothly - initial time and velocity are those of the
        // current point

        numStartPoints = 2;

        // Put current point (point at lastEvaluationTime_) as starting point for this trajectory,
        // followed by the last path point - since using insert(...begin()), insert in reverse order
        TaskPose lastPose = poses_[ranges_.size() - 1];
        appendPathPoses.insert(appendPathPoses.begin(), lastPose);

        TaskPose currentPose = evaluateTaskPose(currentTrajectory_, lastEvaluationTime_);
        appendPathPoses.insert(appendPathPoses.begin(), currentPose);
        startTime = lastEvaluationTime_;

        // Set the starting velocity to be the velocity at the lastEvaluationPoint_
        startVelocity = evaluateTaskVelocity(currentTrajectory_, lastEvaluationTime_);

    } else {
        // Trajectory has already finished - initial time and velocity is that of the current point
        // which will have the same pose as the last point and should therefore have 0 velocity

        numStartPoints = 1;

        // Put current point as the starting point for this trajectory
        TaskPose currentPose = evaluateTaskPose(currentTrajectory_, lastEvaluationTime_);
        appendPathPoses.insert(appendPathPoses.begin(), currentPose);
        startTime = lastEvaluationTime_;

        // Set the starting velocity to be the velocity at the last evaluation point - should be 0
        startVelocity = evaluateTaskVelocity(currentTrajectory_, lastEvaluationTime_);
    }
    lck.unlock();

    // Remove points with duplicate poses
    for (std::size_t i = 0; i < appendPathPoses.size() - 1; i++) {
        if (areAlmostEqual(appendPathPoses[i + 1], appendPathPoses[i], epsilon_)) {
            appendPathPoses.erase(appendPathPoses.begin() + i);
            logger_->info("Erased duplicated point in position {}", i);
            i--;
        }
    }

    // Determine length of path after duplicate points have been removed
    std::size_t appendPathLength = appendPathPoses.size();

    if (appendPathLength <= numStartPoints) {
        throw std::runtime_error("Path contained only duplicate points. There are no new points to "
                                 "append.");
    }

    // New trajectory variable definitions - defined here so the size of the vectors are known
    TaskTrajectory appendTrajectory;  // new trajectory being made
    std::vector<double> timeInstances(appendPathLength);
    std::vector<double> xPoints(appendPathLength);
    std::vector<double> yPoints(appendPathLength);
    std::vector<double> zPoints(appendPathLength);
    std::vector<Orientation> orientationPoints(appendPathLength);

    // Define start time
    timeInstances[0] = startTime;

    // Decompose append path poses into x,y,z,orientation points and calculate time instances for
    // new path points
    std::unique_lock<std::mutex> lckProfile(profileValuesMtx_);
    for (std::size_t i = 0; i < appendPathLength; i++) {
        // Store position and orientation of this append path point
        Eigen::Vector3d position = appendPathPoses[i].getPosition();
        xPoints[i] = position[0];
        yPoints[i] = position[1];
        zPoints[i] = position[2];
        orientationPoints[i] = appendPathPoses[i].getOrientation();

        if (i < 1) {  // Already defined time instance for first point so no need to calculate it
            continue;
        }

        // Determine the movement that takes the longest (x, y, z, orientation) and set all time
        // instances based on that so that the trajectories hit the desired pose at the same time
        std::vector<double> movementTimes;
        movementTimes.push_back(getMovementTime(xPoints[i - 1], xPoints[i], 0));  // x
        movementTimes.push_back(getMovementTime(yPoints[i - 1], yPoints[i], 1));  // y
        movementTimes.push_back(getMovementTime(zPoints[i - 1], zPoints[i], 2));  // z
        movementTimes.push_back(getMovementTime(orientationPoints[i - 1], orientationPoints[i]));

        // Use max movement time
        timeInstances[i] = timeInstances[i - 1] +
            *std::max_element(movementTimes.begin(), movementTimes.end());
    }
    lckProfile.unlock();

    // Create new trajectory
    appendTrajectory.linear =
        std::make_shared<std::array<CubicPolynomial, 3>>(std::array<CubicPolynomial, 3>(
            {CubicPolynomial(xPoints, startVelocity[0], zeroTaskVelocity_[0], timeInstances),
             CubicPolynomial(yPoints, startVelocity[1], zeroTaskVelocity_[1], timeInstances),
             CubicPolynomial(zPoints, startVelocity[2], zeroTaskVelocity_[2], timeInstances)}));
    appendTrajectory.angular = std::make_shared<CubicOrientationSpline>(CubicOrientationSpline(
        orientationPoints,
        startVelocity.raw().tail(3),
        zeroTaskVelocity_.raw().tail(3),
        timeInstances));

    // Update variables storing trajectory info
    lck.lock();
    if (isFirstTrajectory_) {  // If it was the first trajectory, it no longer is
        // Set these vars here in case first trajectory append was invalid
        isFirstTrajectory_ = false;

        // Add initial pose and start time to global poses and ranges
        // - do this here instead of in setInitialPose() in case it is called multiple times.
        poses_.insert(poses_.begin(), {initialPose_});
        ranges_.insert(ranges_.begin(), {lastEvaluationTime_});
    }

    // Specify start time of new trajectory - overwrite trajectory in map if this key already exists
    trajectories_[timeInstances[0]] = appendTrajectory;

    // Add these poses to global poses - only add new append path poses
    poses_.insert(poses_.end(), appendPathPoses.begin() + numStartPoints, appendPathPoses.end());
    // Store the last trajectory created
    lastTrajectory_ = appendTrajectory;
    // Remove the last starting point time if there is one since it will be updated in the next line
    if (numStartPoints == 2) {
        ranges_.erase(ranges_.begin() + ranges_.size() - 1);
    }
    // Add new timeInstances to ranges - don't add first timeInstance as it's already in the list
    ranges_.insert(ranges_.end(), timeInstances.begin() + 1, timeInstances.end());
}

void CubicTaskTrajectory::setProfileVelocity(const TaskVelocity& newProfileVelocity) {
    logger_->debug("setProfileVelocity");
    std::scoped_lock<std::mutex> lckProfile(profileValuesMtx_);
    maxVelocity_ = newProfileVelocity;
}

void CubicTaskTrajectory::setProfileAcceleration(const TaskAcceleration& newProfileAcceleration) {
    logger_->debug("setProfileAcceleration");
    std::scoped_lock<std::mutex> lckProfile(profileValuesMtx_);
    maxAcceleration_ = newProfileAcceleration;
}

void CubicTaskTrajectory::reset() {
    logger_->debug("reset");
    std::scoped_lock<std::mutex> lck(mtx_);

    poses_.clear();
    ranges_.clear();
    trajectories_.clear();
    isFirstTrajectory_ = true;
    isInitialPoseSet_ = false;
    lastEvaluationTime_ = 0.0;
    currentTrajectory_.clear();
    lastTrajectory_.clear();
}

void CubicTaskTrajectory::clearMemory() {
    logger_->debug("clearMemory");
    std::scoped_lock<std::mutex> lck(mtx_);
    if (trajectories_.size() < 2) {
        logger_->info("Less than 2 trajectories defined, cannot clear any memory.");
        return;
    }
    // Determine which trajectory is currently being used based on the lastEvaluationPoint_
    auto it = trajectories_.upper_bound(lastEvaluationTime_);
    it--;
    if (it == trajectories_.begin()) {
        logger_->warn(
            "The time is still inside the first trajectory, no previous trajectories to clear");
        return;
    }
    // Erase from the beginning to the previous traj
    uint64_t originalNumTrajectories = trajectories_.size();
    trajectories_.erase(trajectories_.begin(), it--);  // it-- is the previous trajectory
    logger_->info(
        "Cleared {} old trajectories. First trajectory now begins at {}s",
        originalNumTrajectories - trajectories_.size(),
        trajectories_.begin()->first);
}

bool CubicTaskTrajectory::isTrajectoryRunning() {
    logger_->debug("isTrajectoryRunning");
    if (isFirstTrajectory_) return false;
    return ranges_.back() > lastEvaluationTime_;
}

TaskSignals CubicTaskTrajectory::getTrajectoryPoint(double Tp) {
    logger_->debug("getTrajectoryPoint {}", Tp);
    std::scoped_lock<std::mutex> lck(mtx_);

    // Check for invalid function calls
    if (lastEvaluationTime_ > Tp) {
        throw std::runtime_error("Going backwards in a trajectory is not permitted");
    }
    if (!isInitialPoseSet_) {
        throw std::runtime_error(
            "No trajectories have been generated, cannot retrieve trajectory point");
    }

    TaskSignals result;
    lastEvaluationTime_ = Tp;

    if (trajectories_.empty()) {  // no trajectory has been defined but initial pose has been set
        result.pose = initialPose_;
        result.velocity = zeroTaskVelocity_;
        result.acceleration = zeroTaskAcceleration_;
        return result;
    }

    // Determine which trajectory this evaluation time belongs to
    auto it = trajectories_.upper_bound(Tp);  // Gets first iterator that is ordered after value, Tp
    it--;  // Want iterator before the one that is ordered after Tp
    currentTrajectory_ = it->second;

    // Evaluate the current trajectory at this time
    result.pose = evaluateTaskPose(currentTrajectory_, Tp);
    result.velocity = evaluateTaskVelocity(currentTrajectory_, Tp);

    // TODO(jplayang): This can be removed once we make sure the acceleration is continuous and can
    // end at 0
    if (Tp > ranges_.back()) {  // Past last point in trajectory
        result.acceleration = zeroTaskAcceleration_;
        return result;
    }
    result.acceleration = evaluateTaskAcceleration(currentTrajectory_, Tp);
    return result;
}

double CubicTaskTrajectory::getMovementTime(
    const double& point1,
    const double& point2,
    const std::size_t& taskParameterIndex) {
    double velocityRestrictedTime =
        std::fabs(point2 - point1) * 1.5 / maxVelocity_[taskParameterIndex] + 1;
    double accelerationRestrictedTime =
        std::sqrt(std::fabs(point2 - point1) / (6.0 / maxAcceleration_[taskParameterIndex])) + 1;
    return std::max(velocityRestrictedTime, accelerationRestrictedTime);
}

double CubicTaskTrajectory::getMovementTime(
    const Orientation& point1,
    const Orientation& point2) {
    const Rotation relativeRotation(multiply(invert(point1), point2));
    double velocityRestrictedTime = angularVelocityFromRotation(relativeRotation).norm() * 1.5 *
            sqrt(3.0) / maxVelocity_.raw().tail(3).norm() + 1;  // sqrt(3) to account for 3 dims
    double accelerationRestrictedTime =
        std::sqrt(angularVelocityFromRotation(relativeRotation).norm() /
            (6.0 * sqrt(3.0) / maxAcceleration_.raw().tail(3).norm())) + 1;
    return std::max(velocityRestrictedTime, accelerationRestrictedTime);
}

TaskPose CubicTaskTrajectory::evaluateTaskPose(
    const TaskTrajectory& trajectory,
    const double& evaluationTime) {
    return TaskPose(
        Eigen::Vector3d(
            {trajectory.linear->at(0).evaluate(evaluationTime, 0).value(),
             trajectory.linear->at(1).evaluate(evaluationTime, 0).value(),
             trajectory.linear->at(2).evaluate(evaluationTime, 0).value()}),
        trajectory.angular->evaluateOrientation(evaluationTime).value());
}

TaskVelocity CubicTaskTrajectory::evaluateTaskVelocity(
    const TaskTrajectory& trajectory,
    const double& evaluationTime) {
    Eigen::Vector3d angularVelocity = trajectory.angular->evaluate(evaluationTime, 1).value();

    return TaskVelocity(
        {trajectory.linear->at(0).evaluate(evaluationTime, 1).value(),  // x linear velocity
         trajectory.linear->at(1).evaluate(evaluationTime, 1).value(),  // y linear velocity
         trajectory.linear->at(2).evaluate(evaluationTime, 1).value(),  // z linear velocity
         angularVelocity[0],
         angularVelocity[1],
         angularVelocity[2]});
}

TaskAcceleration CubicTaskTrajectory::evaluateTaskAcceleration(
    const TaskTrajectory& trajectory,
    const double& evaluationTime) {
    Eigen::Vector3d angularAcceleration = trajectory.angular->evaluate(evaluationTime, 2).value();

    return TaskAcceleration(
        {trajectory.linear->at(0).evaluate(evaluationTime, 2).value(),  // x linear acceleration
         trajectory.linear->at(1).evaluate(evaluationTime, 2).value(),  // y linear acceleration
         trajectory.linear->at(2).evaluate(evaluationTime, 2).value(),  // z linear acceleration
         angularAcceleration[0],
         angularAcceleration[1],
         angularAcceleration[2]});
}

}  // namespace crf::control::trajectorygenerator
