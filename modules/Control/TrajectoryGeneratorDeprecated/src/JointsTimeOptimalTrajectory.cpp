/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <exception>
#include <cmath>
#include <list>
#include <vector>
#include <string>
#include <memory>
#include <Eigen/Core>
#include <boost/optional.hpp>

#include "EventLogger/EventLogger.hpp"
#include "TrajectoryGeneratorDeprecated/JointsTimeOptimalTrajectory.hpp"
#include "TrajectoryGeneratorDeprecated/TrajectoryData.hpp"

#include "TimeOptimalTrajectoryGenerator/TrajectoryHelper.hpp"
#include "TimeOptimalTrajectoryGenerator/Path.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

namespace crf::control::trajectorygeneratordeprecated {

JointsTimeOptimalTrajectory::JointsTimeOptimalTrajectory(JointVelocities maxVelocity,
    JointAccelerations maxAcceleration,
    float timeStep,
    float maxDeviation):
    logger_("JointsTimeOptimalTrajectory"),
    timeStep_(static_cast<double>(timeStep)),
    maxDeviation_(static_cast<double>(maxDeviation)),
    isInitialized_(false) {
    logger_->debug("CTor");
    if (maxVelocity.size() != maxAcceleration.size()) {
        throw std::runtime_error("Input variable dimensions are different");
    }
    dimNumber_ = maxVelocity.size();
    maxVelocity_.resize(dimNumber_);
    maxAcceleration_.resize(dimNumber_);
    for (unsigned int i=0; i < dimNumber_; i++) {
        maxVelocity_(i) = maxVelocity[i];
        maxAcceleration_(i) = maxAcceleration[i];
    }
}

JointsTimeOptimalTrajectory::~JointsTimeOptimalTrajectory() {
    logger_->debug("DTor");
}

bool JointsTimeOptimalTrajectory::computeTrajectory(const std::vector<JointPositions> &path) {
    logger_->debug("computeTrajectory");
    if (path.size() < 2) {
        logger_->error("The path is too small");
        isInitialized_ = false;
        return false;
    }
    JointPositions previous = path[0];
    std::vector<JointPositions> cleanPath;
    cleanPath.push_back(previous);
    for (std::size_t i = 1; i < path.size(); i++) {
        if (!areAlmostEqual(previous, path[i])) cleanPath.push_back(path[i]);
        previous = path[i];
    }
    std::list<Eigen::VectorXd> pathList;
    Eigen::VectorXd pathPoint;
    pathPoint.resize(dimNumber_);
    if (cleanPath.size() < 2) {
        logger_->error("The path is too small");
        isInitialized_ = false;
        return false;
    }
    for (unsigned int i=0; i < cleanPath.size(); i++) {
        if (cleanPath[i].size() != dimNumber_) {
            logger_->error("The dimension of the pose number {} is different than the one "\
            "defined in the constructor", i);
            isInitialized_ = false;
            return false;
        }
        for (unsigned int j=0; j < dimNumber_; j++) {
            pathPoint(j) = cleanPath[i][j];
        }
        pathList.push_back(pathPoint);
    }
    trajecObj_.reset(new Trajectory(Path(pathList, maxDeviation_),
        maxVelocity_,
        maxAcceleration_,
        timeStep_));
    if (!trajecObj_->isValid()) {
        logger_->error("The trajectory can't be generated");
        return false;
    } else if (std::isnan(static_cast<float>(trajecObj_->getDuration()))) {
        logger_->error("Generated trajectory is not valid: {}",
            static_cast<float>(trajecObj_->getDuration()));
        return false;
    } else {
        logger_->info("Trajectory generated");
        isInitialized_ = true;
        return true;
    }
}

boost::optional<float> JointsTimeOptimalTrajectory::getDuration() const {
    if (!isInitialized_) {
        logger_->error("Wrong initialization - Returning empty vector");
        return boost::none;
    }
    if (!trajecObj_->isValid()) {
        logger_->error("The trajectory can't be generated - Returning empty value");
        return boost::none;
    }
    float duration = static_cast<float>(trajecObj_->getDuration());
    if (std::isnan(duration))return boost::none;
    return duration;
}

boost::optional<JointPositions> JointsTimeOptimalTrajectory::getJointPositions(float time) const {
    if (!isInitialized_) {
        logger_->error("Wrong initialization - Returning empty vector");
        return boost::none;
    }
    if (!trajecObj_->isValid()) {
        logger_->error("The trajectory can't be generated - Returning empty vector");
        return boost::none;
    }
    if (time > static_cast<float>(trajecObj_->getDuration())) {
        logger_->error("The time is bigger than the duration of the trajectory "\
            "- Returning empty vector");
        return boost::none;
    }
    if (time < 0) {
        logger_->error("The time is negative - Returning empty vector");
        return boost::none;
    }
    Eigen::VectorXd position = trajecObj_->getPosition(static_cast<double>(time));
    JointPositions result(dimNumber_);

    for (unsigned int i=0; i < dimNumber_; i++) {
        result[i] = position(i);
    }
    return result;
}

boost::optional<JointVelocities> JointsTimeOptimalTrajectory::getJointVelocities(float time) const {
    if (!isInitialized_) {
        logger_->error("Wrong initialization - Returning empty vector");
        return boost::none;
    }
    if (!trajecObj_->isValid()) {
        return boost::none;
    }
    if (time > static_cast<float>(trajecObj_->getDuration())) {
        logger_->error("The time is bigger than the duration of the trajectory "\
            "- Returning empty vector");
        return boost::none;
    }
    if (time < 0) {
        logger_->error("The time is negative - Returning empty vector");
        return boost::none;
    }
    Eigen::VectorXd velocity = trajecObj_->getVelocity(static_cast<double>(time));
    JointVelocities result(dimNumber_);

    for (unsigned int i=0; i < dimNumber_; i++) {
        result[i] = velocity(i);
    }
    return result;
}

boost::optional<JointAccelerations> JointsTimeOptimalTrajectory::getJointAccelerations(
    float time) const {
    logger_->warn("getAcceleration: operation not supported");
    return boost::none;
}

boost::optional<JointForceTorques> JointsTimeOptimalTrajectory::getJointForceTorques(
    float time) const {
    logger_->warn("getTorque: operation not supported");
    return boost::none;
}

boost::optional<JointsTrajectoryData> JointsTimeOptimalTrajectory::getJointsTrajectory() const {
    logger_->debug("getTrajectory");
    JointsTrajectoryData result{};
    if (!isInitialized_) {
        logger_->error("Wrong initialization - Returning empty vector");
        return boost::none;
    }
    if (!trajecObj_->isValid()) {
        return boost::none;
    }
    float duration = getDuration().get();
    logger_->info("duration {}", duration);
    int pointsNumber = duration/timeStep_;
    logger_->info("pointsNumber {}", pointsNumber);
    for (float t=0.0; t < duration; t+=timeStep_) {
        result.time.push_back(t);
        result.position.push_back(getJointPositions(t).get());
        result.velocity.push_back(getJointVelocities(t).get());
    }
    return result;
}

}  // namespace crf::control::trajectorygeneratordeprecated
