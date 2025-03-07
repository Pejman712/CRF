/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

// #define EIGEN_DONT_VECTORIZE
// #define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <exception>
#include <list>
#include <vector>
#include <string>
#include <memory>
#include <Eigen/Core>
#include <boost/optional.hpp>

#include "EventLogger/EventLogger.hpp"
#include "TrajectoryGeneratorDeprecated/TaskTimeOptimalTrajectory.hpp"
#include "TrajectoryGeneratorDeprecated/TrajectoryData.hpp"

#include "TimeOptimalTrajectoryGenerator/TrajectoryHelper.hpp"
#include "TimeOptimalTrajectoryGenerator/Path.hpp"

using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;

namespace crf::control::trajectorygeneratordeprecated {

TaskTimeOptimalTrajectory::TaskTimeOptimalTrajectory(TaskVelocity maxVelocity,
    TaskAcceleration maxAcceleration,
    float timeStep,
    float maxDeviation):
    logger_("TaskTimeOptimalTrajectory"),
    timeStep_(static_cast<double>(timeStep)),
    maxDeviation_(static_cast<double>(maxDeviation)),
    isInitialized_(false) {
    logger_->debug("CTor");
    dimNumber_ = 6;
    maxVelocity_.resize(dimNumber_);
    maxAcceleration_.resize(dimNumber_);
    for (int i=0; i < dimNumber_; i++) {
        maxVelocity_(i) = static_cast<double>(maxVelocity[i]);
        maxAcceleration_(i) = static_cast<double>(maxAcceleration[i]);
    }
}

TaskTimeOptimalTrajectory::~TaskTimeOptimalTrajectory() {
    logger_->debug("DTor");
}

bool TaskTimeOptimalTrajectory::computeTrajectory(const std::vector<TaskPose> &path) {
    logger_->debug("computeTrajectory");
    std::list<Eigen::VectorXd> pathList;
    Eigen::VectorXd pathPoint;
    pathPoint.resize(dimNumber_);
    if  (path.size() < 2) {
        logger_->error("The path is too small");
        isInitialized_ = false;
        return false;
    }
    for (unsigned int i=0; i < path.size(); i++) {
        pathPoint.segment<3>(0) = path[i].getPosition();
        for (int y = 0; y < 3; y++) {
            pathPoint(y + 3) = path[i].getCardanXYZ()[y];
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
    } else {
        logger_->info("Trajectory generated");
        isInitialized_ = true;
        return true;
    }
}

boost::optional<float> TaskTimeOptimalTrajectory::getDuration() const {
    if (!isInitialized_) {
        logger_->error("Wrong initialization - Returning empty vector");
        return boost::none;
    }
    if (!trajecObj_->isValid()) {
        logger_->error("The trajectory can't be generated - Returning empty value");
        return boost::none;
    }
    return static_cast<float>(trajecObj_->getDuration());
}

boost::optional<TaskPose> TaskTimeOptimalTrajectory::getTaskPose(
    float time) const {
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
    Eigen::VectorXf pose = trajecObj_->getPosition(static_cast<double>(time)).cast<float>();
    TaskPose result(
        {pose(0), pose(1), pose(2)},
        crf::math::rotation::CardanXYZ({pose(3), pose(4), pose(5)}));
    return result;
}

boost::optional<TaskVelocity> TaskTimeOptimalTrajectory::getTaskVelocity(
    float time) const {
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
    Eigen::VectorXf velocity = trajecObj_->getVelocity(static_cast<double>(time)).cast<float>();
    TaskVelocity result({
        velocity(0), velocity(1), velocity(2),
        velocity(3), velocity(4), velocity(5)
    });
    return result;
}

boost::optional<TaskAcceleration> TaskTimeOptimalTrajectory::getTaskAcceleration(
    float time) const {
    logger_->debug("getTaskAcceleration");
    logger_->error("Not available task acceleration");
    return boost::none;
}

boost::optional<TaskTrajectoryData> TaskTimeOptimalTrajectory::getTaskTrajectory() const {  // NOLINT
    logger_->debug("getTrajectory");
    TaskTrajectoryData result{};
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
        result.position.push_back(getTaskPose(t).get());
        result.velocity.push_back(getTaskVelocity(t).get());
    }
    return result;
}

}  // namespace crf::control::trajectorygeneratordeprecated
