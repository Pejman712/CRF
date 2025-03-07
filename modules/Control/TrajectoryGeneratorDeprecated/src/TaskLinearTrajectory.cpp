/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <exception>
#include <list>
#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>
#include <boost/optional.hpp>

#include "TrajectoryGeneratorDeprecated/TaskLinearTrajectory.hpp"
#include "TrajectoryGeneratorDeprecated/TrajectoryData.hpp"

#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>

using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;

namespace crf::control::trajectorygeneratordeprecated {

TaskLinearTrajectory::TaskLinearTrajectory(TaskVelocity maxVelocity,
    TaskAcceleration maxAcceleration,
    float timeStep) :
    logger_("TaskLinearTrajectory"),
    timeStep_(static_cast<double>(timeStep)),
    isInitialized_(false) {
        logger_->debug("CTor");
        maxVelocity_ = maxVelocity;
        maxAcceleration_ = maxAcceleration;
    }

TaskLinearTrajectory::~TaskLinearTrajectory() {
    logger_->debug("DTor");
    destroy();
}

void TaskLinearTrajectory::destroy() {
    logger_->debug("destroy");
    if (isInitialized_) {
        traj_->Destroy();
    }
}

bool TaskLinearTrajectory::computeTrajectory(const std::vector<TaskPose> &path)  {
    logger_->debug("computeTrajectory()");
    if (path.size() < 2) {
        logger_->warn("Trajectory must have at least 2 points and it has {}", path.size());
        return false;
    }
    std::vector<KDL::Frame> frame_path;
    for (unsigned int i=0; i < path.size(); i++) {
        frame_path.push_back(positionToFrame(path[i]));
    }
    return computeTrajectory(frame_path);
}

bool TaskLinearTrajectory::computeTrajectory(const std::vector<KDL::Frame> &path) {
    logger_->debug("computeTrajectory()");
    destroy();
    traj_ = std::make_unique<KDL::Trajectory_Composite>();
    if (path.size() < 2) {
        logger_->warn("Trajectory must have at least 2 frames and it has {}", path.size());
        return false;
    }
    // All the objects created here are destroyed by the traj_ destructor.
    for (unsigned int i=1; i < path.size(); i++) {
         KDL::RotationalInterpolation_SingleAxis* rotation =
            new KDL::RotationalInterpolation_SingleAxis();
        KDL::Path* path_line =
            new KDL::Path_Line(
                path[i-1],
                path[i],
                rotation,
                1);
        float minVelocity = maxVelocity_[0];
        float minAcceleration = maxAcceleration_[0];
        for (int i=1; i < 3; i++) {
            minVelocity = minVelocity > maxVelocity_[i] ? maxVelocity_[i] : minVelocity;
            minAcceleration = minAcceleration > maxAcceleration_[i] ?
                maxAcceleration_[i] : minAcceleration;
        }
        KDL::VelocityProfile* prof = new KDL::VelocityProfile_Trap(
                static_cast<double>(minVelocity),
                static_cast<double>(minAcceleration));
        prof->SetProfile(0, path_line->PathLength());
        KDL::Trajectory_Segment* segment = new KDL::Trajectory_Segment(
            path_line,
            prof);
        traj_->Add(segment);
    }
    isInitialized_ = true;
    return true;
}


boost::optional<float> TaskLinearTrajectory::getDuration() const {
    if (!isInitialized_) return boost::none;
    return static_cast<float>(traj_->Duration());
}

boost::optional<TaskPose> TaskLinearTrajectory::getTaskPose(
    float time) const {
    if (!isInitialized_) return boost::none;
    if ((time < 0) || (time > getDuration())) return boost::none;

    KDL::Frame frame = traj_->Pos(static_cast<double>(time));
    Eigen::Vector3d xyzPos({frame.p(0), frame.p(1), frame.p(2)});
    Eigen::Matrix3d rotMatrix;
    rotMatrix << frame.M(0, 0), frame.M(0, 1), frame.M(0, 2),
                 frame.M(1, 0), frame.M(1, 1), frame.M(1, 2),
                 frame.M(2, 0), frame.M(2, 1), frame.M(2, 2);
    return TaskPose(xyzPos, rotMatrix);
}

boost::optional<TaskVelocity> TaskLinearTrajectory::getTaskVelocity(
    float time) const {
    if (!isInitialized_) return boost::none;
    if ((time < 0) || (time > getDuration())) return boost::none;
    TaskVelocity velocity;
    auto twist = traj_->Vel(static_cast<double>(time));
    for (int i=0; i < 6; i++) {
        velocity[i] = twist(i);
    }
    return velocity;
}

boost::optional<TaskAcceleration> TaskLinearTrajectory::getTaskAcceleration(
    float time) const {
    if (!isInitialized_) return boost::none;
    if ((time < 0) || (time > getDuration())) return boost::none;
    TaskAcceleration acc;
    auto twist = traj_->Acc(static_cast<double>(time));
    for (int i=0; i < 6; i++) {
        acc[i] = twist(i);
    }
    return acc;
}


boost::optional<TaskTrajectoryData> TaskLinearTrajectory::getTaskTrajectory() const {  // NOLINT
    logger_->debug("getTrajectory");
    TaskTrajectoryData result{};
    if (!isInitialized_) {
        logger_->error("Wrong initialization - Returning empty vector");
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

KDL::Frame TaskLinearTrajectory::positionToFrame(const TaskPose& vector) const {
    crf::math::rotation::CardanXYZ cardanXYZ = vector.getCardanXYZ();
    Eigen::Vector3d position = vector.getPosition();
    return KDL::Frame(
        KDL::Rotation::RPY(
            cardanXYZ[0],
            cardanXYZ[1],
            cardanXYZ[2]),
        KDL::Vector(
            position[0],
            position[1],
            position[2]));
}

}  // namespace crf::control::trajectorygeneratordeprecated
