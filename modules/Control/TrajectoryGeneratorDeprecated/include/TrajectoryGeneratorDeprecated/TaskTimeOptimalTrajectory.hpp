/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

// #define EIGEN_DONT_VECTORIZE
// #define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <boost/optional.hpp>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"
#include "TrajectoryGeneratorDeprecated/TrajectoryData.hpp"
#include "TrajectoryGeneratorDeprecated/ITaskTrajectoryGenerator.hpp"
#include "TimeOptimalTrajectoryGenerator/TrajectoryHelper.hpp"

namespace crf::control::trajectorygeneratordeprecated {

/*
 * Generates a time optimal trajectory that exactly follows a given differentiable task-space
 * path within given bounds on task accelerations and velocities. Also a path preprocessing 
 * method is done to make nondifferentiable paths differentiable by adding circular blends.
 */
class TaskTimeOptimalTrajectory: public ITaskTrajectoryGenerator {
 public:
    TaskTimeOptimalTrajectory(utility::types::TaskVelocity maxVelocity,
        utility::types::TaskAcceleration maxAcceleration,
        float timeStep,
        float maxDeviation);
    ~TaskTimeOptimalTrajectory() override;

    bool computeTrajectory(const std::vector<utility::types::TaskPose> &path) override;
    boost::optional<float> getDuration() const override;
    boost::optional<utility::types::TaskPose> getTaskPose(
        float time) const override;
    boost::optional<utility::types::TaskVelocity> getTaskVelocity(
        float time) const override;
    boost::optional<utility::types::TaskAcceleration> getTaskAcceleration(
        float time) const override;
    boost::optional<TaskTrajectoryData> getTaskTrajectory() const override;

 private:
    utility::logger::EventLogger logger_;
    std::unique_ptr<Trajectory> trajecObj_;
    Eigen::VectorXd maxVelocity_;
    Eigen::VectorXd maxAcceleration_;
    int dimNumber_;
    double timeStep_;
    double maxDeviation_;
    bool isInitialized_;
};

}  // namespace crf::control::trajectorygeneratordeprecated
