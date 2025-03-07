/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <vector>
#include <memory>
#include <boost/optional.hpp>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"
#include "TrajectoryGeneratorDeprecated/TrajectoryData.hpp"
#include "TrajectoryGeneratorDeprecated/IJointsTrajectoryGenerator.hpp"
#include "TimeOptimalTrajectoryGenerator/TrajectoryHelper.hpp"

namespace crf::control::trajectorygeneratordeprecated {

/*
 * Generates a time optimal trajectory that exactly follows a given differentiable joint-space
 * path within given bounds on joint accelerations and velocities. Also a path preprocessing 
 * method is done to make nondifferentiable paths differentiable by adding circular blends.
 */
class JointsTimeOptimalTrajectory: public IJointsTrajectoryGenerator {
 public:
    JointsTimeOptimalTrajectory(utility::types::JointVelocities maxVelocity,
        utility::types::JointAccelerations maxAcceleration,
        float timeStep,
        float maxDeviation);
    ~JointsTimeOptimalTrajectory() override;

    bool computeTrajectory(const std::vector<utility::types::JointPositions> &path) override;
    boost::optional<float> getDuration() const override;
    boost::optional<utility::types::JointPositions> getJointPositions(
        float time) const override;
    boost::optional<utility::types::JointVelocities> getJointVelocities(
        float time) const override;
    boost::optional<utility::types::JointAccelerations> getJointAccelerations(
        float time) const override;
    boost::optional<crf::utility::types::JointForceTorques> getJointForceTorques(
        float time) const override;
    boost::optional<JointsTrajectoryData> getJointsTrajectory() const override;

 private:
    utility::logger::EventLogger logger_;
    std::unique_ptr<Trajectory> trajecObj_;
    Eigen::VectorXd maxVelocity_;
    Eigen::VectorXd maxAcceleration_;
    unsigned int dimNumber_;
    double timeStep_;
    double maxDeviation_;
    bool isInitialized_;
};

}  // namespace crf::control::trajectorygeneratordeprecated
