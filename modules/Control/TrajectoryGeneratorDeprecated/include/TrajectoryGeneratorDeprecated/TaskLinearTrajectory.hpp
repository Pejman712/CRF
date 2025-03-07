/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>
#include <boost/optional.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/rotational_interpolation.hpp>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"
#include "TrajectoryGeneratorDeprecated/TrajectoryData.hpp"
#include "TrajectoryGeneratorDeprecated/ITaskTrajectoryGenerator.hpp"

namespace crf::control::trajectorygeneratordeprecated {

class TaskLinearTrajectory : public ITaskTrajectoryGenerator {
 public:
    TaskLinearTrajectory(utility::types::TaskVelocity maxVelocity,
        utility::types::TaskAcceleration maxAcceleration,
        float timeStep);
    ~TaskLinearTrajectory() override;

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
    utility::types::TaskVelocity maxVelocity_;
    utility::types::TaskAcceleration maxAcceleration_;
    std::unique_ptr<KDL::Trajectory_Composite> traj_;

    double timeStep_;
    bool isInitialized_;

    KDL::Frame positionToFrame(const utility::types::TaskPose& vector) const;

    bool computeTrajectory(const std::vector<KDL::Frame> &path);

    void destroy();
};

}  // namespace crf::control::trajectorygeneratordeprecated
