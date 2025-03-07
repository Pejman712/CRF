/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Chelsea Davidson CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */
#pragma once

#include <memory>
#include <vector>
#include <map>

#include "EventLogger/EventLogger.hpp"
#include "GeometricMethods/CubicOrientationSpline/CubicOrientationSpline.hpp"
#include "GeometricMethods/CubicPolynomial/CubicPolynomial.hpp"
#include "Rotation/Rotation.hpp"
#include "TrajectoryGenerator/ITaskTrajectoryGenerator.hpp"
#include "Types/Types.hpp"

using crf::math::geometricmethods::CubicPolynomial;
using crf::math::geometricmethods::CubicOrientationSpline;
using crf::math::rotation::Orientation;
using crf::math::rotation::angularVelocityFromRotation;
using crf::utility::types::areAlmostEqual;

namespace crf::control::trajectorygenerator {

/**
 * @ingroup group_cubic_task_trajectory
 * @brief Implementation of ITaskTrajectoryGenerator. Constructs a Cubic Task Trajectory object
 *        which starts at an initial pose (set by the user), and interpolates between a set of
 *        appended TaskPoses. The trajectory is constructed such that it doesn't exceed the
 *        specified maximum TaskVelocity and TaskAcceleration.
 *
 * @param maxVelocity The maximum TaskVelocity allowed in the trajectory
 * @param maxAcceleration The maximum TaskAcceleration allowed in the trajectory
 *
 * @details This class is thread safe
 */
class CubicTaskTrajectory : public ITaskTrajectoryGenerator {
 public:
    CubicTaskTrajectory(TaskVelocity maxVelocity, TaskAcceleration maxAcceleration);
    CubicTaskTrajectory() = delete;
    ~CubicTaskTrajectory() override;

    void setInitialPose(const TaskPose& initialPose);
    void append(const std::vector<TaskPose>& path);
    void setProfileVelocity(const TaskVelocity& newProfileVelocity);
    void setProfileAcceleration(const TaskAcceleration& newProfileAcceleration);
    void reset();
    void clearMemory();
    bool isTrajectoryRunning();
    TaskSignals getTrajectoryPoint(double Tp);

 private:
    class TaskTrajectory {
     public:
        // These fields have to be ptrs so that they can be default constructed
        std::shared_ptr<std::array<CubicPolynomial, 3>> linear;
        std::shared_ptr<CubicOrientationSpline> angular;

        void clear() {
            linear.reset();
            angular.reset();
        }
    };

    // Trajectory parameters
    TaskPose initialPose_;
    TaskVelocity maxVelocity_;
    TaskAcceleration maxAcceleration_;

    // Global trajectory path points (point has a pose, time, velocity, and acceleration)
    std::vector<TaskPose> poses_;  // poses of the path points
    std::vector<double> ranges_;   // time instances of the path points

    // Trajectories
    std::map<double, TaskTrajectory> trajectories_;  // map storing each trajectory & its start time
    TaskTrajectory currentTrajectory_;  // trajectory currently being evaluated
    TaskTrajectory lastTrajectory_;     // the most recent trajectory added to the trajectory map

    // Evaluation state information
    std::atomic<double> lastEvaluationTime_;  // time trajectory (currentTrajectory_) was evaluated
    std::atomic<bool> isFirstTrajectory_;
    std::atomic<bool> isInitialPoseSet_;

    std::mutex mtx_;
    std::mutex profileValuesMtx_;
    const double epsilon_ = 1e-12;  // Defines small change. Used to check pose duplicates in path
    TaskVelocity zeroTaskVelocity_ = TaskVelocity({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    TaskAcceleration zeroTaskAcceleration_ = TaskAcceleration({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    crf::utility::logger::EventLogger logger_;

    /**
     * @brief Calculates the time it will take to get from point1 to point2 if the velocity and
     * acceleration limits are satisfied.
     *
     * @param point1 The first point
     * @param point2 The second point
     * @param taskParameterIndex The index corresponding to the task parameter (0 for x, 1 for y, 2
     * for z).
     *
     * @return double The maximum of the time taken when the velocity limit is satisfied and the
     * time taken when the acceleration limit is satisfied.
     */
    double getMovementTime(const double& point1, const double& point2,
        const std::size_t& taskParameterIndex);

    /**
     * @brief Calculates the time it will take to get from point1 to point2 if the velocity and
     * acceleration limits are satisfied.
     *
     * @param point1 The first orientation point
     * @param point2 The second orientation point
     *
     * @return double The maximum of the time taken when the velocity limit is satisfied and the
     * time taken when the acceleration limit is satisfied.
     */
    double getMovementTime(const Orientation& point1, const Orientation& point2);

    /**
     * @brief Evaluates each component trajectory of the TaskTrajectory (linear and angular)
     * at this evaluationTime and returns the resultant pose.
     *
     * @param trajectory The TaskTrajectory being evaluated.
     * @param evaluationTime The time at which the trajectory is being evaluated.
     *
     * @return TaskPose The TaskPose at this evaluationTime in the trajectory.
     */
    TaskPose evaluateTaskPose(const TaskTrajectory& trajectory, const double& evaluationTime);

    /**
     * @brief Evaluates each component trajectory of the TaskTrajectory (linear and angular)
     * at this evaluationTime and returns the resultant TaskVelocity.
     *
     * @param trajectory The TaskTrajectory being evaluated.
     * @param evaluationTime The time at which the trajectory is being evaluated.
     *
     * @return TaskVelocity The TaskVelocity at this evaluationTime in the trajectory.
     */
    TaskVelocity evaluateTaskVelocity(const TaskTrajectory& trajectory,
        const double& evaluationTime);

    /**
     * @brief Evaluates each component trajectory of the TaskTrajectory (linear and angular)
     * at this evaluationTime and returns the resultant TaskAcceleration.
     *
     * @param trajectory The TaskTrajectory being evaluated.
     * @param evaluationTime The time at which the trajectory is being evaluated.
     *
     * @return TaskAcceleration The TaskAcceleration at this evaluationTime in the trajectory.
     */
    TaskAcceleration evaluateTaskAcceleration(const TaskTrajectory& trajectory,
        const double& evaluationTime);
};

}  // namespace crf::control::trajectorygenerator
