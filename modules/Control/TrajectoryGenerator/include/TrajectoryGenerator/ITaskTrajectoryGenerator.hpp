/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>

#include "Types/Signals.hpp"
#include "Types/TaskTypes/TaskSpace.hpp"

using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskSignals;
using crf::utility::types::TaskSpace;

namespace crf::control::trajectorygenerator {

/**
 * @brief Interface from which all the trajectory generators are based off
 *
 */
class ITaskTrajectoryGenerator {
 public:
    virtual ~ITaskTrajectoryGenerator() = default;

     /**
     * @brief Method to set the initial pose of the robot
     *
     * @param initialPose The robot's pose at the start of the trajectory
     */
    virtual void setInitialPose(const TaskPose& initialPose) = 0;

    /**
     * @brief Method to append a vector of geometric points in the n dimensions of the robot
     *
     * @param path Geometric points to append to the trajectory
     */
    virtual void append(const std::vector<TaskPose>& path) = 0;

    /**
     * @brief Method to change the velocity used for the trajectory profile. This will not
     * affect trajectories already generated
     *
     * @param newProfileVelocity New profile velocity
     */
    virtual void setProfileVelocity(const TaskVelocity& newProfileVelocity) = 0;

    /**
     * @brief Method to change the acceleration used for the trajectory profile. This will not
     * affect trajectories already generated
     *
     * @param newProfileAcceleration New profile acceleration
     */
    virtual void setProfileAcceleration(const TaskAcceleration& newProfileAcceleration) = 0;

    /**
     * @brief Method to erase all the saved points of the trajectory
     *
     */
    virtual void reset() = 0;

    /**
     * @brief Method to erase all the already used points with a safety
     * margin to save memory and resources.
     *
     */
    virtual void clearMemory() = 0;

    /**
     * @brief Method to let the user know if the last point that was evaluated
     * is inside the trajectory.
     * @return true if the last evaluated point is inside the range of a trajectory
     * @return false if it's after this range or if no trajectory has been defined
     */
    virtual bool isTrajectoryRunning() = 0;

    /**
     * @brief Get the Trajectory Point object for a certain time Tp
     *
     * @param Tp Time for which we want the trajectory point
     * @return ControllerData<std::vector<double>> The value of the four derivatives
     * for each n dimensions
     */
    virtual TaskSignals getTrajectoryPoint(double Tp) = 0;
};

}  // namespace crf::control::trajectorygenerator
