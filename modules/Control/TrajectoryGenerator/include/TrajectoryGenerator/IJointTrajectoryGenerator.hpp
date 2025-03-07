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

namespace crf::control::trajectorygenerator {

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointSignals;

/**
 * @brief Interface from which all the trajectory generators are based off
 *
 */
class IJointTrajectoryGenerator {
 public:
    virtual ~IJointTrajectoryGenerator() = default;

    /**
     * @brief Method to set the initial joint poisiton of the trajectory
     *
     * @param initialPosition The robot's position at the start of the trajectory
     */
    virtual void setInitialPosition(const JointPositions& initialPosition) = 0;

    /**
     * @brief Method to append a vector of geometric points in the n dimensions of the robot
     *
     * @param std::vector<std::vector<double>> Geometric points to
     * append to the trajectory
     */
    virtual void append(const std::vector<JointPositions>& path) = 0;

    /**
     * @brief Method to change the velocity used for the trajectory profile. This will not
     * affect trajectories already generated
     *
     * @param JointVelocities New profile velocity
     */
    virtual void setProfileVelocity(const JointVelocities& path) = 0;

    /**
     * @brief Method to change the acceleration used for the trajectory profile. This will not
     * affect trajectories already generated
     *
     * @param JointAccelerations New profile acceleration
     */
    virtual void setProfileAcceleration(const JointAccelerations& path) = 0;

    /**
     * @brief Method to erase all the saved points of the trajectory
     *
     */
    virtual void reset() = 0;

    /**
     * @brief Method to erase all the already used points with a safety margin to save memory
     * and resources.
     *
     */
    virtual void clearMemory() = 0;

    /**
     * @brief Method to let the user know if the last point that was evaluated is inside the
     * trajectory.
     * @return true if the last evaluated point is inside the range of a trajectory
     * @return false if it's after this range
     */
    virtual bool isTrajectoryRunning() = 0;

    /**
     * @brief Get the Trajectory Point object for a certain time Tp
     *
     * @param Tp Time for which we want the trajectory point
     * @return ControllerData<std::vector<double>> The value of the four derivatives
     * for each n dimensions
     */
    virtual JointSignals getTrajectoryPoint(double Tp) = 0;
};

}  // namespace crf::control::trajectorygenerator
