/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>
#include <boost/optional.hpp>

#include "Types/Types.hpp"
#include "TrajectoryGeneratorDeprecated/TrajectoryData.hpp"

namespace crf::control::trajectorygeneratordeprecated {

class IJointsTrajectoryGenerator {
 public:
    virtual ~IJointsTrajectoryGenerator() = default;
    /*
     * Returns:
     *  - True if the trajectory was calculated
     *  - False on failure (e.g. failed to calculate the trajecotry)
     */
    virtual bool computeTrajectory(const std::vector<utility::types::JointPositions> &path) = 0;
    /*
     * Returns:
     *  - The duration of the trajectory in seconds 
     *  - boost::none on failure (e.g. trajectory not calculated)
     */
    virtual boost::optional<float> getDuration() const = 0;
    /*
     * Returns:
     *  - The positions at the given time in seconds 
     *  - boost::none on failure (e.g. trajectory not calculated)
     */
    virtual boost::optional<utility::types::JointPositions> getJointPositions(
        float time) const = 0;
    /*
     * Returns:
     *  - The velocities at the given time in seconds 
     *  - boost::none on failure (e.g. trajectory not calculated)
     */
    virtual boost::optional<utility::types::JointVelocities> getJointVelocities(
        float time) const = 0;
    /*
     * Returns:
     *  - The acceleration at the given time in seconds 
     *  - boost::none on failure (e.g. trajectory not calculated)
     */
    virtual boost::optional<utility::types::JointAccelerations> getJointAccelerations(
        float time) const = 0;
    /*
     * Returns:
     *  - The acceleration at the given time in seconds 
     *  - boost::none on failure (e.g. trajectory not calculated)
     */
    virtual boost::optional<crf::utility::types::JointForceTorques> getJointForceTorques(float time) const = 0; // NOLINT
    /*
     * Returns:
     *  - The positions, velocities and times of the complete trajectory
     *  - boost::none on failure (e.g. trajectory not calculated)
     */
    virtual boost::optional<JointsTrajectoryData> getJointsTrajectory() const = 0;
};

}  // namespace crf::control::trajectorygeneratordeprecated
