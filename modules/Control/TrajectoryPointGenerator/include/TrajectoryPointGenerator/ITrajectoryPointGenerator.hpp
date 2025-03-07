/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>

#include <boost/optional.hpp>

#include "Types/TaskTypes/TaskTypes.hpp"

namespace crf::control::trajectorypointgenerator {

enum ControlMode {
    VELOCITY = 1,  // Velocity based algorithm with target velocity specified
    POSITION = 2   // Position based algorithm with target position
                   // and optionally target velocity specified
};

/* 
 * Class interacts with online task trajectory generator algorithm to generate the next trajectory point
 */
class ITrajectoryPointGenerator {
 public:
    virtual ~ITrajectoryPointGenerator() = default;
    /*
     * Returns:
     *  - The selected control mode (velocity or position based algorithm)
     *  - boost::none on failure (e.g. trajectory point not calculated)
     */
    virtual ControlMode getControlMode() const = 0;
    /*
     * Returns:
     *  - The position, velocity and acceleration of the latest trajectory point computed
     *  - boost::none on failure (e.g. trajectory point not calculated)
     */
    virtual boost::optional<
        utility::types::TaskTrajectoryData> getTaskTrajectoryPoint() const = 0;
    /*
     * Returns:
     *  - True if the target position was updated and passed the validity check
     *  - False on failure (e.g. invalid input)
     */
    virtual bool updatePositionTarget(
            const utility::types::TaskPose& targetPosition) = 0;
    /*
     * Returns:
     *  - True if the target velocity was updated and passed the validity check
     *  - False on failure (e.g. invalid input)
     */
    virtual bool updateVelocityTarget(
            const utility::types::TaskVelocity& targetVelocity) = 0;
    /*
     * Returns:
     *  - True if current state updated
     *  - False on failure (e.g. invalid state provided)
     */
    virtual bool updateCurrentState(
            const utility::types::TaskTrajectoryData& currentState) = 0;
    /*
     * Returns:
     *  - True if motion constraints have been successfully updated
     *  - False on failure (e.g. invalid constraints provided)
     */
    virtual bool updateMotionConstraints(
            const utility::types::TaskTrajectoryData& maximumState) = 0;
};

}  // namespace crf::control::trajectorypointgenerator
