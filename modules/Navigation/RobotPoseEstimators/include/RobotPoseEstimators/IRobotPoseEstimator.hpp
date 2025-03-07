#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <boost/optional.hpp>

#include "CommonInterfaces/IInitializable.hpp"
#include "Types/TaskTypes/TaskSignals.hpp"

namespace crf {
namespace applications {
namespace robotposeestimator {

class IRobotPoseEstimator : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IRobotPoseEstimator() = default;
    /*
     * Returns current estimated position of the robot relative to the starting point
     * in the form of xyz position and the Euler angles:
     *   [x, y, z, roll, pitch, yaw]
     * Returns boost::zero if estimation unsuccessful
     */
    virtual boost::optional<utility::types::TaskPose> getPosition() = 0;
    /*
     * Returns current estimated velocity of the robot with respect to its moving center
     * meaning the change in its task position
     *   [xdot, ydot, zdot, rolldot, pitchdot, yawdot]
     * Returns boost::zero if estimation unsuccessful
     */
    virtual boost::optional<utility::types::TaskVelocity> getVelocity() = 0;
    /*
     * Returns current estimated acceleration of the robot with respect to its moving center
     * meaning the change in its task velocity
     *   [xdotdot, ydotdot, zdotdot, rolldotdot, pitchdotdot, yawdotdot]
     * Returns boost::zero if estimation unsuccessful
     */
    virtual boost::optional<utility::types::TaskAcceleration> getAcceleration() = 0;
};

}  // namespace robotposeestimator
}  // namespace applications
}  // namespace crf
