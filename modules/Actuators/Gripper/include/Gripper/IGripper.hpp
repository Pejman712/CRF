/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include "CommonInterfaces/IInitializable.hpp"

#include <boost/optional.hpp>

namespace crf::actuators::gripper {

/**
 * @ingroup group_gripper
*/
class IGripper: public utility::commoninterfaces::IInitializable {
 public:
    enum GripperState {
        Gripper_Open = 0,
        Gripper_Closed = 100
    };
    ~IGripper() override = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief Get Position of the gripper
     *
     * @return position in percentages, 0 is open, 100 closed
     * @return boost::none on failure (e.g. API failure)
     */
    virtual boost::optional<float> getPosition() = 0;
    /**
     * @brief Get Position of the gripper
     *
     * @return true, if object detected while grasping
     * @return false, otherwise
     */
    virtual bool isGrasping() = 0;
    /**
     * @brief Makes the robot move the gripper to the desired position
     * Position is between [0 - open,100 - closed]
     * Velocity of each joint is implementation-defined.
     *
     * @return true if possible
     * @return false if it failed (e.g. values out of limits)
     */
    virtual bool setPosition(float percentage) = 0;
    virtual bool setPosition(GripperState state) = 0;
    /**
     * @brief Sets the grasping force of the gripper movements
     * Force is between [0 ,100 ]% of maximum value
     *
     * @return true if possible
     * @return false if it failed (e.g. values out of limits)
     */
    virtual bool setGraspingForce(float percentage) = 0;
    /**
     * @brief Cancel all movements and stops the gripper
     * E.g. by setting the velocities to 0 or by applying mechanical brakes.
     * Exact behavior is implementation-defined
     *
     * @return true if possible
     * @return false if it failed (e.g. communication failure)
     */
    virtual bool stopGripper() = 0;
    /**
     * @brief The gripper will start moving immediately with given speed, until it fully
     * closes/opens, Velocity is between [-100 - opens,100 - closes]
     *
     * @return true if possible
     * @return false if it failed (e.g. communication failure)
     */
    virtual bool setVelocity(float percentage) = 0;
};

}  // namespace crf::actuators::gripper
