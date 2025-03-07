#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>

#include <boost/optional.hpp>

#include "CommonInterfaces/IInitializable.hpp"
#include "Types/TaskTypes/TaskVelocity.hpp"
#include "Types/TaskTypes/TaskPose.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"

namespace crf::actuators::robotbase {

class IRobotBase : public utility::commoninterfaces::IInitializable {
 public:
    ~IRobotBase() override = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;
    /* 
     * Returns:
     *  - Task position transformation in meters
     *  - boost::none on failure (e.g. inverse kinematics failed or unavailable)
     */
    virtual boost::optional<utility::types::TaskPose> getTaskPose() = 0;
    /* 
     * Returns:
     *  - Task velocity with respect to the base frame in meters and radians
     *  - boost::none on failure (e.g. inverse kinematics failed or unavailable)
     */
    virtual boost::optional<utility::types::TaskVelocity> getTaskVelocity() = 0;
    /* 
     * Returns:
     *  - motors velocity
     *  - boost::none on failure (e.g. unavailable)
     */
    virtual boost::optional<std::vector<float> > getMotorsVelocities() = 0;
    /* 
     * Returns:
     *  - motors current
     *  - boost::none on failure (e.g. unavailable)
     */
    virtual boost::optional<std::vector<float> > getMotorsCurrent() = 0;
    /*
     * Makes the robot move with the desired velocity in space.
     * Returns true if possible
     * Returns false if it failed (e.g. inverse kinematics failure)
     */
    virtual bool setTaskVelocity(const utility::types::TaskVelocity& velocity) = 0;
    /*
     * Makes the robot move the motors with desired velocity.
     * Returns true if possible
     * Returns false if it failed
     */
    virtual bool setWheelsVelocity(const std::vector<float> velocity) = 0;
    /*
     * Cancel all movements and stops the robot base
     * Exact behavior is implementation-defined
     * Returns true if possible
     * Returns false if it failed (e.g. communication failure)
     */
    virtual bool stopBase() = 0;

    /*
     * Returns true if one of the motors is not enabled
     */
    virtual bool errorActive() = 0;
    /*
     * Returns true if all the motors are enabled at the end of the acknolwedge
     */
    virtual bool acknowledgeError() = 0;
    /**
     * @brief Get the robot base configuration object
     * 
     * @return std::shared_ptr<RobotBaseConfiguration> 
     */
    virtual std::shared_ptr<RobotBaseConfiguration> getConfiguration() = 0;

    virtual bool setMaximumWheelsAcceleration(float acceleration) = 0;
    virtual bool setMaximumWheelsVelocity(float velocity) = 0;
    virtual bool setMaximumTaskVelocity(const utility::types::TaskVelocity& velocity) = 0;

    virtual float getMaximumWheelsAcceleration() const = 0;
    virtual float getMaximumWheelsVelocity() const = 0;
    virtual utility::types::TaskVelocity getMaximumTaskVelocity() const = 0;
};

}  // namespace crf::actuators::robotbase
