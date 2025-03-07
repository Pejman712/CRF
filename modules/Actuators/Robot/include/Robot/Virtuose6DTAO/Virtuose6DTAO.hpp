/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <set>
#include <vector>

#include "Robot/IRobot.hpp"
#include "Haption/HaptionRaptorAPI/HaptionRaptorAPI.hpp"
#include "crf/expected.hpp"
#include "Types/Types.hpp"
#include "Robot/RobotConfiguration.hpp"
#include "Robot/Virtuose6DTAO/Virtuose6DTAOConfiguration.hpp"

namespace crf::actuators::robot {

class Virtuose6DTAO: public IRobot {
 public:
    Virtuose6DTAO(std::shared_ptr<devices::haption::IHaptionAPI> haptionInterface,
        const Virtuose6DTAOConfiguration& configuration);
    Virtuose6DTAO() = delete;
    ~Virtuose6DTAO() override;

    bool initialize() override;
    bool deinitialize() override;

    crf::expected<crf::utility::types::JointPositions> getJointPositions() override;
    crf::expected<crf::utility::types::JointVelocities> getJointVelocities() override;
    crf::expected<crf::utility::types::JointAccelerations> getJointAccelerations() override;
    crf::expected<crf::utility::types::JointForceTorques> getJointForceTorques() override;

    crf::expected<crf::utility::types::TaskPose> getTaskPose() override;
    crf::expected<crf::utility::types::TaskVelocity> getTaskVelocity() override;
    crf::expected<crf::utility::types::TaskAcceleration> getTaskAcceleration() override;
    crf::expected<crf::utility::types::TaskForceTorque> getTaskForceTorque() override;

    crf::expected<bool> setJointPositions(const bool& isSmoothTrajectory,
        const crf::utility::types::JointPositions& jointPositions,
        const crf::utility::types::JointVelocities& jointVelocities =
            crf::utility::types::JointVelocities(),
        const crf::utility::types::JointAccelerations& jointAccelerations =
            crf::utility::types::JointAccelerations()) override;
    crf::expected<bool> setJointVelocities(const bool& isSmoothTrajectory,
        const crf::utility::types::JointVelocities& jointVelocities,
        const crf::utility::types::JointAccelerations& jointAccelerations =
            crf::utility::types::JointAccelerations()) override;
    crf::expected<bool> setJointForceTorques(const bool& isSmoothTrajectory,
        const crf::utility::types::JointForceTorques& jointForceTorques) override;

    crf::expected<bool> setTaskPose(const bool& isSmoothTrajectory,
        const crf::utility::types::TaskPose& taskPose,
        const crf::utility::types::TaskVelocity& taskVelocity =
            crf::utility::types::TaskVelocity(),
        const crf::utility::types::TaskAcceleration& taskAcceleration =
            crf::utility::types::TaskAcceleration()) override;
    crf::expected<bool> setTaskVelocity(const bool& isSmoothTrajectory,
        const crf::utility::types::TaskVelocity& taskVelocity,
        const crf::utility::types::TaskAcceleration& taskAcceleration =
            crf::utility::types::TaskAcceleration()) override;
    crf::expected<bool> setTaskForceTorque(const bool& isSmoothTrajectory,
        const crf::utility::types::TaskForceTorque& taskForceTorque) override;

    crf::expected<crf::utility::types::JointVelocities> getProfileJointVelocities() override;
    crf::expected<crf::utility::types::JointAccelerations> getProfileJointAccelerations() override;

    crf::expected<crf::utility::types::TaskVelocity> getProfileTaskVelocity() override;
    crf::expected<crf::utility::types::TaskAcceleration> getProfileTaskAcceleration() override;

    crf::expected<bool> setProfileJointVelocities(
        const crf::utility::types::JointVelocities& jointVelocities) override;
    crf::expected<bool> setProfileJointAccelerations(
        const crf::utility::types::JointAccelerations& jointAccelerations) override;

    crf::expected<bool> setProfileTaskVelocity(
        const crf::utility::types::TaskVelocity& taskVelocity) override;
    crf::expected<bool> setProfileTaskAcceleration(
        const crf::utility::types::TaskAcceleration& taskAcceleration) override;

    crf::expected<bool> setGravity(const std::array<double, 3>& gravity) override;

    crf::expected<bool> softStop() override;
    crf::expected<bool> hardStop() override;

    crf::expected<bool> setBrakes(std::vector<bool> brakesStatus);
    crf::expected<std::vector<bool>> getBrakes();

    std::set<Code> robotStatus() override;
    crf::expected<bool> resetFaultState() override;

    std::shared_ptr<RobotConfiguration> getConfiguration() override;

 private:
    std::shared_ptr<devices::haption::IHaptionAPI> haptionInterface_;
    crf::actuators::robot::Virtuose6DTAOConfiguration configuration_;
    crf::utility::logger::EventLogger logger_;
    bool isInitialized_;

    /**
     * @brief Haption requires lower accuracy because the given orientation is not a quaternion
     *        with high decimal precision.
     * 
     */
    const double quaternionAccuracy_ = 1e-6;

    /**
     * @brief Set the Initial Cartesian Pose object
     * 
     * @param pose 
     * @return crf::expected<bool> 
     */
    crf::expected<bool> setInitialCartesianPose(const crf::utility::types::TaskPose& pose);
};

}  // namespace crf::actuators::robot
