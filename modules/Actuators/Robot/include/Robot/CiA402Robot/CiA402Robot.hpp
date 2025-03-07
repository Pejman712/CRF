/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */
#pragma once

#include <memory>
#include <vector>
#include <set>
#include <string>

#include "crf/expected.hpp"

#include "CANopenDrivers/CiA402/ICiA402Driver.hpp"
#include "Robot/IRobot.hpp"
#include "Types/Types.hpp"

#include "Robot/CiA402Robot/CiA402RobotConfiguration.hpp"

using crf::devices::canopendrivers::ICiA402Driver;

namespace crf::actuators::robot {

/**
 * @brief Implementation of IRobot that creates a robot which is a combination
 * of motors that follow the CANopen CiA402 standard. This class controls them in
 * pararel to achieve a robot-like behaviour.
 *
 */
class CiA402Robot: public IRobot {
 public:
    CiA402Robot() = delete;
    CiA402Robot(std::vector<std::shared_ptr<ICiA402Driver>> motors,
        const CiA402RobotConfiguration& configuration);
    ~CiA402Robot() override;

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

    crf::expected<bool> setBrakes(std::vector<bool> brakesStatus) override;
    crf::expected<std::vector<bool>> getBrakes() override;

    std::set<Code> robotStatus() override;
    crf::expected<bool> resetFaultState() override;

    std::shared_ptr<RobotConfiguration> getConfiguration() override;

 private:
    std::vector<std::shared_ptr<ICiA402Driver>> motors_;
    CiA402RobotConfiguration robotConfiguration_;

    bool initialized_;
    uint64_t nMotors_;
    ProfileParameters profileParams_;

    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::robot
