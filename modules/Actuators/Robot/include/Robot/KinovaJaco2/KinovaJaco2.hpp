/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <set>

#include "Robot/IRobot.hpp"
#include "Robot/KinovaJaco2/KinovaJaco2Configuration.hpp"
#include "KinovaJacoAPI/IKinovaJacoAPIInterface.hpp"
#include "KinovaJacoAPI/KinovaJacoAPIInterface.hpp"

#include "crf/expected.hpp"
#include "EventLogger/EventLogger.hpp"

using crf::communication::kinovajacoapi::IKinovaJacoAPIInterface;

namespace crf::actuators::robot {

/**
 * @brief Implementation of the Kinova Jaco 2 using the manufacturer API. Currently the code only
 *        works with the robots of 6 DoF. For some functionalities (setJointPositions and
 *        setTaskPose) the controller needs to move the robot to the home position first.
 *        This is why is added as a virtual method. Position readings sometimes have n*2PI offset
 *        and it's not possible to control the arm in this mode at all if ethernetMoveHome was not
 *        invoked before. The joints torques are gravity free.
 */
class KinovaJaco2: public crf::actuators::robot::IRobot {
 public:
    KinovaJaco2() = delete;
    KinovaJaco2(std::shared_ptr<IKinovaJacoAPIInterface> apiInterface,
        const KinovaJaco2Configuration& robotConfigFile);
    ~KinovaJaco2() override;

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
        const crf::utility::types::JointVelocities& jointVelocities = crf::utility::types::JointVelocities(),  // NOLINT
        const crf::utility::types::JointAccelerations& jointAccelerations = crf::utility::types::JointAccelerations()) override;  // NOLINT
    crf::expected<bool> setJointVelocities(const bool& isSmoothTrajectory,
        const crf::utility::types::JointVelocities& jointVelocities,
        const crf::utility::types::JointAccelerations& jointAccelerations = crf::utility::types::JointAccelerations()) override;  // NOLINT
    crf::expected<bool> setJointForceTorques(const bool& isSmoothTrajectory,
        const crf::utility::types::JointForceTorques& jointForceTorques) override;
    crf::expected<bool> setTaskPose(const bool& isSmoothTrajectory,
        const crf::utility::types::TaskPose& taskPose,
        const crf::utility::types::TaskVelocity& taskVelocity = crf::utility::types::TaskVelocity(),  // NOLINT
        const crf::utility::types::TaskAcceleration& taskAcceleration = crf::utility::types::TaskAcceleration()) override;  // NOLINT
    crf::expected<bool> setTaskVelocity(const bool& isSmoothTrajectory,
        const crf::utility::types::TaskVelocity& taskVelocity,
        const crf::utility::types::TaskAcceleration& taskAcceleration = crf::utility::types::TaskAcceleration()) override;  // NOLINT
    crf::expected<bool> setTaskForceTorque(const bool& isSmoothTrajectory,
        const crf::utility::types::TaskForceTorque& taskForceTorque) override;

    crf::expected<crf::utility::types::JointVelocities> getProfileJointVelocities() override;
    crf::expected<crf::utility::types::JointAccelerations> getProfileJointAccelerations() override;  // NOLINT
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
    std::shared_ptr<crf::communication::kinovajacoapi::IKinovaJacoAPIInterface> apiInterface_;
    KinovaJaco2Configuration configuration_;

    bool initialized_;

    std::atomic<crf::Code> errorCode_;

    utility::logger::EventLogger logger_;

    const int localCommandPort_ = 25015;
    const int localBroadcastPort_ = 25025;
    const int RXTimeoutMs_ = 1000;

    double deg2rad(float angle);
    float rad2deg(double angle);

    crf::Code parseErrorCode(const int& ec);
};

}  // namespace crf::actuators::robot
