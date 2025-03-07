/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <set>
#include <bitset>
#include <cmath>
#include <exception>
#include <fstream>

#include <arpa/inet.h>
#include <boost/optional.hpp>
#include <nlohmann/json.hpp>

#include "crf/expected.hpp"

#include "Robot/IRobot.hpp"
#include "Types/Types.hpp"
#include "Robot/RobotConfiguration.hpp"
#include "EventLogger/EventLogger.hpp"
#include "KortexAPI/IKortexMovementAPIInterface.hpp"
#include "Robot/KinovaGen3/KinovaGen3Configuration.hpp"

namespace crf::actuators::robot {

class KinovaGen3: public IRobot {
 public:
     KinovaGen3() = delete;
     KinovaGen3(
        std::shared_ptr<crf::communication::kortexapi::IKortexMovementAPIInterface> KGInterface,
        const KinovaGen3Configuration& robotConfigFile);
    ~KinovaGen3() override;

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

    crf::expected<bool> softStop() override;
    crf::expected<bool> hardStop() override;

    crf::expected<bool> setBrakes(std::vector<bool> brakesStatus);
    crf::expected<std::vector<bool>> getBrakes();
    crf::expected<bool> setGravity(const std::array<double, 3>& gravity) override;

    std::set<Code> robotStatus() override;
    crf::expected<bool> resetFaultState() override;

    std::shared_ptr<RobotConfiguration> getConfiguration() override;

 private:
    std::shared_ptr<crf::communication::kortexapi::IKortexMovementAPIInterface> kortex_;
    KinovaGen3Configuration configuration_;

    bool isInitialized_;

    crf::utility::logger::EventLogger logger_;

    const float deg2rad_ = M_PI/180;
};


}  // namespace crf::actuators::robot
