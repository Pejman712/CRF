/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author:       Thomas Breant CERN EN/SMM/MRO 2020
 * Contributor:  Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
*/
#pragma once

#include "RobotArm/IRobotArm.hpp"
#include "Types/Types.hpp"
#include "CANSocket/ICANSocket.hpp"
#include "CANOpenDevices/CANOpenMotors/ERB.hpp"
#include "Gripper/SchunkGripperCANOpen/SchunkGripperCANOpen.hpp"

#include <Eigen/Core>
#include <boost/optional.hpp>

#include <vector>
#include <thread>
#include <chrono>
#include <memory>
#include <string>
#include <fstream>

namespace crf::actuators::pilzarm {

class PilzArm: public crf::actuators::robotarm::IRobotArm {
 public:
    PilzArm() = delete;
    PilzArm(const PilzArm& other) = delete;
    PilzArm(PilzArm&& other) = delete;

    PilzArm(std::shared_ptr<crf::communication::cansocket::ICANSocket>, const std::string&,
        std::shared_ptr<crf::devices::canopendevices::CANOpenContext>,
        std::shared_ptr<crf::actuators::gripper::SchunkGripperCANOpen>) = delete;
    PilzArm(std::shared_ptr<crf::communication::cansocket::ICANSocket> socket,
        const nlohmann::json& robotConfigFile,
        std::shared_ptr<crf::actuators::gripper::SchunkGripperCANOpen> gripper,
        std::shared_ptr<crf::devices::canopendevices::CANOpenContext> ctx);
    ~PilzArm() override;

    bool initialize() override;
    bool deinitialize() override;

    boost::optional<utility::types::JointPositions> getJointPositions() override;
    boost::optional<utility::types::JointVelocities> getJointVelocities() override;
    boost::optional<crf::utility::types::JointForceTorques> getJointForceTorques() override;
    boost::optional<utility::types::TaskPose> getTaskPose() override;
    boost::optional<utility::types::TaskVelocity> getTaskVelocity() override;
    boost::optional<crf::utility::types::TaskForceTorque> getTaskForceTorque() override;

    bool setJointPositions(const utility::types::JointPositions& jointPositions) override;
    bool setJointPositions(const utility::types::JointPositions& jointPositions,
        const utility::types::JointVelocities& jointVelocities,
        bool relative = false);
    bool setJointPositions(const utility::types::JointPositions& jointPositions,
        const utility::types::JointVelocities& jointVelocities,
        const utility::types::JointAccelerations& jointAccelerations,
        bool relative = false);
    bool setJointVelocities(const utility::types::JointVelocities& jointVelocities) override;
    bool setJointVelocities(const utility::types::JointVelocities& jointVelocities,
        const utility::types::JointAccelerations& jointAccelerations);
    bool setJointVelocities(const utility::types::JointVelocities& jointVelocities,
        const utility::types::JointAccelerations& jointAccelerations,
        const utility::types::JointAccelerations& jointsDeceleration);
    bool setJointForceTorques(const crf::utility::types::JointForceTorques& jointForceTorques) override; // NOLINT
    bool setTaskPose(const utility::types::TaskPose& position) override;
    bool setTaskVelocity(const utility::types::TaskVelocity& velocity, bool TCP) override;

    // Arm puts on breaks
    bool stopArm() override;
    bool enableBrakes() override;
    bool disableBrakes() override;

    std::shared_ptr<actuators::robotarm::RobotArmConfiguration> getConfiguration() override;

 private:
    nlohmann::json robotConfigFile_;
    std::shared_ptr<actuators::robotarm::RobotArmConfiguration> configuration_;
    utility::logger::EventLogger logger_;
    std::shared_ptr<crf::communication::cansocket::ICANSocket> socket_;
    std::shared_ptr<crf::devices::canopendevices::CANOpenContext> ctx_;
    std::shared_ptr<crf::actuators::gripper::SchunkGripperCANOpen> gripper_;
    std::vector<std::shared_ptr<crf::devices::canopendevices::ERB>> arm_;
    bool initialized_;

    const float millidegreeToRad_ = M_PI / 180 /1000;
    const float radToMilliDeg_ = (180.0 / M_PI) *1000;
    const float defaultVelocityPercentage_ = 15;
    const int timeOutForSafeWriteInUSec_ = 500;

    bool softStop();
};

}  // namespace crf::actuators::pilzarm
