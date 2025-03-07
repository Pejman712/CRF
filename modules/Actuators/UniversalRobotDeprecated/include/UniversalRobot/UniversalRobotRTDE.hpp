/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include "EventLogger/EventLogger.hpp"
#include "RobotArm/IRobotArm.hpp"
#include "UniversalRobot/UniversalRobotConfiguration.hpp"
#include "UniversalRobot/IUniversalRobotRTDEInterface.hpp"

namespace crf::actuators::universalrobot {

/*
 * @brief Connects to the UR robots using the RTDE protocol, due to this the local PC has to have
 *        the IP address 192.168.1.100 and it uses the port 5900
 */
class UniversalRobotRTDE: public robotarm::IRobotArm {
 public:
    UniversalRobotRTDE() = delete;
    UniversalRobotRTDE(std::shared_ptr<IUniversalRobotRTDEInterface>, const std::string&) = delete;
    UniversalRobotRTDE(std::shared_ptr<IUniversalRobotRTDEInterface> urRtdeInterface,
        const nlohmann::json& robotConfigFile);

    ~UniversalRobotRTDE() override;

    bool initialize() override;
    bool deinitialize() override;

    boost::optional<utility::types::JointPositions> getJointPositions() override;
    boost::optional<utility::types::JointVelocities> getJointVelocities() override;
    boost::optional<crf::utility::types::JointForceTorques> getJointForceTorques() override;
    boost::optional<utility::types::TaskPose> getTaskPose() override;
    boost::optional<utility::types::TaskVelocity> getTaskVelocity() override;
    boost::optional<crf::utility::types::TaskForceTorque> getTaskForceTorque() override;
    bool setJointPositions(const utility::types::JointPositions& jointPositions) override;
    bool setJointVelocities(const utility::types::JointVelocities& jointVelocities) override;
    bool setJointForceTorques(const crf::utility::types::JointForceTorques& jointForceTorques) override; // NOLINT
    bool setTaskPose(const utility::types::TaskPose& position) override;
    bool setTaskVelocity(const utility::types::TaskVelocity& velocity,
        bool toolCenterPoint = false) override;
    bool stopArm() override;
    bool enableBrakes() override;
    bool disableBrakes() override;
    std::shared_ptr<robotarm::RobotArmConfiguration> getConfiguration() override;

    /*
     * @brief Makes the robot move each joint with the desired velocities in radians/second. Joint
     *        accelerations are implementation-defined.
     * @return True if possible, False if it failed (e.g. values out of limits)
     */
    bool setJointVelocities(const utility::types::JointVelocities& jointVelocities,
        double maxAcceleration);

 private:
    std::shared_ptr<IUniversalRobotRTDEInterface> urRtdeInterface_;
    nlohmann::json robotConfigFile_;
    utility::logger::EventLogger logger_;
    std::shared_ptr<UniversalRobotConfiguration> configuration_;
    bool isInitialized_;
};

}  // namespace crf::actuators::universalrobot
