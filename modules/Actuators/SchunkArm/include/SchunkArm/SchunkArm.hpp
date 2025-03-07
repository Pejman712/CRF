#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "RobotArm/IRobotArm.hpp"
#include "Types/Types.hpp"
#include "CANSocket/ICANSocket.hpp"
#include "SchunkArm/SchunkDevice.hpp"
#include "SchunkArm/SchunkGripper.hpp"

#include <Eigen/Core>
#include <boost/optional.hpp>

#include <vector>
#include <thread>
#include <chrono>
#include <memory>
#include <string>

namespace crf::actuators::schunkarm {

class SchunkArm: public robotarm::IRobotArm {
 public:
    SchunkArm() = delete;
    SchunkArm(const SchunkArm& other) = delete;
    SchunkArm(SchunkArm&& other) = delete;
    SchunkArm(std::shared_ptr<ICANSocket>,
        const std::string&,
        std::shared_ptr<SchunkGripper>) = delete;
    // Allocates the memory, no IO is performed, no threads are started
    SchunkArm(std::shared_ptr<ICANSocket> socket,
        const nlohmann::json& robotConfigFile,
        std::shared_ptr<SchunkGripper> gripper = nullptr);
    ~SchunkArm() override;

    // Starts the controller thread and initializes IO
    bool initialize() override;
    // Stops the threads, frees memory
    bool deinitialize() override;

    // value is radians, for position its [-pi;pi]
    boost::optional<utility::types::JointPositions> getJointPositions() override;
    boost::optional<utility::types::JointVelocities> getJointVelocities() override;
    // not supported
    boost::optional<crf::utility::types::JointForceTorques> getJointForceTorques() override;
    boost::optional<utility::types::TaskPose> getTaskPose() override;
    boost::optional<utility::types::TaskVelocity> getTaskVelocity() override;
    boost::optional<crf::utility::types::TaskForceTorque> getTaskForceTorque() override;

    // units are in rad, rad/s
    bool setJointPositions(const utility::types::JointPositions& jointPositions) override;
    bool setJointVelocities(const utility::types::JointVelocities& jointPositions) override;
    // not supported
    bool setJointForceTorques(const crf::utility::types::JointForceTorques& jointForceTorques) override; // NOLINT
    bool setTaskPose(const utility::types::TaskPose& position) override;
    bool setTaskVelocity(const utility::types::TaskVelocity& velocity, bool TCP) override;

    // Arm puts on breaks
    bool stopArm() override;

    bool enableBrakes() override;
    bool disableBrakes() override;

    std::shared_ptr<crf::actuators::robotarm::RobotArmConfiguration> getConfiguration() override;

 private:
    const float millidegreeToRad_;
    const int numberOfJoints_;
    const int mainLoopPeriodMilliseconds_;
    utility::logger::EventLogger logger_;
    std::shared_ptr<ICANSocket> socket_;
    nlohmann::json robotConfigFile_;
    std::shared_ptr<SchunkGripper> gripper_;
    utility::types::JointPositions jointPositions_;
    const float defaultVelocityPercentage_;
    // this is  the default velocity setJointPositions will run
    // if you want to be sure that your message arrives,
    // you may want to wait between sending several messages
    const int timeOutForSafeWriteInUSec_;
    utility::types::JointVelocities jointVelocities_;
    std::shared_ptr<crf::actuators::robotarm::RobotArmConfiguration> configuration_;
    std::vector<std::shared_ptr<SchunkDevice> > arm_;
    std::thread controlLoopThread_;
    std::atomic<bool> runControlLoop_;
    bool initialized_;

    bool startControlLoop();
    bool stopControlLoop();
    // Reads out the buffer, handles the messages
    bool updateRobotState();
    bool softStop();
    bool hardStop();
    // This is a soft-real time controller of the robot arm
    void mainControlLoop();
};

}  // namespace crf::actuators::schunkarm
