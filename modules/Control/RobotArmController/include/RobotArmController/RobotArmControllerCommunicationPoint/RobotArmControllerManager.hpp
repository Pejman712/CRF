/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>

#include <nlohmann/json.hpp>

#include "DeviceManager/IDeviceManager.hpp"
#include "RobotArm/IRobotArm.hpp"
#include "Gripper/IGripper.hpp"
#include "RobotArmController/ControllerMode.hpp"
#include "RobotArmController/IRobotArmController.hpp"
#include "RobotArmController/RobotArmVelocityController/RobotArmVelocityController.hpp"
#include "ComponentAccessControl/SimpleAccessControl/SimpleAccessControl.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "Types/JsonConverters.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::control::robotarmcontroller {

/*
 * @brief 
 */
class RobotArmControllerManager: public crf::utility::devicemanager::IDeviceManager {  // NOLINT
 public:
    RobotArmControllerManager() = delete;
    RobotArmControllerManager(std::shared_ptr<crf::actuators::robotarm::IRobotArm> robotarm,
        std::shared_ptr<crf::actuators::gripper::IGripper> gripper = nullptr,
        const std::chrono::milliseconds& inizializationTimeout  = std::chrono::seconds(60),
        const std::chrono::milliseconds& controlAccessTimeout  = std::chrono::seconds(10));
    RobotArmControllerManager(const RobotArmControllerManager& other) = delete;
    RobotArmControllerManager(RobotArmControllerManager&& other) = delete;
    ~RobotArmControllerManager();
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool lockControl(const uint32_t &priority);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool unlockControl(const uint32_t &priority);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setControllerMode(const uint32_t &priority,
        const crf::control::robotarmcontroller::ControllerMode& mode);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    std::future<bool> setPosition(const int priority,
        std::vector<crf::utility::types::JointPositions> position);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    std::future<bool> setPosition(const int priority,
        std::vector<crf::utility::types::TaskPose> position,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setVelocity(const int priority, crf::utility::types::JointVelocities velocity);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setVelocity(const int priority, crf::utility::types::TaskVelocity velocity,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setAcceleration(const int priority,
        crf::utility::types::JointAccelerations acceleration);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setAcceleration(const int priority,
        crf::utility::types::TaskAcceleration acceleration,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool interruptTrajectory(const int priority);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setJointsMaximumVelocity(const int priority,
        crf::utility::types::JointVelocities velocity);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setJointsMaximumAcceleration(const int priority,
        crf::utility::types::JointAccelerations acceleration);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setTaskMaximumVelocity(const int priority,
        crf::utility::types::TaskVelocity velocity);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setTaskMaximumAcceleration(const int priority,
        crf::utility::types::TaskAcceleration acceleration);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setGripperVelocity(const int priority, float velocity);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setGripperPosition(const int priority, float position);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    nlohmann::json getStatus();

 private:
    std::shared_ptr<crf::actuators::robotarm::IRobotArm> robotarm_;
    std::shared_ptr<crf::actuators::gripper::IGripper> gripper_;

    std::unique_ptr<crf::control::robotarmcontroller::IRobotArmController> controller_;
    crf::control::robotarmcontroller::ControllerMode controllerMode_;
    crf::communication::componentaccesscontrol::SimpleAccessControl simpleAccessControl_;
    std::chrono::milliseconds initializationTimeout_;
    std::chrono::high_resolution_clock::time_point lastRequestTime_;
    std::mutex accessControlMutex_;
    std::mutex initializationMtx_;
    std::condition_variable initializationCV_;
    std::atomic<bool> controllerInitialized_;
    std::thread checkLatestRequestThread_;

    crf::utility::logger::EventLogger logger_;

    bool checkCommandPriority(const int &priority);
    void checkLatestRequestTime();
    void checkController();
};

}  // namespace crf::control::robotarmcontroller
