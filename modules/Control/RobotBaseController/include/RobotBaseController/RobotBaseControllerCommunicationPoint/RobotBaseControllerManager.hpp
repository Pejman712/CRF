/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>

#include <nlohmann/json.hpp>

#include "DeviceManager/IDeviceManager.hpp"
#include "RobotBase/IRobotBase.hpp"
#include "LinearStage/ILinearStage.hpp"
#include "RobotBaseController/IRobotBaseController.hpp"
#include "RobotBaseController/RobotBaseVelocityController/RobotBaseVelocityController.hpp"
#include "RobotBaseController/ControllerMode.hpp"
#include "ComponentAccessControl/SimpleAccessControl/SimpleAccessControl.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "Types/JsonConverters.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::control::robotbasecontroller {

/*
 * @brief 
 */
class RobotBaseControllerManager: public crf::utility::devicemanager::IDeviceManager {
 public:
    RobotBaseControllerManager() = delete;
    RobotBaseControllerManager(std::shared_ptr<crf::actuators::robotbase::IRobotBase> robotbase,
        std::shared_ptr<crf::actuators::linearstage::ILinearStage> stage = nullptr,
        const std::chrono::milliseconds& initializationTimeout  = std::chrono::seconds(60),
        const std::chrono::milliseconds& controlAccessTimeout  = std::chrono::seconds(10));
    RobotBaseControllerManager(const RobotBaseControllerManager& other) = delete;
    RobotBaseControllerManager(RobotBaseControllerManager&& other) = delete;
    ~RobotBaseControllerManager();

    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool lockControl(const uint32_t& priority);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool unlockControl(const uint32_t& priority);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setControllerMode(const uint32_t &priority,
        const ControllerMode& mode);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    std::future<bool> setPosition(const int& priority,
        std::vector<crf::utility::types::TaskPose> position);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setVelocity(const int& priority, const crf::utility::types::TaskVelocity& velocity);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool interruptTrajectory(const int& priority);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setMaximumVelocity(const int& priority,
        const crf::utility::types::TaskVelocity& velocity);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setMaximumAcceleration(const int& priority,
        const crf::utility::types::TaskAcceleration& acceleration);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setStageVelocity(const int& priority, const float& velocity);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    bool setStagePosition(const int& priority, const float& position);
    /*
     * @brief 
     * @param 
     * @param 
     * @return 
     */
    nlohmann::json getStatus() override;

 private:
    std::shared_ptr<crf::actuators::robotbase::IRobotBase> robotbase_;
    std::shared_ptr<crf::actuators::linearstage::ILinearStage> stage_;
    crf::communication::componentaccesscontrol::SimpleAccessControl simpleAccessControl_;
    std::shared_ptr<crf::control::robotbasecontroller::IRobotBaseController> controller_;

    crf::control::robotbasecontroller::ControllerMode controllerMode_;
    std::chrono::high_resolution_clock::time_point lastRequestTime_;
    std::chrono::milliseconds initializationTimeout_;
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

}  // namespace crf::control::robotbasecontroller
