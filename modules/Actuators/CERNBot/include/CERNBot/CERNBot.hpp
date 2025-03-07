/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include <boost/optional.hpp>

#include "CANOpenDevices/ICANOpenContext.hpp"
#include "CANOpenDevices/CANOpenMotors/ICANOpenMotor.hpp"
#include "CANOpenDevices/ObjectDictionary.hpp"
#include "CANSocket/ICANSocket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "RobotBase/IRobotBase.hpp"
#include "RobotBase/IRobotBaseKinematics.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"

namespace crf::actuators::robotbase {

class CERNBot : public IRobotBase {
 public:
    CERNBot() = delete;
    CERNBot(std::shared_ptr<communication::cansocket::ICANSocket> socket,
        const nlohmann::json& configuration);

    // Constructor used only for testing
    CERNBot(const nlohmann::json& configuration,
        const std::vector<std::shared_ptr<devices::canopendevices::ICANOpenMotor> >& motors,
        std::shared_ptr<devices::canopendevices::ICANOpenContext> ctx);

    CERNBot(const CERNBot&) = delete;
    CERNBot(CERNBot&&) = delete;
    ~CERNBot() override;

    bool initialize() override;
    bool deinitialize() override;

    boost::optional<crf::utility::types::TaskPose> getTaskPose() override;
    boost::optional<crf::utility::types::TaskVelocity> getTaskVelocity() override;
    boost::optional<std::vector<float> > getMotorsVelocities() override;
    boost::optional<std::vector<float> > getMotorsCurrent() override;
    bool setTaskVelocity(const crf::utility::types::TaskVelocity& velocity) override;
    bool setWheelsVelocity(const std::vector<float> velocity) override;
    bool stopBase() override;

    bool errorActive() override;
    bool acknowledgeError() override;

    std::shared_ptr<RobotBaseConfiguration> getConfiguration() override;

    bool setMaximumWheelsAcceleration(float acceleration) override;
    bool setMaximumWheelsVelocity(float velocity) override;
    bool setMaximumTaskVelocity(const utility::types::TaskVelocity& velocity) override;

    float getMaximumWheelsAcceleration() const override;
    float getMaximumWheelsVelocity() const override;
    utility::types::TaskVelocity getMaximumTaskVelocity() const override;

 private:
    utility::logger::EventLogger logger_;
    bool initialized_;

    const float radToRPMConverter_;
    const std::vector<float> motorsDirection_;
    const float gearBoxReduction_;

    std::shared_ptr<communication::cansocket::ICANSocket> socket_;
    std::vector<std::shared_ptr<devices::canopendevices::ICANOpenMotor> > motors_;
    std::shared_ptr<devices::canopendevices::ICANOpenContext> ctx_;
    std::shared_ptr<RobotBaseConfiguration> configuration_;
    std::unique_ptr<IRobotBaseKinematics> kinematics_;

    utility::types::TaskVelocity maximumTaskVelocity_;
    float maximumWheelsVelocity_;
    float maximumWheelsAcceleration_;
};

}  // namespace crf::actuators::robotbase
