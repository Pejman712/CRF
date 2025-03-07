#pragma once
/* © Copyright CERN 2019.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi EN/SMM/MRO
 *
 *  ==================================================================================================
*/

#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include <boost/optional.hpp>

#include "EventLogger/EventLogger.hpp"
#include "RobotBase/IRobotBase.hpp"
#include "RobotBase/IRobotBaseKinematics.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"
#include "RobotBase/IEtherCATRobotBase.hpp"
#include "EtherCATDevices/EtherCATDef.hpp"
#include "RobotBase/EtherCATRobotBase.hpp"



namespace crf {
namespace actuators {
namespace robotbase {


class SPSRobot : public IRobotBase {
 public:
    SPSRobot() = delete;
    SPSRobot(std::string portName,
        const nlohmann::json& configuration);

    // Constructor used only for testing
    SPSRobot(const nlohmann::json& configuration,
        const std::shared_ptr<actuators::robotbase::IEtherCATRobotBase> base);

    SPSRobot(const SPSRobot&) = delete;
    SPSRobot(SPSRobot&&) = delete;
    ~SPSRobot() override;

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

    const float radToCount_ = 11408.22632082;

    std::string portName_;
    std::vector<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor> > motors_;
    std::shared_ptr<actuators::robotbase::IEtherCATRobotBase> base_;
    std::shared_ptr<RobotBaseConfiguration> configuration_;
    std::unique_ptr<IRobotBaseKinematics> kinematics_;

    utility::types::TaskVelocity maximumTaskVelocity_;
    float maximumWheelsVelocity_;
    float maximumWheelsAcceleration_;
};

}  // namespace robotbase
}  // namespace actuators
}  // namespace crf