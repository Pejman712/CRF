/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <nlohmann/json.hpp>

#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "EventLogger/EventLogger.hpp"
#include "RobotArm/IRobotArm.hpp"
#include "EtherCATDevices/EtherCATDef.hpp"

namespace crf::actuators::ethercatrobotarm {

class EtherCATRobotArm : public robotarm::IRobotArm {
 public:
    EtherCATRobotArm(const nlohmann::json& armConfigFile,
        std::shared_ptr<devices::ethercatdevices::IEtherCATMotor> joint1,
        std::shared_ptr<devices::ethercatdevices::IEtherCATMotor> joint2,
        std::shared_ptr<devices::ethercatdevices::IEtherCATMotor> joint3);
    EtherCATRobotArm(const std::string& armConfigFile,
        std::shared_ptr<devices::ethercatdevices::IEtherCATMotor> joint1,
        std::shared_ptr<devices::ethercatdevices::IEtherCATMotor> joint2,
        std::shared_ptr<devices::ethercatdevices::IEtherCATMotor> joint3) = delete;
    EtherCATRobotArm() = delete;
    EtherCATRobotArm(const EtherCATRobotArm&) = delete;
    EtherCATRobotArm(EtherCATRobotArm&&) = delete;
    ~EtherCATRobotArm() override;

    bool initialize() override;
    bool deinitialize() override;

    boost::optional<crf::utility::types::JointPositions> getJointPositions() override;
    boost::optional<crf::utility::types::JointVelocities> getJointVelocities() override;
    boost::optional<crf::utility::types::JointForceTorques> getJointForceTorques() override;
    boost::optional<crf::utility::types::TaskPose> getTaskPose() override;
    boost::optional<crf::utility::types::TaskVelocity> getTaskVelocity() override;
    boost::optional<crf::utility::types::TaskForceTorque> getTaskForceTorque() override;

    bool setJointPositions(const crf::utility::types::JointPositions& jointPositions) override;
    bool setJointVelocities(const crf::utility::types::JointVelocities& jointVelocities) override;
    bool setJointForceTorques(const crf::utility::types::JointForceTorques& jointForceTorques) override; // NOLINT
    bool setTaskPose(const crf::utility::types::TaskPose& position) override;
    bool setTaskVelocity(const crf::utility::types::TaskVelocity& velocity, bool TCP) override; // NOLINT

    bool stopArm() override;

    bool enableBrakes() override;
    bool disableBrakes() override;
    std::shared_ptr<robotarm::RobotArmConfiguration> getConfiguration() override;

 private:
    utility::logger::EventLogger logger_;
    nlohmann::json armConfigFile_;
    std::vector<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor> > joints_;
    bool initialized_;
    std::shared_ptr<robotarm::RobotArmConfiguration> configuration_;
    std::vector<int> jointsDirection_;
    std::vector<double> jointPositionsOffset_;
    std::vector<double> jointPositionsExtraOffset_;
    const int numberOfInitializationAttempts = 10;

    /*
     * @brief This is the maximumm current of the drivers, by defautl the driver is set up to 0 so
     *        we need this to be able to move the motors.
     */
    const std::vector<uint16_t> jointMaxCurrents_ = {50000, 50000, 50000};
    /*
     * @brief Conversion between the internal measuring units of the driver and the SI units
     *        (rad/s). These values were taken from the driver software
     */
    const std::vector<double> jointConverFactors_ = {333771.788905168, 333771.788905168, 1000000};
    /*
     * @brief The first joint is connected to a belt that transmits the movements. This belts
     *        creates a reduction like a gearbox.
     */
    const double HDBelt_ = 1.6;
};

}  // namespace crf::actuators::ethercatrobotarm
