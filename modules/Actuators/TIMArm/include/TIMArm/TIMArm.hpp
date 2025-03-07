/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 * Contributor: Francesco Riccardi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <boost/optional.hpp>

#include "RobotArm/IRobotArm.hpp"
#include "EtherCATDevices/TIMRobotArmWagonMotors/TIMRobotArmWagonMotors.hpp"
#include "KinovaArm/IKinovaApiInterface.hpp"
#include "Types/Types.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "EventLogger/EventLogger.hpp"
#include "TIMArm/TIMArmConfiguration.hpp"
#include "EtherCATRobotArm/EtherCATRobotArm.hpp"
#include "KinovaArm/KinovaJaco.hpp"

namespace crf::actuators::timarm {

class TIMArm : public robotarm::IRobotArm {
 public:
    TIMArm() = delete;
    TIMArm(const nlohmann::json& armConfigFile,
        std::shared_ptr<crf::devices::ethercatdevices::TIMRobotArmWagonMotors> timMotors,
        std::shared_ptr<crf::actuators::kinovaarm::IKinovaApiInterface> kinovaAPI);
    TIMArm(const std::string&,
        std::shared_ptr<crf::devices::ethercatdevices::TIMRobotArmWagonMotors> timMotors,
        std::shared_ptr<crf::actuators::kinovaarm::IKinovaApiInterface>) = delete;
    TIMArm(const TIMArm&) = delete;
    TIMArm(TIMArm&&) = delete;
    ~TIMArm() override;

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
    bool setTaskVelocity(const crf::utility::types::TaskVelocity& velocity,
        bool TCP) override;
    bool stopArm() override;
    bool enableBrakes() override;
    bool disableBrakes() override;
    std::shared_ptr<crf::actuators::robotarm::RobotArmConfiguration> getConfiguration() override;

 private:
    nlohmann::json armConfigFile_;
    std::shared_ptr<crf::devices::ethercatdevices::TIMRobotArmWagonMotors> timMotors_;
    std::shared_ptr<crf::actuators::kinovaarm::IKinovaApiInterface> kinovaAPI_;

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<crf::actuators::timarm::TIMArmConfiguration> armConfiguration_;
    std::shared_ptr<crf::actuators::ethercatrobotarm::EtherCATRobotArm> etherCATArm_;
    std::shared_ptr<crf::actuators::kinovaarm::KinovaJaco> kinovaArm_;
    bool isInitialized_;

    /*
     *  Puts in  single type the positions of all the ehterCAT motors and the Kinova joints
     */
    crf::utility::types::JointPositions combinePositions(
        const crf::utility::types::JointPositions& jointPositionsHarmonicArm,
        const crf::utility::types::JointPositions& jointPositionsKinovaArm);
    /*
     *  Puts in  single type the velocity of all the ehterCAT motors and the Kinova joints
     */
    crf::utility::types::JointVelocities combineVelocities(
        const crf::utility::types::JointVelocities& jointVelocitiesHarmonicArm,
        const crf::utility::types::JointVelocities& jointVelocitiesKinovaArm);
    /*
     *  Puts in  single type the torque of all the ehterCAT motors and the Kinova joints
     */
    crf::utility::types::JointForceTorques combineTorques(
        const crf::utility::types::JointForceTorques& jointForceTorquesHarmonicArm,
        const crf::utility::types::JointForceTorques& jointForceTorquesKinovaArm);
    /*
     *  Returns the velocity of the last joints, defined by the number of joints of the Kinova arm
     */
    crf::utility::types::JointVelocities extractKinovaVelocity(
        const crf::utility::types::JointVelocities& jointVelocitiesTIMKinovaArm);
    /*
      *  Returns the velocity of the firsts joints, defined by the number of joints of the Harmonic arm
     */
    crf::utility::types::JointVelocities extractHarmonicVelocity(
        const crf::utility::types::JointVelocities& jointVelocitiesTIMKinovaArm);
    /*
     *  Returns the position of the last joints, defined by the number of joints of the Kinova arm
     */
    crf::utility::types::JointForceTorques extractKinovaTorque(
        const crf::utility::types::JointForceTorques& jointForceTorquesTIMKinovaArm);
    /*
     *  Returns the torque of the firsts joints, defined by the number of joints of the Harmonic arm
     */
    crf::utility::types::JointForceTorques extractHarmonicTorque(
        const crf::utility::types::JointForceTorques& jointForceTorquesTIMKinovaArm);
};

}  // namespace crf::actuators::timarm
