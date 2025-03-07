/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include "CommonInterfaces/IInitializable.hpp"
#include "Types/Types.hpp"
#include "KinovaArm/KinovaJaco.hpp"
#include "RobotArmKinematics/IRobotArmKinematics.hpp"
#include "KinovaArmConfiguration.hpp"

#include <string>
#include <memory>
#include <future>
#include <vector>

namespace crf::actuators::kinovaarm {

class KinovaAdmittanceController: public utility::commoninterfaces::IInitializable {
 public:
    KinovaAdmittanceController(std::shared_ptr<KinovaJaco> arm,
        std::shared_ptr<crf::control::robotarmkinematics::IRobotArmKinematics> kinematics,
        const std::string& configFilePath);
    ~KinovaAdmittanceController() override;
    bool initialize() override;
    bool deinitialize() override;
    bool setJointPositions(const utility::types::JointPositions&);

    utility::types::JointPositions getJointPositions();
    utility::types::TaskPose getTaskPose();

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<KinovaJaco> arm_;
    std::shared_ptr<crf::control::robotarmkinematics::IRobotArmKinematics> kinematics_;
    std::shared_ptr<crf::actuators::robotarm::RobotArmConfiguration> armConfiguration_;
    std::string configFile_;

    utility::types::JointPositions jointLimits_;
    crf::utility::types::JointForceTorques torqueMinLimits_;
    crf::utility::types::JointForceTorques torqueMaxLimits_;
    std::vector<float> velocityLowPassFilter_;

    utility::types::JointPositions currentJointPositions_;
    utility::types::JointVelocities currentJointVelocities_;
    crf::utility::types::JointForceTorques currentJointForceTorques_;

    std::chrono::microseconds rtLoopTime_;
    std::thread controlLoopThread_;
    bool runControlLoop_;
    bool positionControl_;
    utility::types::JointPositions targetJointPositions_;
    bool startControlLoop();
    bool stopControlLoop();
    void controlLoop();
};

}  // namespace crf::actuators::kinovaarm
