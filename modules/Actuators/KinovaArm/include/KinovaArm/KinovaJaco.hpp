/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <boost/optional.hpp>

#include "EventLogger/EventLogger.hpp"
#include "RobotArm/IRobotArm.hpp"
#include "KinovaArm/KinovaArmConfiguration.hpp"
#include "KinovaArm/IKinovaApiInterface.hpp"

namespace crf::actuators::kinovaarm {

/*
 * @brief Implementation of the Kinova Jaco 2 using the manufacturer API. Currently the code only
 *        works with the robots of 6 DoF. For some functionalities (setJointPositions and
 *        setTaskPose) the controller needs to move the robot to the home position first.
 *        This is why is added as a virtual method. Position readings sometimes have n*2PI offset
 *        and it's not possible to control the arm in this mode at all if ethernetMoveHome was not
 *        invoked before. The joints torques are gravity free.
 */
class KinovaJaco: public robotarm::IRobotArm {
 public:
    KinovaJaco() = delete;
    KinovaJaco(std::shared_ptr<IKinovaApiInterface>, const std::string&) = delete;
    explicit KinovaJaco(std::shared_ptr<IKinovaApiInterface> apiInterface,
        const nlohmann::json& robotConfigFile);
    ~KinovaJaco() override;

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
    bool setTaskVelocity(const utility::types::TaskVelocity& velocity, bool toolCenterPoint = false) override; // NOLINT
    bool stopArm() override;
    bool enableBrakes() override;
    bool disableBrakes() override;
    std::shared_ptr<robotarm::RobotArmConfiguration> getConfiguration() override;

    virtual bool moveHomePosition();
    virtual bool zeroJointForceTorques();

 private:
    std::shared_ptr<IKinovaApiInterface> apiInterface_;
    nlohmann::json robotConfigFile_;
    utility::logger::EventLogger logger_;
    std::shared_ptr<KinovaArmConfiguration> configuration_;
    bool isInitialized_;
    std::vector<int> jointsDirection_;
    std::vector<double> jointPositionsOffset_;
    bool movedHome_;

    /*
     * @brief The velocity given by the Kinova API is wrong and seems to be almost half of the
     *        real one
     */
    const double velocityAPICorrectionFactor_ = 1.92;

    double deg2rad(float angle);
    float rad2deg(double angle);
};

}  // namespace crf::actuators::kinovaarm
