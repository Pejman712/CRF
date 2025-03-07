/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */


#include <memory>
#include <string>
#include <vector>
#include <set>
#include <bitset>
#include <cmath>
#include <exception>
#include <fstream>
#include <math.h>

#include <arpa/inet.h>
#include <boost/optional.hpp>
#include <nlohmann/json.hpp>

#include "crf/expected.hpp"

#include "Robot/IRobot.hpp"
#include "Types/Types.hpp"
#include "Robot/RobotConfiguration.hpp"
#include "EventLogger/EventLogger.hpp"
#include "KortexAPI/KortexMovementAPIInterface.hpp"
#include "KortexAPI/IKortexMovementAPIInterface.hpp"
#include "Robot/KinovaGen3/KinovaGen3Configuration.hpp"
#include "Robot/KinovaGen3/KinovaGen3.hpp"

namespace crf::actuators::robot {

KinovaGen3::KinovaGen3(
    std::shared_ptr<crf::communication::kortexapi::IKortexMovementAPIInterface> KGInterface,
    const KinovaGen3Configuration& robotConfigFile):
    kortex_(KGInterface),
    configuration_(robotConfigFile),
    isInitialized_(false),
    logger_("KinovaGen3") {
    logger_->debug("CTor");
}

KinovaGen3::~KinovaGen3() {
    logger_->debug("DTor");
    deinitialize();
}

bool KinovaGen3::initialize() {
    logger_->debug("initialize");
    if (isInitialized_) {
        logger_->warn("Already initialize");
        return true;
    }
    if (!kortex_->connect(configuration_.getIPAddress(),
        configuration_.getTCPPort(), configuration_.getUDPPort())) {
        logger_->error("Failed to connect");
        return false;
    }
    logger_->info("Connection successfull");
    try {
        kortex_->SetActivationStatus(true);
        kortex_->set_username(configuration_.getUsername().c_str());
        kortex_->set_password(configuration_.getPassword().c_str());
        kortex_->set_session_inactivity_timeout(configuration_.getSessionTimeout());
        kortex_->set_connection_inactivity_timeout(configuration_.getConnectionTimeout());
        kortex_->CreateSession();
    }
    catch(Kinova::Api::KDetailedException& ex) {
        auto error = ex.getErrorInfo().getError();
        logger_->error("Failed to initialize due to a Kortex exception: {}", ex.what());
        logger_->error("Kortex error code: {}", error.error_code());
        logger_->error("Kortex code string equivalent: {}", Kinova::Api::ErrorCodes_Name(
            Kinova::Api::ErrorCodes(error.error_code())));
        logger_->error("Kortex sub-code: {}", error.error_sub_code());
        logger_->error("Kortex sub-string: {}", error.error_sub_string());
        logger_->error("Kortex sub-code string equivalent: {}", Kinova::Api::SubErrorCodes_Name(
            Kinova::Api::SubErrorCodes(error.error_sub_code())));
        return false;
    }
    isInitialized_ = true;
    return true;
}

bool KinovaGen3::deinitialize() {
    logger_->debug("deinitialize");
    if (!isInitialized_) {
        logger_->warn("Not initialized");
        return true;
    }
    kortex_->CloseSession();
    kortex_->SetActivationStatus(false);
    kortex_->disconnect();
    isInitialized_ = false;
    return true;
}

crf::expected<crf::utility::types::JointPositions> KinovaGen3::getJointPositions() {
    logger_->debug("getJointPositions");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    std::vector<double> output;
    /**
     * When reaching negative positions, Kinova Robot will automatically add 2*PI
     * to make is positive. When value is larger than PI, its oringinal positions
     * should be negative, so we minus 2*PI to make it negative as it should be.
     */ 
    for (int i = 0; i < configuration_.getJointSpaceDoF(); i++) {
        float pos = kortex_->actuator_feedback(i).position();
        if (pos > 180) {
            pos = pos - 360;
        }
        output.push_back(pos*deg2rad_);
    }
    return utility::types::JointPositions(output);
}

crf::expected<crf::utility::types::JointVelocities> KinovaGen3::getJointVelocities() {
    logger_->debug("getJointVelocities {}", configuration_.getJointSpaceDoF());
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    std::vector<double> output;
    for (int i = 0; i < configuration_.getJointSpaceDoF(); i++) {
        output.push_back(kortex_->actuator_feedback(i).velocity()*deg2rad_);
    }
    return utility::types::JointVelocities(output);
}

crf::expected<crf::utility::types::JointAccelerations> KinovaGen3::getJointAccelerations() {
    logger_->debug("getJointAccelerations");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::JointForceTorques> KinovaGen3::getJointForceTorques() {
    logger_->debug("getJointForceTorques");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    std::vector<double> output;
    for (int i = 0; i < configuration_.getJointSpaceDoF(); i++) {
        output.push_back(kortex_->actuator_feedback(i).torque());
    }
    return crf::utility::types::JointForceTorques(output);
}

crf::expected<crf::utility::types::TaskPose> KinovaGen3::getTaskPose() {
    logger_->debug("getTaskPiosition");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    auto baseFeedback = kortex_->RefreshFeedback().base();
    return crf::utility::types::TaskPose(
            {baseFeedback.tool_pose_x(),
             baseFeedback.tool_pose_y(),
             baseFeedback.tool_pose_z()},
        crf::math::rotation::CardanXYZ(
            {baseFeedback.tool_pose_theta_x()*deg2rad_,
             baseFeedback.tool_pose_theta_y()*deg2rad_,
             baseFeedback.tool_pose_theta_z()*deg2rad_}));
}

crf::expected<crf::utility::types::TaskVelocity> KinovaGen3::getTaskVelocity() {
    logger_->debug("getTaskVelocity");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    auto baseFeedback = kortex_->RefreshFeedback().base();
    return utility::types::TaskVelocity({
        baseFeedback.tool_twist_linear_x(),
        baseFeedback.tool_twist_linear_y(),
        baseFeedback.tool_twist_linear_z(),
        baseFeedback.tool_twist_angular_x()*deg2rad_,
        baseFeedback.tool_twist_angular_y()*deg2rad_,
        baseFeedback.tool_twist_angular_z()*deg2rad_});
}

crf::expected<crf::utility::types::TaskAcceleration> KinovaGen3::getTaskAcceleration() {
    logger_->debug("getTaskAcceleration");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::TaskForceTorque> KinovaGen3::getTaskForceTorque() {
    logger_->debug("getTaskForceTorque");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    auto baseFeedback = kortex_->RefreshFeedback().base();
    return crf::utility::types::TaskForceTorque({
        baseFeedback.tool_external_wrench_force_x(),
        baseFeedback.tool_external_wrench_force_y(),
        baseFeedback.tool_external_wrench_force_z(),
        baseFeedback.tool_external_wrench_torque_x(),
        baseFeedback.tool_external_wrench_torque_y(),
        baseFeedback.tool_external_wrench_torque_z()});
}

crf::expected<bool> KinovaGen3::setJointPositions(const bool& isSmoothTrajectory,
    const crf::utility::types::JointPositions& jointPositions,
    const crf::utility::types::JointVelocities& jointVelocities,
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->debug("setJointPositions");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    if (jointPositions.size() != configuration_.getJointSpaceDoF()) {
        logger_->error("jointPositions size is not matching with nDoF({}).",
            configuration_.getJointSpaceDoF());
        return crf::Code::BadRequest;
    }
    std::vector<double> convertedPosition;
    for (int i = 0; i < configuration_.getJointSpaceDoF(); i++) {
        if (jointPositions[i] >= configuration_.getJointLimits().maxPosition[i] ||
            jointPositions[i] <= configuration_.getJointLimits().minPosition[i]) {
            logger_->error("Velocity of {} actuator out of range ({} - {})", i+1,
            configuration_.getJointLimits().minPosition[i],
            configuration_.getJointLimits().maxPosition[i]);
            return crf::Code::BadRequest;
        }
        convertedPosition.push_back(jointPositions[i]/deg2rad_);
    }
    try {
        kortex_->set_high_level_position(convertedPosition, configuration_.getJointSpaceDoF());
        kortex_->send_command_position();
    }
    catch(Kinova::Api::KDetailedException& ex) {
        auto error = ex.getErrorInfo().getError();
        logger_->error("Failed to set joints position due to a Kortex exception: {}", ex.what());
        logger_->error("Kortex error code: {}", error.error_code());
        logger_->error("Kortex code string equivalent: {}", Kinova::Api::ErrorCodes_Name(
            Kinova::Api::ErrorCodes(error.error_code())));
        logger_->error("Kortex sub-code: {}", error.error_sub_code());
        logger_->error("Kortex sub-string: {}", error.error_sub_string());
        logger_->error("Kortex sub-code string equivalent: {}", Kinova::Api::SubErrorCodes_Name(
            Kinova::Api::SubErrorCodes(error.error_sub_code())));
        return false;
    }
    return true;
}

crf::expected<bool> KinovaGen3::setJointVelocities(const bool& isSmoothTrajectory,
    const crf::utility::types::JointVelocities& jointVelocities,
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->debug("setJointVelocities");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    if (jointVelocities.size() != configuration_.getJointSpaceDoF()) {
        logger_->error("jointVelocities size is not matching with nDoF({}).",
            configuration_.getJointSpaceDoF());
        return crf::Code::BadRequest;
    }
    for (int i = 0; i < configuration_.getJointSpaceDoF(); i++) {
        if (abs(jointVelocities[i]) >= configuration_.getJointLimits().maxVelocity[i]) {
            logger_->error("Velocity of {} actuator out of range ({} vs {})", i+1,
                jointVelocities, configuration_.getJointLimits().maxVelocity);
            return crf::Code::BadRequest;
        }
    }
    /**
     * For now, we are using high-level velocity control because set_velocity doesn't
     * work in low-level control. The actual implementation of velocity control in the
     * actuators causes the velocity to be directly feeded to the velocity control loop,
     * which can cause a certain delay in the command and the drifting due to the gravity
     * effect. In short, if we use set_velocity(), some joints will move because of gravity
     * even if we don't send the velocity to them.
     */
    try {
        kortex_->clear_velocity_high();
        for (int i = 0; i < configuration_.getJointSpaceDoF(); i++) {
            kortex_->set_high_level_velocity(jointVelocities[i]/deg2rad_, i);
        }
        kortex_->send_command_velocity();
        return true;
    }
    catch(Kinova::Api::KDetailedException& ex) {
        auto error = ex.getErrorInfo().getError();
        logger_->error("Failed to set joints position due to a Kortex exception: {}", ex.what());
        logger_->error("Kortex error code: {}", error.error_code());
        logger_->error("Kortex code string equivalent: {}", Kinova::Api::ErrorCodes_Name(
            Kinova::Api::ErrorCodes(error.error_code())));
        logger_->error("Kortex sub-code: {}", error.error_sub_code());
        logger_->error("Kortex sub-string: {}", error.error_sub_string());
        logger_->error("Kortex sub-code string equivalent: {}", Kinova::Api::SubErrorCodes_Name(
            Kinova::Api::SubErrorCodes(error.error_sub_code())));
        return false;
    }
}

crf::expected<bool> KinovaGen3::setJointForceTorques(const bool& isSmoothTrajectory,
    const crf::utility::types::JointForceTorques& jointForceTorques) {
    logger_->debug("setJointForceTorques");
    return crf::Code::NotImplemented;
}

crf::expected<bool> KinovaGen3::setTaskPose(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskPose& taskPose,
    const crf::utility::types::TaskVelocity& taskVelocity,
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->debug("setTaskPose");
    logger_->warn("No Cartesian move with low servoing mode");
    return crf::Code::NotImplemented;
}

crf::expected<bool> KinovaGen3::setTaskVelocity(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskVelocity& taskVelocity,
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->debug("setTaskVelocity");
    logger_->warn("No Cartesian move with low servoing mode");
    return crf::Code::NotImplemented;
}

crf::expected<bool> KinovaGen3::setTaskForceTorque(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskForceTorque& taskForceTorque) {
    logger_->debug("setTaskForceTorque");
    logger_->warn("No Cartesian move with low servoing mode");
    return crf::Code::NotImplemented;
}

crf::expected<crf::utility::types::JointVelocities> KinovaGen3::getProfileJointVelocities() {
    logger_->debug("getProfileJointVelocities");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::JointAccelerations> KinovaGen3::getProfileJointAccelerations() {
    logger_->debug("getProfileJointAccelerations");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::TaskVelocity> KinovaGen3::getProfileTaskVelocity() {
    logger_->debug("getProfileTaskVelocity");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::TaskAcceleration> KinovaGen3::getProfileTaskAcceleration() {
    logger_->debug("getProfileTaskAcceleration");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> KinovaGen3::setProfileJointVelocities(
    const crf::utility::types::JointVelocities& jointVelocities) {
    logger_->debug("setProfileJointVelocities");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> KinovaGen3::setProfileJointAccelerations(
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->debug("setProfileJointAccelerations");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> KinovaGen3::setProfileTaskVelocity(
    const crf::utility::types::TaskVelocity& taskVelocity) {
    logger_->debug("setProfileTaskVelocity");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> KinovaGen3::setProfileTaskAcceleration(
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->debug("setProfileTaskAcceleration");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> KinovaGen3::softStop() {
    logger_->debug("softStop");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> KinovaGen3::hardStop() {
    logger_->debug("hardStop");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> KinovaGen3::setBrakes(std::vector<bool> brakesStatus) {
    logger_->debug("setBrakes");
    return crf::Code::MethodNotAllowed;
}

crf::expected<std::vector<bool>> KinovaGen3::getBrakes() {
    logger_->debug("getBrakes");
    return crf::Code::MethodNotAllowed;
}

std::set<Code> KinovaGen3::robotStatus() {
    std::set<Code> statusList;
    logger_->debug("robotStatus");
    return statusList;
}

crf::expected<bool> KinovaGen3::resetFaultState() {
    logger_->debug("resetFaultState");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> KinovaGen3::setGravity(const std::array<double, 3>& gravity) {
    logger_->debug("setGravity");
    return crf::Code::MethodNotAllowed;
}

std::shared_ptr<RobotConfiguration> KinovaGen3::getConfiguration() {
    logger_->debug("getConfiguration");
    return std::make_shared<RobotConfiguration>(configuration_);
}

}  // namespace crf::actuators::robot
