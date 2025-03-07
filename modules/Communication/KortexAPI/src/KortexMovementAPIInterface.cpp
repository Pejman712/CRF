/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <KDetailedException.h>

#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <TransportClientUdp.h>
#include <SessionManager.h>
#include <BaseCyclicClientRpc.h>
#include <BaseClientRpc.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>

#include "KortexAPI/KortexMovementAPIInterface.hpp"

namespace crf::communication::kortexapi {

KortexMovementAPIInterface::KortexMovementAPIInterface():
    createSessionInfo_(),
    jointSpeedsHigh_(),
    action_() {
}

bool KortexMovementAPIInterface::connect(std::string ip, uint32_t tcpPort, uint32_t udpPort) {
    tcpClient_ = std::make_shared<Kinova::Api::TransportClientTcp>();
    udpClient_ = std::make_shared<Kinova::Api::TransportClientUdp>();
    tcpRouter_ = std::make_shared<Kinova::Api::RouterClient>(tcpClient_.get(), errorHandler);
    udpRouter_ = std::make_shared<Kinova::Api::RouterClient>(udpClient_.get(), errorHandler);
    if (tcpClient_->connect(ip, tcpPort) && udpClient_->connect(ip, udpPort)) {
        tcpBase_ = std::make_shared<Kinova::Api::Base::BaseClient>(tcpRouter_.get());
        udpBase_ = std::make_shared<Kinova::Api::BaseCyclic::BaseCyclicClient>(udpRouter_.get());
        return true;
    }
    return false;
}

void KortexMovementAPIInterface::disconnect() {
    tcpClient_->disconnect();
    udpClient_->disconnect();
}

void KortexMovementAPIInterface::set_username(const char* value) {
    tcpSessionManager_ = std::make_shared<Kinova::Api::SessionManager>(tcpRouter_.get());
    udpSessionManager_ = std::make_shared<Kinova::Api::SessionManager>(udpRouter_.get());
    createSessionInfo_.set_username(value);
}

void KortexMovementAPIInterface::set_password(const char* value) {
    createSessionInfo_.set_password(value);
}

void KortexMovementAPIInterface::set_session_inactivity_timeout(google::protobuf::uint32 value) {
    createSessionInfo_.set_session_inactivity_timeout(value);
}

void KortexMovementAPIInterface::set_connection_inactivity_timeout(google::protobuf::uint32 value) {
    createSessionInfo_.set_connection_inactivity_timeout(value);
}

void KortexMovementAPIInterface::CreateSession() {
    tcpSessionManager_->CreateSession(createSessionInfo_);
    udpSessionManager_->CreateSession(createSessionInfo_);
}

void KortexMovementAPIInterface::CloseSession() {
    tcpSessionManager_->CloseSession();
    udpSessionManager_->CloseSession();
}

void KortexMovementAPIInterface::SetActivationStatus(bool isActive) {
    tcpRouter_->SetActivationStatus(isActive);
    udpRouter_->SetActivationStatus(isActive);
}

Kinova::Api::BaseCyclic::Feedback KortexMovementAPIInterface::RefreshFeedback(uint32_t deviceId,
    const Kinova::Api::RouterClientSendOptions& options) {
    return udpBase_->RefreshFeedback();
}

Kinova::Api::BaseCyclic::BaseFeedback KortexMovementAPIInterface::base_feedback(uint32_t deviceId,
    const Kinova::Api::RouterClientSendOptions& options) {
    return udpBase_->RefreshFeedback().base();
}

Kinova::Api::BaseCyclic::ActuatorFeedback KortexMovementAPIInterface::actuator_feedback(int index) {
    return udpBase_->RefreshFeedback().actuators(index);
}

void KortexMovementAPIInterface::set_high_level_velocity(float value, int index) {
    Kinova::Api::Base::JointSpeed* jointSpeed = jointSpeedsHigh_.add_joint_speeds();
    jointSpeed->set_joint_identifier(index);
    jointSpeed->set_value(value);
    jointSpeed->set_duration(1);
}

void KortexMovementAPIInterface::clear_velocity_high() {
    jointSpeedsHigh_.clear_joint_speeds();
}

void KortexMovementAPIInterface::send_command_velocity() {
    tcpBase_->SendJointSpeedsCommand(jointSpeedsHigh_);
}

void KortexMovementAPIInterface::set_high_level_position(
    const std::vector<double>& jointPositions, int dof) {
    action_ = Kinova::Api::Base::Action();
    Kinova::Api::Base::JointAngles* jointAngles_ =
        action_.mutable_reach_joint_angles()->mutable_joint_angles();
    for (int i = 0; i < dof; i++) {
        Kinova::Api::Base::JointAngle* jointAngle = jointAngles_->add_joint_angles();
        jointAngle->set_joint_identifier(i);
        jointAngle->set_value(jointPositions[i]);
    }
}

void KortexMovementAPIInterface::send_command_position() {
    std::promise<Kinova::Api::Base::ActionEvent> finishPromise;
    std::future<Kinova::Api::Base::ActionEvent> finishFuture = finishPromise.get_future();
    Kinova::Api::Common::NotificationHandle handle = tcpBase_->OnNotificationActionTopic(
        createEventListenerByPromise(&finishPromise), Kinova::Api::Common::NotificationOptions());
    tcpBase_->ExecuteAction(action_);
    finishFuture.wait_for(timeDuration_);
    tcpBase_->Unsubscribe(handle);
    finishFuture.get();
}

void KortexMovementAPIInterface::errorHandler(const Kinova::Api::KError& error) {
    // TODO(@adiazros): Store all errors in a variable and make them available at ahigher level to
    // handler them.
    return;
}

std::function<void(Kinova::Api::Base::ActionNotification)> KortexMovementAPIInterface::createEventListenerByPromise(  // NOLINT
    std::promise<Kinova::Api::Base::ActionEvent>* finishPromise) {
    return [finishPromise] (Kinova::Api::Base::ActionNotification notification) {
        Kinova::Api::Base::ActionEvent actionEvent = notification.action_event();
        if (actionEvent == Kinova::Api::Base::ActionEvent::ACTION_END ||
            actionEvent == Kinova::Api::Base::ActionEvent::ACTION_ABORT) {
            finishPromise->set_value(actionEvent);
        }
    };
}

}  // namespace crf::communication::kortexapi
