/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */
#pragma once

#include <string>
#include <vector>
#include <memory>

#include "KortexAPI/IKortexMovementAPIInterface.hpp"
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

namespace crf::communication::kortexapi {

class KortexMovementAPIInterface: public IKortexMovementAPIInterface {
 public:
    KortexMovementAPIInterface();
    ~KortexMovementAPIInterface() override = default;

    bool connect(std::string ip, uint32_t tcpPort, uint32_t udpPort) override;
    void disconnect() override;
    void set_username(const char* value) override;
    void set_password(const char* value) override;
    void set_session_inactivity_timeout(google::protobuf::uint32 value) override;
    void set_connection_inactivity_timeout(google::protobuf::uint32 value) override;
    void CreateSession() override;
    void CloseSession() override;
    void SetActivationStatus(bool isActive) override;

    Kinova::Api::BaseCyclic::Feedback RefreshFeedback(uint32_t deviceId = 0,
        const Kinova::Api::RouterClientSendOptions& options = {false, 0, 3000}) override;
    Kinova::Api::BaseCyclic::BaseFeedback base_feedback(uint32_t deviceId = 0,
        const Kinova::Api::RouterClientSendOptions& options = {false, 0, 3000}) override;
    Kinova::Api::BaseCyclic::ActuatorFeedback actuator_feedback(int index) override;

    void set_high_level_velocity(float value, int index) override;
    void clear_velocity_high() override;
    void send_command_velocity() override;
    void set_high_level_position(const std::vector<double>& jointPositions, int dof) override;
    void send_command_position() override;

 private:
    std::shared_ptr<Kinova::Api::TransportClientTcp> tcpClient_;
    std::shared_ptr<Kinova::Api::TransportClientUdp> udpClient_;
    std::shared_ptr<Kinova::Api::RouterClient> tcpRouter_;
    std::shared_ptr<Kinova::Api::RouterClient> udpRouter_;
    std::shared_ptr<Kinova::Api::SessionManager> tcpSessionManager_;
    std::shared_ptr<Kinova::Api::SessionManager> udpSessionManager_;
    std::shared_ptr<Kinova::Api::Base::BaseClient> tcpBase_;
    std::shared_ptr<Kinova::Api::BaseCyclic::BaseCyclicClient> udpBase_;
    Kinova::Api::Session::CreateSessionInfo createSessionInfo_;
    Kinova::Api::Base::JointSpeeds jointSpeedsHigh_;
    Kinova::Api::Base::Action action_;

    const std::chrono::seconds timeDuration_ = std::chrono::seconds{20};

    /**
     * @brief 
     * 
     * @param error 
     */
    static void errorHandler(const Kinova::Api::KError& error);
    /**
     * @brief Create a Event Listener By Promise object
     * 
     * @param finishPromise 
     * @return std::function<void(Kinova::Api::Base::ActionNotification)> 
     */
    std::function<void(Kinova::Api::Base::ActionNotification)> createEventListenerByPromise(
        std::promise<Kinova::Api::Base::ActionEvent>* finishPromise);
};
}  // namespace crf::communication::kortexapi
