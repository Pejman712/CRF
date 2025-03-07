/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>
#include <vector>
#include <string>

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

class KortexMovementAPIInterfaceMock : public IKortexMovementAPIInterface {
 public:
    MOCK_METHOD(bool, connect, (std::string ip, uint32_t port, uint32_t port_realtime), (override));
    MOCK_METHOD(void, disconnect, (), (override));
    MOCK_METHOD(void, set_username, (const char* value), (override));
    MOCK_METHOD(void, set_password, (const char* value), (override));
    MOCK_METHOD(void, set_session_inactivity_timeout,
        (::google::protobuf::uint32 value), (override));
    MOCK_METHOD(void, set_connection_inactivity_timeout,
        (::google::protobuf::uint32 value), (override));
    MOCK_METHOD(void, CreateSession, (), (override));
    MOCK_METHOD(void, CloseSession, (), (override));
    MOCK_METHOD(void, SetActivationStatus, (bool isActive), (override));
    MOCK_METHOD(Kinova::Api::BaseCyclic::Feedback, RefreshFeedback,
        (uint32_t deviceId, const Kinova::Api::RouterClientSendOptions& options), (override));
    MOCK_METHOD(::Kinova::Api::BaseCyclic::BaseFeedback, base_feedback,
        (uint32_t deviceId, const Kinova::Api::RouterClientSendOptions& options), (override));
    MOCK_METHOD(::Kinova::Api::BaseCyclic::ActuatorFeedback, actuator_feedback,
        (int index), (override));
    MOCK_METHOD(void, set_high_level_velocity, (float value, int index), (override));
    MOCK_METHOD(void, clear_velocity_high, (), (override));
    MOCK_METHOD(void, send_command_velocity, (), (override));
    MOCK_METHOD(void, set_high_level_position,
        (const std::vector<double>& jointPositions, int Dof), (override));
    MOCK_METHOD(void, send_command_position, (), (override));
};

}  // namespace crf::communication::kortexapi
