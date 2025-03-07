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

class IKortexMovementAPIInterface {
 public:
    virtual ~IKortexMovementAPIInterface() = default;

    /**
     * @brief Connect to the robot
     * 
     * @param ip IP address
     * @param tcpPort connecting port
     * @param udpPort connecting port for real time functionalities
     * @return true if successfully connected
     * @return false if it failed
     */
    virtual bool connect(std::string ip, uint32_t tcpPort, uint32_t udpPort) = 0;

    /**
     * @brief Disconnect the Robot (TCP)
     */
    virtual void disconnect() = 0;

    /**
     * @brief set user name for session. "admin" is the one that works.
     * @param value user name
     */
    virtual void set_username(const char* value) = 0;

    /**
     * @brief set user password for session. "admin" is the one that works.
     * @param value your password
     */
    virtual void set_password(const char* value) = 0;

    /**
     * @brief set inactivity timeout for session.
     * @param value max time for session inactivity 
     */
    virtual void set_session_inactivity_timeout(google::protobuf::uint32 value) = 0;

    /**
     * @brief set inactivity timeout for connection.
     * @param value max time for connection inactivity 
     */
    virtual void set_connection_inactivity_timeout(google::protobuf::uint32 value) = 0;

    /**
     * @brief create a new session.
     */
    virtual void CreateSession() = 0;

    /**
     * @brief close a new session.
     */
    virtual void CloseSession() = 0;

    /**
     * @brief set activation status for router, when a router is established, this status is
     *        automatically true, so it's mostly used in close
     * @param isActive status you want
     */
    virtual void SetActivationStatus(bool isActive) = 0;

    /**
     * @brief get feedback from the interface module in its status
     */ 
    virtual Kinova::Api::BaseCyclic::Feedback RefreshFeedback(uint32_t deviceId = 0,
        const Kinova::Api::RouterClientSendOptions& options = {false, 0, 3000}) = 0;

    /**
     * @brief get base feedback from the interface module in its status
     */
    virtual Kinova::Api::BaseCyclic::BaseFeedback base_feedback(uint32_t deviceId = 0,
        const Kinova::Api::RouterClientSendOptions& options = {false, 0, 3000}) = 0;

    /**
     * @brief get actuator feedback from the interface module in its status
     * @param index the number of the actuator
     */     
    virtual Kinova::Api::BaseCyclic::ActuatorFeedback actuator_feedback(int index) = 0;

    /**
     * @brief set joint speeds in a high level control
     * @param value velocity in degree
     * @param index the number of actuator
    */
    virtual void set_high_level_velocity(float value, int index) = 0;

    /**
     * @brief clear all velocity
     */
    virtual void clear_velocity_high() = 0;

    /**
      * @brief send velocity commands
     */
    virtual void send_command_velocity() = 0;

    /**
     * @brief set joint positions in a high level control
     * @param value position in degree
     * @param index the number of actuator
    */
    virtual void set_high_level_position(const std::vector<double>& jointPositions, int dof) = 0;

    /**
     * @brief send position commands
    */
    virtual void send_command_position() = 0;
};

}  // namespace crf::communication::kortexapi
