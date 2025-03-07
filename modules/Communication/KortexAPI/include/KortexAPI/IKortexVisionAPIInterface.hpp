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

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <SessionManager.h>

#include <DeviceManagerClientRpc.h>
#include <VisionConfigClientRpc.h>

namespace crf::communication::kortexapi {

class IKortexVisionAPIInterface {
 public:
    virtual ~IKortexVisionAPIInterface() = default;

    /**
     * @brief Connect the Robot (TCP)
     * @param IP IP address
     * @param PORT connecting port 
     * @return True if successfully connected
     */
    virtual bool connect(std::string ip, uint32_t port) = 0;

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
    virtual void set_session_inactivity_timeout(::google::protobuf::uint32 value) = 0;

    /**
     * @brief set inactivity timeout for connection.
     * @param value max time for connection inactivity 
     */
    virtual void set_connection_inactivity_timeout(::google::protobuf::uint32 value) = 0;

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
     * @brief get intrinsic parameters
     * @param sensoridentifier To tell if it's color sensor or depth sensor
     * @param deviceId id for vision module
     * @param sensor the sensor we want (color & depth)
     * @return A class contains all intrinsic parameters
    */
    virtual Kinova::Api::VisionConfig::IntrinsicParameters GetIntrinsicParameters(
        Kinova::Api::VisionConfig::SensorIdentifier* sensoridentifier,
        ::Kinova::Api::VisionConfig::Sensor sensor, uint32_t deviceId = 0,
        const Kinova::Api::RouterClientSendOptions& options = {false, 0, 3000}) = 0;

    /**
     * @brief get intrinsic parameters
     * @param sensoridentifier To tell if it's color sensor or depth sensor
     * @param deviceId id for vision module
     * @param sensor the sensor we want (color & depth)
     * @param resolution resolution we want 
     * @return A class contains all intrinsic parameters
    */
    virtual Kinova::Api::VisionConfig::IntrinsicParameters GetIntrinsicParametersProfile(
        Kinova::Api::VisionConfig::IntrinsicProfileIdentifier* intrinsicprofileidentifier,
        ::Kinova::Api::VisionConfig::Sensor sensor,
        ::Kinova::Api::VisionConfig::Resolution resolution, uint32_t deviceId = 0,
        const Kinova::Api::RouterClientSendOptions& options = {false, 0, 3000}) = 0;

    /**
     * @brief get extrinsics parameters
     * @param deviceId id for vision module
     * @return A class contains all extrinsic parameters
    */
    virtual Kinova::Api::VisionConfig::ExtrinsicParameters GetExtrinsicParameters(
        uint32_t deviceId = 0, const Kinova::Api::RouterClientSendOptions& options
        = {false, 0, 3000}) = 0;

    /**
     * @brief get device id for vision module
     * @return device id for vision module
    */
    virtual uint32_t get_vision_device_id() = 0;

    /**
     * @brief get sensor settings for vision module
     * @return sensor setting class, including 
    */
    virtual Kinova::Api::VisionConfig::SensorSettings GetSensorSettings(
        Kinova::Api::VisionConfig::SensorIdentifier* sensoridentifier,
        ::Kinova::Api::VisionConfig::Sensor sensor, uint32_t deviceId = 0,
        const Kinova::Api::RouterClientSendOptions& options = {false, 0, 3000}) = 0;

    /**
     * @brief set resolution (for color & depth)
    */
    virtual void SetResolution(Kinova::Api::VisionConfig::IntrinsicParameters* intrinsicparameters,
        Kinova::Api::VisionConfig::Sensor sensor, Kinova::Api::VisionConfig::Resolution resolution,
        uint32_t deviceId = 0, const Kinova::Api::RouterClientSendOptions& options
        = {false, 0, 3000}) = 0;

    /**
     * @brief set framerate (for color & depth)
    */
    virtual void SetFramerate(Kinova::Api::VisionConfig::SensorSettings* sensorsettings,
        ::Kinova::Api::VisionConfig::Sensor sensor,
        Kinova::Api::VisionConfig::Resolution resolution,
        ::Kinova::Api::VisionConfig::FrameRate framerate, uint32_t deviceId = 0,
        const Kinova::Api::RouterClientSendOptions& options = {false, 0, 3000}) = 0;
};
}  // namespace crf::communication::kortexapi
