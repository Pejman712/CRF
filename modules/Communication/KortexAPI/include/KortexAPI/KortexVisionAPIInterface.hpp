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

#include "KortexAPI/IKortexVisionAPIInterface.hpp"

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <SessionManager.h>

#include <DeviceManagerClientRpc.h>
#include <VisionConfigClientRpc.h>

namespace crf::communication::kortexapi {

class KortexVisionAPIInterface: public IKortexVisionAPIInterface {
 public:
    KortexVisionAPIInterface();
    ~KortexVisionAPIInterface() override = default;

    bool connect(std::string ip, uint32_t port) override;
    void disconnect() override;
    void set_username(const char* value) override;
    void set_password(const char* value) override;
    void set_session_inactivity_timeout(::google::protobuf::uint32 value) override;
    void set_connection_inactivity_timeout(::google::protobuf::uint32 value) override;
    void CreateSession() override;
    void CloseSession() override;
    void SetActivationStatus(bool isActive) override;
    uint32_t get_vision_device_id() override;
    Kinova::Api::VisionConfig::IntrinsicParameters GetIntrinsicParameters(
      Kinova::Api::VisionConfig::SensorIdentifier* sensoridentifier,
      ::Kinova::Api::VisionConfig::Sensor sensor, uint32_t deviceId = 0,
      const Kinova::Api::RouterClientSendOptions& options = {false, 0, 3000}) override;
    Kinova::Api::VisionConfig::IntrinsicParameters GetIntrinsicParametersProfile(
      Kinova::Api::VisionConfig::IntrinsicProfileIdentifier* intrinsicprofileidentifier,
      ::Kinova::Api::VisionConfig::Sensor sensor,
      ::Kinova::Api::VisionConfig::Resolution resolution, uint32_t deviceId = 0,
      const Kinova::Api::RouterClientSendOptions& options = {false, 0, 3000}) override;
    Kinova::Api::VisionConfig::ExtrinsicParameters GetExtrinsicParameters(uint32_t deviceId = 0,
      const Kinova::Api::RouterClientSendOptions& options = {false, 0, 3000}) override;
    Kinova::Api::VisionConfig::SensorSettings GetSensorSettings(
      Kinova::Api::VisionConfig::SensorIdentifier* sensoridentifier,
      ::Kinova::Api::VisionConfig::Sensor sensor, uint32_t deviceId = 0,
      const Kinova::Api::RouterClientSendOptions& options = {false, 0, 3000}) override;
    void SetResolution(Kinova::Api::VisionConfig::IntrinsicParameters* intrinsicparameters,
      Kinova::Api::VisionConfig::Sensor sensor, Kinova::Api::VisionConfig::Resolution resolution,
      uint32_t deviceId = 0, const Kinova::Api::RouterClientSendOptions& options
      = {false, 0, 3000}) override;
    void SetFramerate(Kinova::Api::VisionConfig::SensorSettings* sensorsettings,
      ::Kinova::Api::VisionConfig::Sensor sensor, Kinova::Api::VisionConfig::Resolution resolution,
      ::Kinova::Api::VisionConfig::FrameRate framerate, uint32_t deviceId = 0,
      const Kinova::Api::RouterClientSendOptions& options = {false, 0, 3000}) override;

 private:
    std::shared_ptr<Kinova::Api::TransportClientTcp> tcpClient_;
    std::shared_ptr<Kinova::Api::RouterClient> routerClient_;
    std::shared_ptr<Kinova::Api::SessionManager> SessionManager_;
    Kinova::Api::Session::CreateSessionInfo create_session_info_;
    std::shared_ptr<Kinova::Api::VisionConfig::VisionConfigClient> vision_config_;
    std::shared_ptr<Kinova::Api::DeviceManager::DeviceManagerClient> device_manager_;
    uint32_t vision_device_id_;
};
}  // namespace crf::communication::kortexapi
