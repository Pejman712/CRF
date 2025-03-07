/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include "KortexAPI/KortexVisionAPIInterface.hpp"

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <SessionManager.h>

#include <DeviceManagerClientRpc.h>
#include <VisionConfigClientRpc.h>

namespace crf::communication::kortexapi {

KortexVisionAPIInterface::KortexVisionAPIInterface():
    create_session_info_() {}

bool KortexVisionAPIInterface::connect(std::string ip, uint32_t port) {
    auto error_callback =
        [] (const Kinova::Api::KError& e){};
    tcpClient_ = std::make_shared<Kinova::Api::TransportClientTcp>();
    routerClient_ =
        std::make_shared<Kinova::Api::RouterClient>(tcpClient_.get(), error_callback);
    if (tcpClient_->connect(ip, port)) {
        device_manager_ =
            std::make_shared<Kinova::Api::DeviceManager::DeviceManagerClient>(routerClient_.get());
        vision_config_ =
            std::make_shared<Kinova::Api::VisionConfig::VisionConfigClient>(routerClient_.get());
        return true;
    }
    return false;
}

void KortexVisionAPIInterface::disconnect() {
    tcpClient_->disconnect();
}

void KortexVisionAPIInterface::set_username(const char* value) {
    SessionManager_ = std::make_shared<Kinova::Api::SessionManager>(routerClient_.get());
    create_session_info_.set_username(value);
}

void KortexVisionAPIInterface::set_password(const char* value) {
    create_session_info_.set_password(value);
}

void KortexVisionAPIInterface::set_session_inactivity_timeout(::google::protobuf::uint32 value) {
    create_session_info_.set_session_inactivity_timeout(value);
}

void KortexVisionAPIInterface::set_connection_inactivity_timeout(::google::protobuf::uint32 value) {
    create_session_info_.set_connection_inactivity_timeout(value);
}

void KortexVisionAPIInterface::CreateSession() {
    SessionManager_->CreateSession(create_session_info_);
}

void KortexVisionAPIInterface::CloseSession() {
    SessionManager_->CloseSession();
}

void KortexVisionAPIInterface::SetActivationStatus(bool isActive) {
    routerClient_->SetActivationStatus(isActive);
}

Kinova::Api::VisionConfig::IntrinsicParameters KortexVisionAPIInterface::GetIntrinsicParameters(
    Kinova::Api::VisionConfig::SensorIdentifier* sensoridentifier,
    ::Kinova::Api::VisionConfig::Sensor sensor, uint32_t deviceId,
    const Kinova::Api::RouterClientSendOptions& options) {
    (*sensoridentifier).set_sensor(sensor);
    return vision_config_->GetIntrinsicParameters(*sensoridentifier, deviceId);
}

Kinova::Api::VisionConfig::IntrinsicParameters
    KortexVisionAPIInterface::GetIntrinsicParametersProfile(
    Kinova::Api::VisionConfig::IntrinsicProfileIdentifier* intrinsicprofileidentifier,
    ::Kinova::Api::VisionConfig::Sensor sensor,
    ::Kinova::Api::VisionConfig::Resolution resolution, uint32_t deviceId,
    const Kinova::Api::RouterClientSendOptions& options) {
    (*intrinsicprofileidentifier).set_sensor(sensor);
    (*intrinsicprofileidentifier).set_resolution(resolution);
    return vision_config_->GetIntrinsicParametersProfile(*intrinsicprofileidentifier, deviceId);
}

Kinova::Api::VisionConfig::ExtrinsicParameters KortexVisionAPIInterface::GetExtrinsicParameters(
    uint32_t deviceId, const Kinova::Api::RouterClientSendOptions& options) {
    return vision_config_->GetExtrinsicParameters(deviceId);
}

uint32_t KortexVisionAPIInterface::get_vision_device_id() {
    uint32_t vision_device_id = 0;
    auto allDevicesInfo = device_manager_->ReadAllDevices();
    for (auto dev : allDevicesInfo.device_handle()) {
        if (dev.device_type() == Kinova::Api::Common::DeviceTypes::VISION) {
            vision_device_id = dev.device_identifier();
            std::cout << "Vision module found, device Id: " << vision_device_id << std::endl;
            break;
        }
    }
    return vision_device_id;
}

Kinova::Api::VisionConfig::SensorSettings KortexVisionAPIInterface::GetSensorSettings(
    Kinova::Api::VisionConfig::SensorIdentifier* sensoridentifier,
    ::Kinova::Api::VisionConfig::Sensor sensor, uint32_t deviceId,
    const Kinova::Api::RouterClientSendOptions& options) {
    (*sensoridentifier).set_sensor(sensor);
    return vision_config_->GetSensorSettings(*sensoridentifier, deviceId);
}

void KortexVisionAPIInterface::SetResolution(
    Kinova::Api::VisionConfig::IntrinsicParameters* intrinsicparameters,
    Kinova::Api::VisionConfig::Sensor sensor, Kinova::Api::VisionConfig::Resolution resolution,
    uint32_t deviceId, const Kinova::Api::RouterClientSendOptions& options) {
    (*intrinsicparameters).set_sensor(sensor);
    (*intrinsicparameters).set_resolution(resolution);
    (*intrinsicparameters).set_principal_point_x((*intrinsicparameters).principal_point_x());
    (*intrinsicparameters).set_principal_point_y((*intrinsicparameters).principal_point_y());
    (*intrinsicparameters).set_focal_length_x((*intrinsicparameters).focal_length_x());
    (*intrinsicparameters).set_focal_length_y((*intrinsicparameters).focal_length_y());
    (*intrinsicparameters).mutable_distortion_coeffs()->set_k1(
        (*intrinsicparameters).distortion_coeffs().k1());
    (*intrinsicparameters).mutable_distortion_coeffs()->set_k2(
        (*intrinsicparameters).distortion_coeffs().k2());
    (*intrinsicparameters).mutable_distortion_coeffs()->set_p1(
        (*intrinsicparameters).distortion_coeffs().p1());
    (*intrinsicparameters).mutable_distortion_coeffs()->set_p2(
        (*intrinsicparameters).distortion_coeffs().p2());
    (*intrinsicparameters).mutable_distortion_coeffs()->set_k3(
        (*intrinsicparameters).distortion_coeffs().k3());
    vision_config_->SetIntrinsicParameters(*intrinsicparameters, deviceId);
}

void KortexVisionAPIInterface::SetFramerate(
    Kinova::Api::VisionConfig::SensorSettings* sensorsettings,
    ::Kinova::Api::VisionConfig::Sensor sensor, Kinova::Api::VisionConfig::Resolution resolution,
    ::Kinova::Api::VisionConfig::FrameRate framerate, uint32_t deviceId,
    const Kinova::Api::RouterClientSendOptions& options) {
    (*sensorsettings).set_sensor(sensor);
    (*sensorsettings).set_resolution((*sensorsettings).resolution());
    (*sensorsettings).set_frame_rate(framerate);
    (*sensorsettings).set_bit_rate((*sensorsettings).bit_rate());
    vision_config_->SetSensorSettings(*sensorsettings, deviceId);
}

}  // namespace crf::communication::kortexapi
