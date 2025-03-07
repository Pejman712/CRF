/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 * ====================================================================
*/

#include "ViconAPI/ViconApi.hpp"

namespace crf::communication::viconapi {

ViconApi::ViconApi() {
    client_ = std::make_shared<viconSDK::Client>();
}

viconSDK::Output_IsConnected ViconApi::isConnected() const {
    return client_->IsConnected();
}

viconSDK::Output_Disconnect ViconApi::disconnect() {
    return client_->Disconnect();
}

viconSDK::Output_SetConnectionTimeout ViconApi::setConnectionTimeout(unsigned int Timeout) {
    return client_->SetConnectionTimeout(Timeout);
}

viconSDK::Output_Connect ViconApi::connect(const viconSDK::String& HostName) {
    return client_->Connect(HostName);
}

viconSDK::Output_GetFrame ViconApi::getFrame() {
    return client_->GetFrame();
}

viconSDK::Output_EnableSegmentData ViconApi::enableSegmentData() {
    return client_->EnableSegmentData();
}

viconSDK::Output_EnableMarkerData ViconApi::enableMarkerData() {
    return client_->EnableMarkerData();
}

viconSDK::Output_EnableUnlabeledMarkerData ViconApi::enableUnlabeledMarkerData() {
    return client_->EnableUnlabeledMarkerData();
}

viconSDK::Output_EnableLightweightSegmentData ViconApi::enableLightweightSegmentData() {
    return client_->EnableLightweightSegmentData();
}

viconSDK::Output_SetStreamMode ViconApi::setStreamMode(const viconSDK::StreamMode::Enum Mode) {
    return client_->SetStreamMode(Mode);
}

viconSDK::Output_SetAxisMapping ViconApi::setAxisMapping(const viconSDK::Direction::Enum XAxis,
    const viconSDK::Direction::Enum YAxis, const viconSDK::Direction::Enum ZAxis) {
    return client_->SetAxisMapping(XAxis, YAxis, ZAxis);
}

void ViconApi::setBufferSize(unsigned int BufferSize) {
    return client_->SetBufferSize(BufferSize);
}

viconSDK::Output_SetTimingLogFile ViconApi::setTimingLogFile(const viconSDK::String& ClientLog,
    const viconSDK::String& StreamLog) {
    return client_->SetTimingLogFile(ClientLog, StreamLog);
}

viconSDK::Output_GetSubjectCount ViconApi::getSubjectCount() const {
    return client_->GetSubjectCount();
}

viconSDK::Output_GetSubjectName ViconApi::getSubjectName(const unsigned int SubjectIndex) const {
    return client_->GetSubjectName(SubjectIndex);
}

viconSDK::Output_GetSegmentGlobalTranslation ViconApi::getSegmentGlobalTranslation(
    const viconSDK::String& SubjectName, const viconSDK::String& SegmentName) const {
    return client_->GetSegmentGlobalTranslation(SubjectName, SegmentName);
}

viconSDK::Output_GetSegmentGlobalRotationQuaternion ViconApi::getSegmentGlobalRotationQuaternion(
    const viconSDK::String& SubjectName, const viconSDK::String& SegmentName) const {
    return client_->GetSegmentGlobalRotationQuaternion(SubjectName, SegmentName);
}

viconSDK::Output_GetMarkerCount ViconApi::getMarkerCount(
    const viconSDK::String& SubjectName) const {
    return client_->GetMarkerCount(SubjectName);
}

viconSDK::Output_GetMarkerName ViconApi::getMarkerName(const viconSDK::String& SubjectName,
    const unsigned int MarkerIndex) const {
    return client_->GetMarkerName(SubjectName, MarkerIndex);
}

viconSDK::Output_GetMarkerGlobalTranslation ViconApi::getMarkerGlobalTranslation(
    const viconSDK::String& SubjectName, const viconSDK::String& MarkerName) const {
    return client_->GetMarkerGlobalTranslation(SubjectName, MarkerName);
}

}  // namespace crf::communication::viconapi
