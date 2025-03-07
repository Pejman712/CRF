/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 * ====================================================================
*/


#pragma once

#include "ViconAPI/IViconApi.hpp"
#include <memory>

namespace crf::communication::viconapi {

/**
 * @ingroup group_vicon_api
 * @brief API class of Vicon to access the functions of the library.
 * 
 */
class ViconApi : public IViconApi {
 public:
    ViconApi();
    ViconApi(const ViconApi&) = delete;
    ViconApi(ViconApi&&) = delete;
    ~ViconApi() override = default;

    viconSDK::Output_IsConnected isConnected() const override;
    viconSDK::Output_Disconnect disconnect() override;
    viconSDK::Output_SetConnectionTimeout setConnectionTimeout(unsigned int Timeout) override;
    viconSDK::Output_Connect connect(const viconSDK::String& HostName) override;
    viconSDK::Output_GetFrame getFrame() override;
    viconSDK::Output_EnableSegmentData enableSegmentData() override;
    viconSDK::Output_EnableMarkerData enableMarkerData() override;
    viconSDK::Output_EnableUnlabeledMarkerData enableUnlabeledMarkerData() override;
    viconSDK::Output_EnableLightweightSegmentData enableLightweightSegmentData() override;
    viconSDK::Output_SetStreamMode setStreamMode(const viconSDK::StreamMode::Enum Mode) override;
    viconSDK::Output_SetAxisMapping setAxisMapping(const viconSDK::Direction::Enum XAxis,
        const viconSDK::Direction::Enum YAxis, const viconSDK::Direction::Enum ZAxis) override;
    void setBufferSize(unsigned int BufferSize) override;
    viconSDK::Output_SetTimingLogFile setTimingLogFile(const viconSDK::String& ClientLog,
        const viconSDK::String& StreamLog) override;
    viconSDK::Output_GetSubjectCount getSubjectCount() const override;
    viconSDK::Output_GetSubjectName getSubjectName(const unsigned int SubjectIndex) const override;
    viconSDK::Output_GetSegmentGlobalTranslation getSegmentGlobalTranslation(
        const viconSDK::String& SubjectName, const viconSDK::String& SegmentName) const override;
    viconSDK::Output_GetSegmentGlobalRotationQuaternion getSegmentGlobalRotationQuaternion(
        const viconSDK::String& SubjectName, const viconSDK::String& SegmentName) const override;
    viconSDK::Output_GetMarkerCount getMarkerCount(
        const viconSDK::String& SubjectName) const override;
    viconSDK::Output_GetMarkerName getMarkerName(const viconSDK::String& SubjectName,
        const unsigned int MarkerIndex) const override;
    viconSDK::Output_GetMarkerGlobalTranslation getMarkerGlobalTranslation(
        const viconSDK::String& SubjectName, const viconSDK::String& MarkerName) const override;

 private:
    std::shared_ptr<viconSDK::Client> client_;
};

}  // namespace crf::communication::viconapi
