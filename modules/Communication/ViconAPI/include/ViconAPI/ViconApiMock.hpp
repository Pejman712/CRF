/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 * ====================================================================
*/


#pragma once

#include <gmock/gmock.h>
#include "ViconAPI/IViconApi.hpp"

namespace crf::communication::viconapi {

class ViconApiMock : public IViconApi {
 public:
    MOCK_METHOD(viconSDK::Output_IsConnected, isConnected, (), (const, override));
    MOCK_METHOD(viconSDK::Output_Disconnect, disconnect, (), (override));
    MOCK_METHOD(viconSDK::Output_SetConnectionTimeout, setConnectionTimeout,
        (unsigned int Timeout), (override));
    MOCK_METHOD(viconSDK::Output_Connect, connect, (const viconSDK::String& HostName), (override));
    MOCK_METHOD(viconSDK::Output_GetFrame, getFrame, (), (override));
    MOCK_METHOD(viconSDK::Output_EnableSegmentData, enableSegmentData, (), (override));
    MOCK_METHOD(viconSDK::Output_EnableMarkerData, enableMarkerData, (), (override));
    MOCK_METHOD(viconSDK::Output_EnableUnlabeledMarkerData, enableUnlabeledMarkerData, (),
        (override));
    MOCK_METHOD(viconSDK::Output_EnableLightweightSegmentData, enableLightweightSegmentData, (),
        (override));
    MOCK_METHOD(viconSDK::Output_SetStreamMode, setStreamMode,
        (const viconSDK::StreamMode::Enum Mode), (override));
    MOCK_METHOD(viconSDK::Output_SetAxisMapping, setAxisMapping,
        (const viconSDK::Direction::Enum XAxis, const viconSDK::Direction::Enum YAxis,
        const viconSDK::Direction::Enum ZAxis), (override));
    MOCK_METHOD(void, setBufferSize, (unsigned int BufferSize), (override));
    MOCK_METHOD(viconSDK::Output_SetTimingLogFile, setTimingLogFile,
        (const viconSDK::String & ClientLog, const viconSDK::String& StreamLog), (override));
    MOCK_METHOD(viconSDK::Output_GetSubjectCount, getSubjectCount, (), (const, override));
    MOCK_METHOD(viconSDK::Output_GetSubjectName, getSubjectName,
        (const unsigned int SubjectIndex), (const, override));
    MOCK_METHOD(viconSDK::Output_GetSegmentGlobalTranslation, getSegmentGlobalTranslation,
        (const viconSDK::String& SubjectName, const viconSDK::String& SegmentName),
        (const, override));
    MOCK_METHOD(viconSDK::Output_GetSegmentGlobalRotationQuaternion,
        getSegmentGlobalRotationQuaternion, (const viconSDK::String& SubjectName,
        const viconSDK::String& SegmentName), (const, override));
    MOCK_METHOD(viconSDK::Output_GetMarkerCount, getMarkerCount,
        (const viconSDK::String& SubjectName), (const, override));
    MOCK_METHOD(viconSDK::Output_GetMarkerName, getMarkerName,
        (const viconSDK::String& SubjectName, const unsigned int MarkerIndex), (const, override));
    MOCK_METHOD(viconSDK::Output_GetMarkerGlobalTranslation, getMarkerGlobalTranslation,
        (const viconSDK::String& SubjectName, const viconSDK::String& MarkerName),
        (const, override));
};

}  // namespace crf::communication::viconapi
