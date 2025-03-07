/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 * ====================================================================
*/

#pragma once

#include <vector>
#include <string>
#include <gmock/gmock.h>
#include "EventLogger/EventLogger.hpp"
#include "ViconAPI/ViconApiMock.hpp"
#include "ViconAPI/ViconMarker.hpp"
#include "ViconAPI/ViconObject.hpp"

using testing::_;
using testing::Invoke;

namespace crf::communication::viconapi {

class ViconApiMockConfiguration : public ViconApiMock {
 public:
    ViconApiMockConfiguration():
        logger_("ViconApiMockConfiguration"),
        error_(false),
        connected_(false),
        timeoutExpired_(false) {
            logger_->info("CTor");
        }

    ~ViconApiMockConfiguration() {
        logger_->info("DTor");
    }

    void goingToError() {
        error_ = true;
    }

    void goingToNotConnected() {
        connected_ = false;
    }

    void setGetFrameTimeoutExpired() {
        timeoutExpired_ = true;
    }

    void setViconObject(ViconObject object) {
        objectList_.push_back(object);
    }

    void configureMock() {
        ON_CALL(*this, isConnected()).WillByDefault(Invoke(
            [this]() {
                viconSDK::Output_IsConnected output;
                if (connected_) {
                    connected_ = true;
                    output.Connected = true;
                } else {
                    connected_ = false;
                    output.Connected = false;
                }
                return output;
            }));
        ON_CALL(*this, setConnectionTimeout(_)).WillByDefault(Invoke(
            [this](unsigned int Timeout) {
                viconSDK::Output_SetConnectionTimeout output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                } else {
                    output.Result = viconSDK::Result::Success;
                }
                return output;
            }));
        ON_CALL(*this, disconnect()).WillByDefault(Invoke(
            [this]() {
                viconSDK::Output_Disconnect output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                    connected_ = true;
                } else {
                    output.Result = viconSDK::Result::Success;
                    connected_ = false;
                }
                return output;
            }));
        ON_CALL(*this, connect(_)).WillByDefault(Invoke(
            [this](const viconSDK::String& HostName) {
                logger_->debug("Connecting to " + (std::string)HostName);
                viconSDK::Output_Connect output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                    connected_ = false;
                } else {
                    output.Result = viconSDK::Result::Success;
                    connected_ = true;
                }
                return output;
            }));
        ON_CALL(*this, getFrame()).WillByDefault(Invoke(
            [this]() {
                viconSDK::Output_GetFrame output;
                if (timeoutExpired_) {
                    output.Result = viconSDK::Result::Unknown;
                    timeoutExpired_ = false;
                } else {
                    output.Result = viconSDK::Result::Success;
                }
                return output;
            }));
        ON_CALL(*this, enableSegmentData()).WillByDefault(Invoke(
            [this]() {
                viconSDK::Output_EnableSegmentData output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                } else {
                    output.Result = viconSDK::Result::Success;
                }
                return output;
            }));
        ON_CALL(*this, enableMarkerData()).WillByDefault(Invoke(
            [this]() {
                viconSDK::Output_EnableMarkerData output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                } else {
                    output.Result = viconSDK::Result::Success;
                }
                return output;
            }));
        ON_CALL(*this, enableUnlabeledMarkerData()).WillByDefault(Invoke(
            [this]() {
                viconSDK::Output_EnableUnlabeledMarkerData output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                } else {
                    output.Result = viconSDK::Result::Success;
                }
                return output;
            }));
        ON_CALL(*this, enableLightweightSegmentData()).WillByDefault(Invoke(
            [this]() {
                viconSDK::Output_EnableLightweightSegmentData output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                } else {
                    output.Result = viconSDK::Result::Success;
                }
                return output;
            }));
        ON_CALL(*this, setStreamMode(_)).WillByDefault(Invoke(
            [this](const viconSDK::StreamMode::Enum Mode) {
                viconSDK::Output_SetStreamMode output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                } else {
                    output.Result = viconSDK::Result::Success;
                }
                return output;
            }));
        ON_CALL(*this, setAxisMapping(_, _, _)).WillByDefault(Invoke(
            [this](const viconSDK::Direction::Enum XAxis,
                   const viconSDK::Direction::Enum YAxis,
                   const viconSDK::Direction::Enum ZAxis) {
                viconSDK::Output_SetAxisMapping output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                } else {
                    output.Result = viconSDK::Result::Success;
                }
                return output;
            }));
        ON_CALL(*this, setTimingLogFile(_, _)).WillByDefault(Invoke(
            [this](const viconSDK::String& ClientLog,
                   const viconSDK::String& StreamLog) {
                viconSDK::Output_SetTimingLogFile output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                } else {
                    output.Result = viconSDK::Result::Success;
                }
                return output;
            }));
        ON_CALL(*this, getSubjectCount()).WillByDefault(Invoke(
            [this]() {
                viconSDK::Output_GetSubjectCount output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                    output.SubjectCount = 0;
                } else {
                    output.Result = viconSDK::Result::Success;
                    output.SubjectCount = objectList_.size();
                }
                return output;
            }));
        ON_CALL(*this, getSubjectName(_)).WillByDefault(Invoke(
            [this](const unsigned int SubjectIndex) {
                viconSDK::Output_GetSubjectName output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                    output.SubjectName = "";
                } else {
                    output.Result = viconSDK::Result::Success;
                    output.SubjectName = objectList_[SubjectIndex].objectName;
                }
                return output;
            }));
        ON_CALL(*this, getSegmentGlobalTranslation(_, _)).WillByDefault(Invoke(
            [this](const viconSDK::String& SubjectName, const viconSDK::String& SegmentName) {
                viconSDK::Output_GetSegmentGlobalTranslation output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                    output.Translation[0] = 0;
                    output.Translation[1] = 0;
                    output.Translation[2] = 0;
                    output.Occluded = true;
                } else {
                    output.Result = viconSDK::Result::Success;
                    int index = getObjectIndex(SubjectName);
                    output.Translation[0] =
                        objectList_[getObjectIndex(SubjectName)].objectTranslation[0];
                    output.Translation[1] =
                        objectList_[getObjectIndex(SubjectName)].objectTranslation[1];
                    output.Translation[2] =
                        objectList_[getObjectIndex(SubjectName)].objectTranslation[2];
                    output.Occluded =
                        objectList_[getObjectIndex(SubjectName)].objectOccluded;
                }
                return output;
            }));
        ON_CALL(*this, getSegmentGlobalRotationQuaternion(_, _)).WillByDefault(Invoke(
            [this](const viconSDK::String& SubjectName, const viconSDK::String& SegmentName) {
                viconSDK::Output_GetSegmentGlobalRotationQuaternion output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                    output.Rotation[0] = 1;
                    output.Rotation[1] = 0;
                    output.Rotation[2] = 0;
                    output.Rotation[3] = 0;
                    output.Occluded = true;
                } else {
                    output.Result = viconSDK::Result::Success;
                    output.Rotation[0] =
                        objectList_[getObjectIndex(SubjectName)].objectQuaternion[3];
                    output.Rotation[1] =
                        objectList_[getObjectIndex(SubjectName)].objectQuaternion[0];
                    output.Rotation[2] =
                        objectList_[getObjectIndex(SubjectName)].objectQuaternion[1];
                    output.Rotation[3] =
                        objectList_[getObjectIndex(SubjectName)].objectQuaternion[2];
                    output.Occluded =
                        objectList_[getObjectIndex(SubjectName)].objectOccluded;
                }
                return output;
            }));
        ON_CALL(*this, getMarkerCount(_)).WillByDefault(Invoke(
            [this](const viconSDK::String& SubjectName) {
                viconSDK::Output_GetMarkerCount output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                    output.MarkerCount = 0;
                } else {
                    output.Result = viconSDK::Result::Success;
                    output.MarkerCount =
                        objectList_[getObjectIndex(SubjectName)].objectMarkers.size();
                }
                return output;
            }));
        ON_CALL(*this, getMarkerName(_, _)).WillByDefault(Invoke(
            [this](const viconSDK::String& SubjectName,
                   const unsigned int MarkerIndex) {
                viconSDK::Output_GetMarkerName output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                    output.MarkerName = "";
                } else {
                    output.Result = viconSDK::Result::Success;
                    output.MarkerName = objectList_[getObjectIndex(SubjectName)].objectName +
                        std::to_string(MarkerIndex+1);
                }
                return output;
            }));
        ON_CALL(*this, getMarkerGlobalTranslation(_, _)).WillByDefault(Invoke(
            [this](const viconSDK::String& SubjectName, const viconSDK::String& MarkerName) {
                viconSDK::Output_GetMarkerGlobalTranslation output;
                if (error_) {
                    output.Result = viconSDK::Result::Unknown;
                    output.Translation[0] = 0;
                    output.Translation[1] = 0;
                    output.Translation[2] = 0;
                    output.Occluded = true;
                } else {
                    output.Result = viconSDK::Result::Success;
                    ViconObject obj = objectList_[getObjectIndex(SubjectName)];
                    output.Translation[0] =
                        obj.objectMarkers[getMarkerIndex(obj, MarkerName)].markerTranslation[0];
                    output.Translation[1] =
                        obj.objectMarkers[getMarkerIndex(obj, MarkerName)].markerTranslation[1];
                    output.Translation[2] =
                        obj.objectMarkers[getMarkerIndex(obj, MarkerName)].markerTranslation[2];
                    output.Occluded =
                        obj.objectMarkers[getMarkerIndex(obj, MarkerName)].markerOccluded;
                }
                return output;
            }));
        ON_CALL(*this, setBufferSize(_)).WillByDefault(Invoke(
            [this](unsigned int BufferSize) {
                return;
            }));
    }

 private:
    crf::utility::logger::EventLogger logger_;
    bool error_;
    bool connected_;
    bool timeoutExpired_;
    std::vector<ViconObject> objectList_;

    int getObjectIndex(const std::string name) {
        int index = 0;
        bool nameFound = false;
        while (index < objectList_.size() && nameFound == false) {
            if (objectList_[index].objectName == name) {
                nameFound = true;
            }
            index++;
        }
        if (nameFound)
            return index-1;
        else
            return 0;
    }

    int getMarkerIndex(const ViconObject obj, const std::string name) {
        int index = 0;
        bool nameFound = false;
        while (index < obj.objectMarkers.size() && nameFound == false) {
            if (obj.objectMarkers[index].markerName == name) {
                nameFound = true;
            }
            index++;
        }
        if (nameFound)
            return index-1;
        else
            return 0;
    }
};

}  // namespace crf::communication::viconapi
