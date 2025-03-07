/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 * ====================================================================
*/

#pragma once

#include <string>
#include <vector>
#include "crf/expected.hpp"
#include "crf/ResponseCode.hpp"
#include "Types/TaskTypes/TaskPose.hpp"
#include "EventLogger/EventLogger.hpp"
#include "MotionCapture/MotionCaptureMock.hpp"
#include "MotionCapture/MotionCaptureMarker.hpp"
#include "MotionCapture/MotionCaptureObject.hpp"

using testing::_;
using testing::Invoke;

namespace crf::sensors::motioncapture {

class MotionCaptureMockConfiguration : public MotionCaptureMock {
 public:
    MotionCaptureMockConfiguration():
        logger_("MotionCaptureMockConfiguration"),
        error_(false),
        initialized_(false) {
            logger_->info("CTor");
        }

    ~MotionCaptureMockConfiguration() {
        logger_->info("DTor");
    }

    void goingToError() {
        error_ = true;
    }

    void setMotionCapureObject(MotionCaptureObject object) {
        objectList_.push_back(object);
    }

    void configureMock() {
        ON_CALL(*this, initialize()).WillByDefault(Invoke(
            [this] {
                if (initialized_) return false;
                initialized_ = true;
                return true;
            }));
        ON_CALL(*this, deinitialize()).WillByDefault(Invoke(
            [this] {
                if (!initialized_) return false;
                initialized_ = false;
                return true;
            }));
        ON_CALL(*this, getObjectNames()).WillByDefault(Invoke(
            [this]() -> crf::expected<std::vector<std::string>> {
                if (error_) return crf::Code::ThirdPartyQueryFailed;
                std::vector<std::string> objectNamesList;
                for (int index = 0; index < objectList_.size(); index++) {
                    objectNamesList.push_back(objectList_[index].objectName);
                }
                return objectNamesList;
            }));
        ON_CALL(*this, getObjectPose(_)).WillByDefault(Invoke(
            [this](const std::string objectName)
            -> crf::expected<crf::utility::types::TaskPose> {
                if (error_) return crf::Code::ThirdPartyQueryFailed;
                bool objectNameFound = false;
                int index = 0;
                while (index < objectList_.size() && objectNameFound == false) {
                    if (objectList_[index].objectName == objectName) {
                        return objectList_[index].objectPose;
                    }
                    index++;
                }
                return crf::Code::ThirdPartyQueryFailed;
            }));
        ON_CALL(*this, getObjectMarkers(_)).WillByDefault(Invoke(
            [this](const std::string objectName)->crf::expected<std::vector<MotionCaptureMarker>> {
                if (error_) return crf::Code::ThirdPartyQueryFailed;
                bool objectNameFound = false;
                int index = 0;
                std::vector<MotionCaptureMarker> markerList;
                while (index < objectList_.size() && objectNameFound == false) {
                    if (objectList_[index].objectName == objectName) {
                        return objectList_[index].objectMarkers;
                    }
                    index++;
                }
                return crf::Code::ThirdPartyQueryFailed;
            }));
    }

 private:
    crf::utility::logger::EventLogger logger_;
    std::vector<MotionCaptureObject> objectList_;
    bool error_;
    bool initialized_;
};

}  // namespace crf::sensors::motioncapture
