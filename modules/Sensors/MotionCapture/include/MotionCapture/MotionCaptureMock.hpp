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
#include <gmock/gmock.h>
#include "crf/expected.hpp"
#include "Types/TaskTypes/TaskPose.hpp"
#include "MotionCapture/IMotionCapture.hpp"

namespace crf::sensors::motioncapture {

class MotionCaptureMock : public IMotionCapture {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));
    MOCK_METHOD(crf::expected<std::vector<std::string>>, getObjectNames, (), ());
    MOCK_METHOD(crf::expected<crf::utility::types::TaskPose>, getObjectPose,
        (const std::string objectName), ());
    MOCK_METHOD(crf::expected<std::vector<MotionCaptureMarker>>, getObjectMarkers,
        (const std::string objectName), ());
};

}  // namespace crf::sensors::motioncapture
