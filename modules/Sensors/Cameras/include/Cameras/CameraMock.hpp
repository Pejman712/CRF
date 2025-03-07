/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Álvaro García González CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>

#include <gmock/gmock.h>
#include <opencv2/opencv.hpp>
#include "Cameras/ICamera.hpp"

namespace crf {
namespace sensors {
namespace cameras {

class CameraMock : public ICamera {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));
    MOCK_METHOD(cv::Mat, captureImage, (), (override));
    MOCK_METHOD(bool, setProfile, (const Profile& profile), (override));
    MOCK_METHOD(crf::expected<Profile>, getProfile, (), (override));
    MOCK_METHOD(crf::expected<bool>, setProperty, (const Property& property, const int& value), (override));  // NOLINT
    MOCK_METHOD(crf::expected<int>, getProperty, (const Property& property), (override));
    MOCK_METHOD(std::vector<Profile>, listProfiles, (), (override));
};

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
