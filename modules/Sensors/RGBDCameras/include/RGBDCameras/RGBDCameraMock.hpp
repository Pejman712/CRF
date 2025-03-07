/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>
#include <utility>
#include <vector>

#include "RGBDCameras/IRGBDCamera.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

using crf::sensors::cameras::Profile;
using crf::sensors::cameras::Property;
class RGBDCameraMock : public IRGBDCamera {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));

    MOCK_METHOD(cv::Mat, captureImage, (), (override));
    MOCK_METHOD(cv::rgbd::RgbdFrame, captureImageAndDepth, (), (override));
    MOCK_METHOD(RGBDPointCloud, capturePointCloud, (), (override));
    MOCK_METHOD(bool, setProfile, (const Profile& profile), (override));
    MOCK_METHOD(bool, setProfile, (const Profile& profile, const Profile& profile2), (override));
    MOCK_METHOD(bool, setDepthProfile, (const Profile& profile), (override));

    MOCK_METHOD(crf::expected<Profile>, getProfile, (), (override));
    MOCK_METHOD(crf::expected<Profile>, getDepthProfile, (), (override));

    MOCK_METHOD(std::vector<Profile>, listProfiles, (), (override));
    MOCK_METHOD(std::vector<Profile>, listDepthProfiles, (), (override));

    MOCK_METHOD(crf::expected<bool>, setProperty, (const Property& property, const int& value), (override));  // NOLINT
    MOCK_METHOD(crf::expected<int>, getProperty, (const Property& property), (override));
    MOCK_METHOD(crf::expected<cv::Mat>, getColorCameraMatrix, (), (override));
    MOCK_METHOD(crf::expected<cv::Mat>, getColorDistortionMatrix, (), (override));
    MOCK_METHOD(crf::expected<cv::Mat>, getDepthCameraMatrix, (), (override));
    MOCK_METHOD(crf::expected<cv::Mat>, getDepthDistortionMatrix, (), (override));
    MOCK_METHOD(crf::expected<cv::Mat>, getDepth2ColorExtrinsics, (), (override));
};

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
