/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alvaro Garcia Gonzalez BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */


#pragma once

#include <string>
#include <vector>

#include <librealsense2/rs.hpp>
#include "RGBDCameras/IRGBDCamera.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

using crf::sensors::cameras::Profile;
using crf::sensors::cameras::Property;

class RealSenseCamera : public IRGBDCamera {
 public:
    explicit RealSenseCamera(const std::string& serial_number);
    ~RealSenseCamera();

    bool initialize() override;
    bool deinitialize() override;

    cv::Mat captureImage() override;
    cv::rgbd::RgbdFrame captureImageAndDepth() override;
    RGBDPointCloud capturePointCloud() override;

    bool setProfile(const Profile& profile) override;
    bool setProfile(const Profile& streamprofile, const Profile& depthprofile) override;
    bool setDepthProfile(const Profile& profile) override;
    crf::expected<Profile> getProfile() override;
    crf::expected<Profile> getDepthProfile() override;
    std::vector<Profile> listProfiles() override;
    std::vector<Profile> listDepthProfiles() override;
    crf::expected<bool> setProperty(const Property& property, const int& value) override;
    crf::expected<int> getProperty(const Property& property) override;

    crf::expected<cv::Mat> getColorCameraMatrix() override;
    crf::expected<cv::Mat> getColorDistortionMatrix() override;
    crf::expected<cv::Mat> getDepthCameraMatrix() override;
    crf::expected<cv::Mat> getDepthDistortionMatrix() override;
    crf::expected<cv::Mat> getDepth2ColorExtrinsics() override;

 protected:
    utility::logger::EventLogger logger_;
    bool initialized_;
    rs2::pipeline pipeline_;
    rs2::config config_;
    rs2::device device_;
    rs2::video_stream_profile color_profile_;
    rs2::video_stream_profile depth_profile_;
    rs2_intrinsics color_intrinsics_;
    rs2_intrinsics depth_intrinsics_;
    rs2_extrinsics depth2color_extrinsics_;

    std::vector<Profile> imageProfiles_;
    std::vector<Profile> depthProfiles_;

    virtual void parseProfileList();
};

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
