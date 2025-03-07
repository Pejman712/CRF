/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alvaro Garcia Gonzalez BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#define REALSENSE_FILTER_MAX_RANGE 4.0f
#define FRAME_TIMEOUT 5000

#include <algorithm>
#include <librealsense2/rsutil.h>
#include <librealsense2/rs.h>
#include <librealsense2/rs_advanced_mode.h>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

#include "RGBDCameras/RealSenseCamera/D405RealSenseCamera.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

D405RealSenseCamera::D405RealSenseCamera(const std::string& serial_number) :
    RealSenseCamera(serial_number) {
    logger_->debug("CTor");
    logger_->info("Using a RealSense D405 camera");
}


cv::rgbd::RgbdFrame D405RealSenseCamera::captureImageAndDepth() {
    cv::rgbd::RgbdFrame rgbdframe = RealSenseCamera::captureImageAndDepth();
    // Map the distance in the camera to the distance unit used by the GUI (milimeters)
    rgbdframe.depth = rgbdframe.depth * 0.1f;
    return rgbdframe;
}

std::vector<Profile> D405RealSenseCamera::listProfiles() {
    return listDepthProfiles();
}

bool D405RealSenseCamera::setProfile(const Profile& imageProfile) {
    logger_->debug("setProfile(profile)");
    if (!initialized_) {
        logger_->error("Not initialized");
        return false;
    }
    logger_->error("Camera D405 has not RGB feed");
    return false;
}

bool D405RealSenseCamera::setProfile(const Profile& imageProfile, const Profile& depthProfile) {
    logger_->debug("setProfile(profile, profile)");
    if (!initialized_) {
        logger_->error("Not initialized");
        return false;
    }
    logger_->error("Camera D405 has not RGB feed, using only depth");
    return setDepthProfile(depthProfile);
}

void D405RealSenseCamera::parseProfileList() {
    logger_->debug("parseProfileList");
    // Depth
    rs2::depth_sensor depth_sensor = device_.first<rs2::depth_sensor>();
    std::vector<cv::Size> depthResolutions;
    for (auto&& profile : depth_sensor.get_stream_profiles()) {
        if (auto video = profile.as<rs2::video_stream_profile>()) {
            cv::Size resolution(video.width(), video.height());
            if (video.format() == RS2_FORMAT_Z16 &&
                std::find(depthResolutions.begin(), depthResolutions.end(), resolution) == depthResolutions.end()) {  // NOLINT
                // Search framerates for this resolution
                std::vector<uint64_t> framerates;
                for (auto&& p : depth_sensor.get_stream_profiles()) {
                    if (auto v = p.as<rs2::video_stream_profile>()) {
                        if (v.format() == RS2_FORMAT_Z16 &&
                            v.width() == resolution.width &&
                            v.height() == resolution.height) {
                            framerates.push_back(v.fps());
                        }
                    }
                }
                depthProfiles_.emplace_back(resolution, framerates);
                depthResolutions.push_back(resolution);
            }
        }
    }
}

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
