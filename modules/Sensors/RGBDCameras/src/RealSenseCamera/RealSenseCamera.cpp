/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *         Carlos Prados Sesmero CERN EN/SMM/MRO 2020
 * Contributor: Alvaro Garcia Gonzalez BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#define FRAME_TIMEOUT 5000
#define REALSENSE_FILTER_MAX_RANGE 4.0f

#include <algorithm>
#include <librealsense2/rsutil.h>
#include <librealsense2/rs.h>
#include <librealsense2/rs_advanced_mode.h>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

#include "RGBDCameras/RealSenseCamera/RealSenseCamera.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

RealSenseCamera::RealSenseCamera(const std::string& serialNumber) :
    logger_("RealSenseCamera"),
    initialized_(false) {
    logger_->debug("CTor");
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();

    bool found = false;
    logger_->info("size {}", devices.size());

    if (devices.size() == 0) {
        logger_->error("No real sense cameras have been detected");
        throw std::runtime_error("RealSenseCamera(): No real sense cameras have been detected!");
    }

    for (rs2::device device : devices) {
        std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        logger_->info("Camera found, serial number {}", serial);
        if (serial == serialNumber) {
            device_ = device;
            found = true;
            break;
        }
    }

    std::vector<rs2::sensor> sensors = device_.query_sensors();
    rs2_camera_info info;
    for (auto&& sensor : sensors) {
        logger_->info("Camera sensors: {}", sensor.get_info(info));
    }

    if (!found) {
        logger_->error("The device with serial number {} does not exist", serialNumber);
        throw std::runtime_error("Invalid serial number");
    }
}

RealSenseCamera::~RealSenseCamera() {
    logger_->debug("DTor");
    deinitialize();
}

bool RealSenseCamera::initialize() {
    logger_->debug("initialize()");
    if (initialized_) return true;

    parseProfileList();

    // Configure the pipeline
    config_.enable_device(device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    config_.disable_all_streams();
    config_.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);
    config_.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16);

    // Start the pipeline
    pipeline_.start(config_);

    // Get the color and depth profiles
    color_profile_ = pipeline_.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();  // NOLINT
    depth_profile_ = pipeline_.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();  // NOLINT

    // Get the camera intrinsics and extrinsics
    color_intrinsics_ = color_profile_.get_intrinsics();
    depth_intrinsics_ = depth_profile_.get_intrinsics();
    depth2color_extrinsics_ = depth_profile_.get_extrinsics_to(color_profile_);

    logger_->info("Realsense correctly initialized");
    initialized_ = true;
    return true;
}

bool RealSenseCamera::deinitialize() {
    logger_->debug("deinitialize()");
    if (!initialized_) return true;

    try {
        pipeline_.stop();
    } catch (const std::exception& ex) {
        logger_->warn("Failed to stop pipeline: {}", ex.what());
        return false;
    }
    config_.disable_all_streams();
    initialized_ = false;
    return true;
}

cv::Mat RealSenseCamera::captureImage() {
    rs2::frameset frames;
    try {
        frames = pipeline_.wait_for_frames(FRAME_TIMEOUT);
    } catch (const rs2::error& ex) {
        logger_->warn("Failed to grab frame: {}", ex.what());
        return cv::Mat();
    }
    rs2::frame color_frame = frames.get_color_frame();
    cv::Mat color = cv::Mat(
        cv::Size(color_intrinsics_.width, color_intrinsics_.height),
        CV_8UC3, const_cast<void*>(color_frame.get_data()), cv::Mat::AUTO_STEP);
    return color;
}

cv::rgbd::RgbdFrame RealSenseCamera::captureImageAndDepth() {
    rs2::frameset frames;
    try {
        frames = pipeline_.wait_for_frames(FRAME_TIMEOUT);
    } catch (const rs2::error& ex) {
        logger_->warn("Failed to grab frame: {}", ex.what());
        return cv::rgbd::RgbdFrame();
    }
    rs2::frame color_frame = frames.get_color_frame();
    rs2::frame depth_frame = frames.get_depth_frame();
    cv::Mat color = cv::Mat(
        cv::Size(color_intrinsics_.width, color_intrinsics_.height),
        CV_8UC3, const_cast<void*>(color_frame.get_data()), cv::Mat::AUTO_STEP);
    cv::Mat depth = cv::Mat(
        cv::Size(depth_intrinsics_.width, depth_intrinsics_.height),
        CV_16UC1, const_cast<void*>(depth_frame.get_data()), cv::Mat::AUTO_STEP);
    return cv::rgbd::RgbdFrame(color, depth);
}

RGBDPointCloud RealSenseCamera::capturePointCloud() {
    cv::rgbd::RgbdFrame frame = captureImageAndDepth();
    crf::expected<cv::Mat> depthCameraMatrix = getDepthCameraMatrix();
    if (!depthCameraMatrix) {
        logger_->error("Could not obtain depth camera matrix!");
        return RGBDPointCloud();
    }
    if (frame.depth.empty() || frame.depth.empty()) {
        logger_->error("Could not obtain rgb or depth from camera!");
        return RGBDPointCloud();
    }
    cv::Mat points;
    cv::rgbd::depthTo3d(frame.depth, depthCameraMatrix.value(), points);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>(
        frame.depth.cols, frame.depth.rows));
    cloud->is_dense = false;
    float color_point[3];
    float color_pixel[2];

    pcl::PointXYZRGBA invalid_point;
    invalid_point.x = std::numeric_limits<float>::quiet_NaN();
    invalid_point.y = std::numeric_limits<float>::quiet_NaN();
    invalid_point.z = std::numeric_limits<float>::quiet_NaN();

    for (int dy = 0; dy < frame.depth.rows; dy++) {
        for (int dx = 0; dx < frame.depth.cols; dx++) {
            auto p = reinterpret_cast<const cv::Point3f*>(points.ptr(dy, dx));
            if ((p->z == 0) || (isnanf(p->z)) || (p->z > REALSENSE_FILTER_MAX_RANGE)) {
                cloud->at(dx, dy) = invalid_point;
            }

            float depth_point[3] = { p->x, p->y, p->z };
            rs2_transform_point_to_point(color_point, &depth2color_extrinsics_, depth_point);
            rs2_project_point_to_pixel(color_pixel, &color_intrinsics_, color_point);
            auto cx = static_cast<int>(color_pixel[0]);
            auto cy = static_cast<int>(color_pixel[1]);
            if (cx < 0 || cx >= frame.image.cols || cy < 0 || cy >= frame.image.rows) {
                cloud->at(dx, dy).rgb = 1.0;;
            } else {
                cloud->at(dx, dy).r = frame.image.at<cv::Vec3b>(cy, cx)[2];
                cloud->at(dx, dy).g = frame.image.at<cv::Vec3b>(cy, cx)[1];
                cloud->at(dx, dy).b = frame.image.at<cv::Vec3b>(cy, cx)[0];

                cloud->at(dx, dy).x = depth_point[0];
                cloud->at(dx, dy).y = depth_point[1];
                cloud->at(dx, dy).z = depth_point[2];
                cloud->at(dx, dy).a = 255;
            }
        }
    }
    RGBDPointCloud result;
    result.frame = frame;
    result.pointcloud = cloud;
    return result;
}

bool RealSenseCamera::setProfile(const Profile& imageProfile) {
    logger_->debug("setProfile(profile)");
    if (!initialized_) {
        logger_->error("Not initialized");
        return false;
    }

    pipeline_.stop();
    config_.disable_all_streams();
    config_.enable_stream(RS2_STREAM_COLOR, imageProfile.resolution.width,
        imageProfile.resolution.height, RS2_FORMAT_BGR8, imageProfile.framerate);

    config_.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16, 0);  // Enable any depth stream
    pipeline_.start(config_);

    color_profile_ = pipeline_.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();  // NOLINT
    depth_profile_ = pipeline_.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();  // NOLINT

    // Get the camera intrinsics
    color_intrinsics_ = color_profile_.get_intrinsics();
    depth_intrinsics_ = depth_profile_.get_intrinsics();
    depth2color_extrinsics_ = depth_profile_.get_extrinsics_to(color_profile_);
    return true;
}

bool RealSenseCamera::setProfile(const Profile& imageProfile, const Profile& depthProfile) {
    logger_->debug("setProfile(profile, profile)");
    if (!initialized_) {
        logger_->error("Not initialized");
        return false;
    }

    pipeline_.stop();
    config_.disable_all_streams();
    config_.enable_stream(RS2_STREAM_COLOR, imageProfile.resolution.width,
        imageProfile.resolution.height, RS2_FORMAT_BGR8, imageProfile.framerate);
    config_.enable_stream(RS2_STREAM_DEPTH, depthProfile.resolution.width,
        depthProfile.resolution.height, RS2_FORMAT_Z16, depthProfile.framerate);

    pipeline_.start(config_);

    color_profile_ = pipeline_.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();  // NOLINT
    depth_profile_ = pipeline_.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();  // NOLINT

    // Get the camera intrinsics
    color_intrinsics_ = color_profile_.get_intrinsics();
    depth_intrinsics_ = depth_profile_.get_intrinsics();
    depth2color_extrinsics_ = depth_profile_.get_extrinsics_to(color_profile_);
    return true;
}

bool RealSenseCamera::setDepthProfile(const Profile& depthProfile) {
    logger_->debug("setDepthProfile(profile)");
    if (!initialized_) {
        logger_->error("Not initialized");
        return false;
    }
    pipeline_.stop();
    config_.disable_all_streams();
    config_.enable_stream(RS2_STREAM_DEPTH, depthProfile.resolution.width,
        depthProfile.resolution.height, RS2_FORMAT_Z16, depthProfile.framerate);
    config_.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8, 0);  // Enable any color stream
    pipeline_.start(config_);

    // Get the camera intrinsics
    color_profile_ = pipeline_.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();  // NOLINT
    depth_profile_ = pipeline_.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();  // NOLINT

    color_intrinsics_ = color_profile_.get_intrinsics();
    depth_intrinsics_ = depth_profile_.get_intrinsics();
    depth2color_extrinsics_ = depth_profile_.get_extrinsics_to(color_profile_);
    return true;
}

crf::expected<Profile> RealSenseCamera::getProfile() {
    if (!initialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    return Profile(
        cv::Size(color_profile_.width(), color_profile_.height()),
        color_profile_.fps());
}

crf::expected<Profile> RealSenseCamera::getDepthProfile() {
    if (!initialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    return Profile(
        cv::Size(depth_profile_.width(), depth_profile_.height()),
        depth_profile_.fps());
}

std::vector<Profile> RealSenseCamera::listProfiles() {;
    if (!initialized_) {
        logger_->info("Not initialized");
        return std::vector<Profile>();
    }

    return imageProfiles_;
}

std::vector<Profile> RealSenseCamera::listDepthProfiles() {
    if (!initialized_) {
        logger_->info("Not initialized");
        return std::vector<Profile>();
    }

    return depthProfiles_;
}

crf::expected<bool> RealSenseCamera::setProperty(const Property& property, const int& value) {
    return crf::Code::NotImplemented;
}

crf::expected<int> RealSenseCamera::getProperty(const Property& property) {
    return crf::Code::NotImplemented;
}

crf::expected<cv::Mat> RealSenseCamera::getColorCameraMatrix() {
    if (color_intrinsics_.model == RS2_DISTORTION_NONE) {
        // Then the mat should be identity??
        return crf::Code::ThirdPartyQueryFailed;
    }

    cv::Mat intrinsics = cv::Mat::eye(3, 3, CV_32F);
    intrinsics.at<float>(0, 0) = color_intrinsics_.fx;
    intrinsics.at<float>(1, 1) = color_intrinsics_.fy;
    intrinsics.at<float>(0, 2) = color_intrinsics_.ppx;
    intrinsics.at<float>(1, 2) = color_intrinsics_.ppy;
    return intrinsics;
}

crf::expected<cv::Mat> RealSenseCamera::getColorDistortionMatrix() {
    if (color_intrinsics_.model != RS2_DISTORTION_NONE) {
        return crf::Code::ThirdPartyQueryFailed;
    }

    cv::Mat distortion = cv::Mat::zeros(1, 5, CV_32F);
    distortion.at<float>(0, 0) = color_intrinsics_.coeffs[0];
    distortion.at<float>(0, 1) = color_intrinsics_.coeffs[1];
    distortion.at<float>(0, 2) = color_intrinsics_.coeffs[2];
    distortion.at<float>(0, 3) = color_intrinsics_.coeffs[3];
    distortion.at<float>(0, 4) = color_intrinsics_.coeffs[4];
    return distortion;
}

crf::expected<cv::Mat> RealSenseCamera::getDepthCameraMatrix() {
    if (depth_intrinsics_.model == RS2_DISTORTION_NONE) {
        return crf::Code::ThirdPartyQueryFailed;
    }

    cv::Mat intrinsics = cv::Mat::eye(3, 3, CV_32F);
    intrinsics.at<float>(0, 0) = depth_intrinsics_.fx;
    intrinsics.at<float>(1, 1) = depth_intrinsics_.fy;
    intrinsics.at<float>(0, 2) = depth_intrinsics_.ppx;
    intrinsics.at<float>(1, 2) = depth_intrinsics_.ppy;
    return intrinsics;
}

crf::expected<cv::Mat> RealSenseCamera::getDepthDistortionMatrix() {
    if (depth_intrinsics_.model != RS2_DISTORTION_NONE) {
        return crf::Code::ThirdPartyQueryFailed;
    }
    cv::Mat distortion = cv::Mat::zeros(1, 5, CV_32F);
    distortion.at<float>(0, 0) = depth_intrinsics_.coeffs[0];
    distortion.at<float>(0, 1) = depth_intrinsics_.coeffs[1];
    distortion.at<float>(0, 2) = depth_intrinsics_.coeffs[2];
    distortion.at<float>(0, 3) = depth_intrinsics_.coeffs[3];
    distortion.at<float>(0, 4) = depth_intrinsics_.coeffs[4];
    return distortion;
}

crf::expected<cv::Mat> RealSenseCamera::getDepth2ColorExtrinsics() {
    if (depth2color_extrinsics_.rotation[0] == 0.0f) {
        return crf::Code::ThirdPartyQueryFailed;
    }
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat T = cv::Mat::zeros(3, 1, CV_32F);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R.at<float>(i, j) = depth2color_extrinsics_.rotation[i * 3 + j];
        }
        T.at<float>(i, 0) = depth2color_extrinsics_.translation[i];
    }
    cv::Mat transformation = cv::Mat::eye(4, 4, CV_32F);
    R.copyTo(transformation(cv::Rect(0, 0, 3, 3)));
    T.copyTo(transformation(cv::Rect(3, 0, 1, 3)));
    return transformation;
}

// Protected

void RealSenseCamera::parseProfileList() {
    logger_->debug("parseProfileList");
    // Color
    rs2::sensor color_sensor = device_.first<rs2::color_sensor>();
    std::vector<cv::Size> imageResolutions;
    for (auto&& profile : color_sensor.get_stream_profiles()) {
        if (auto video = profile.as<rs2::video_stream_profile>()) {
            cv::Size resolution(video.width(), video.height());
            if (video.format() == RS2_FORMAT_BGR8 &&
                std::find(imageResolutions.begin(), imageResolutions.end(), resolution) == imageResolutions.end()) {  // NOLINT
                // Search framerates for this resolution
                std::vector<uint64_t> framerates;
                for (auto&& p : color_sensor.get_stream_profiles()) {
                    if (auto v = p.as<rs2::video_stream_profile>()) {
                        if (v.format() == RS2_FORMAT_BGR8 &&
                            v.width() == resolution.width &&
                            v.height() == resolution.height) {
                            framerates.push_back(v.fps());
                        }
                    }
                }
                imageProfiles_.emplace_back(resolution, framerates);
                imageResolutions.push_back(resolution);
            }
        }
    }

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
