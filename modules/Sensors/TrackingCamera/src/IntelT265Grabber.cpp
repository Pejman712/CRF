/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <vector>
#include <fstream>
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <nlohmann/json.hpp>
#include <boost/optional.hpp>

#include "TrackingCamera/ContextFactory.hpp"
#include "TrackingCamera/TrackingCameraUtils.hpp"
#include "TrackingCamera/IntelT265Grabber.hpp"

namespace crf {
namespace sensors {
namespace trackingcamera {

IntelT265Grabber::IntelT265Grabber(const nlohmann::json &configCamera):
    logger_("TrackingCameraGrabber"),
    pipe_(crf::sensors::realsense::ContextFactory::getContext()),
    configCamera_(configCamera),
    isRunning_(false) {
    logger_->debug("CTor");
    rs2::log_to_console(rs2_log_severity::RS2_LOG_SEVERITY_WARN);
    std::string serialNumber = configCamera_.at("serialNumber").get<std::string>();
    cfg_.enable_device(serialNumber);
    configureStreams();
}

IntelT265Grabber::~IntelT265Grabber() {
    logger_->debug("DTor");
    if (isRunning_) {
        deinitialize();
    }
}

boost::optional<rs2_extrinsics> IntelT265Grabber::getCameraExtrinsics(bool cameraSelection) {
    if (isRunning_) {
        if (cameraSelection == false) {
            return streams_.cam1_extrin;
        } else {
            return streams_.cam2_extrin;
        }
    }
    return boost::none;
}

boost::optional<rs2_intrinsics> IntelT265Grabber::getCameraIntrinsics(bool cameraSelection) {
    if (isRunning_) {
        if (cameraSelection == false) {
            return streams_.cam1_intrin;
        } else {
            return streams_.cam2_intrin;
        }
    }
    return boost::none;
}

bool IntelT265Grabber::initialize() {
    if (isRunning_) {
        logger_->warn("The grabber was already initialized");
        return false;
    }

    // Start the pipeline with the configuration
    rs2::pipeline_profile profile;
    try {
        profile = pipe_.start(cfg_);
    } catch (const rs2::error e) {
        logger_->error("There was a problem to start the camera: {0}", e.what());
        return false;
    }

    rs2::device device = profile.get_device();

    if (configCamera_.count("wheelsOdometryCalibr") > 0) {
        odometer_ = std::make_shared<rs2::wheel_odometer>(device.first<rs2::wheel_odometer>());

        std::string wheelsConfig = configCamera_.at("wheelsOdometryCalibr").get<std::string>();
        std::string testDirName = __FILE__;
        testDirName = testDirName.substr(0, testDirName.find("src/IntelT265Grabber.cpp"));
        testDirName += wheelsConfig;
        if (!initializeOdometryInput(testDirName)) {
            logger_->warn("Error when trying to initialize odometry sensors");
            return false;
        }
    }

    initializeStreams(profile);

    isRunning_ = true;
    logger_->info("initialized");
    return true;
}

void IntelT265Grabber::initializeStreams(const rs2::pipeline_profile &profile) {
    auto pose_stream = profile.get_stream(RS2_STREAM_POSE).as<rs2::video_stream_profile>();
    auto gyro_stream = profile.get_stream(RS2_STREAM_GYRO).as<rs2::video_stream_profile>();
    auto accel_stream = profile.get_stream(RS2_STREAM_ACCEL).as<rs2::video_stream_profile>();

    auto fisheye1_stream = profile.get_stream(RS2_STREAM_FISHEYE, 1).
        as<rs2::video_stream_profile>();
    auto fisheye2_stream = profile.get_stream(RS2_STREAM_FISHEYE, 2).
        as<rs2::video_stream_profile>();

    streams_.cam1_intrin = fisheye1_stream.get_intrinsics();
    streams_.cam2_intrin = fisheye2_stream.get_intrinsics();

    // This is the pose of the fisheye sensor relative to the T265 coordinate system.
    streams_.cam1_extrin = fisheye1_stream.get_extrinsics_to(profile.get_stream(RS2_STREAM_POSE));
    streams_.cam2_intrin = fisheye2_stream.get_intrinsics();
    streams_.cam2_extrin = fisheye2_stream.get_extrinsics_to(profile.get_stream(RS2_STREAM_POSE));
}


bool IntelT265Grabber::deinitialize() {
    if (!isRunning_) {
        logger_->warn("The grabber was already deinitialized");
        return false;
    }
    try {
        pipe_.stop();
    } catch (const rs2::error& e) {
        logger_->error("There was a problem to deinitialize the grabber: {0}", e.what());
        return false;
    }
    isRunning_ = false;
    return true;
}

bool IntelT265Grabber::resetPose() {
    if (!isRunning_) {
        logger_->warn("Cannot reset pose, because grabber has not been initialized");
        return false;
    }
    pipe_.stop();
    isRunning_ = false;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    pipe_.start();
    isRunning_ = true;
    return true;
}


TrackingData IntelT265Grabber::getTrackingData() {
    TrackingData actualTrackingData;
    if (!isRunning_) {
        logger_->warn("Application is not running");
        return actualTrackingData;
    }
    auto frameset = pipe_.wait_for_frames();

    if (rs2::pose_frame pose_frame = frameset.first_or_default(RS2_STREAM_POSE)) {
        rs2_pose pose_data = pose_frame.get_pose_data();
        actualTrackingData.pose =  pose_data;
    }
    if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL)) {
        actualTrackingData.acceleration = accel_frame.get_motion_data();
    }
    if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO)) {
        actualTrackingData.gyro = gyro_frame.get_motion_data();
    }
    return actualTrackingData;
}

bool IntelT265Grabber::feedBaseVelocity(const std::array<float, 3>& velocity) {
    if (!odometer_) {
        logger_->warn("Attempt to feedBaseVelocity, but no odometry info provided");
        return false;
    }
    rs2_vector baseVelocity{velocity[0], velocity[1], velocity[2]};
    try {
        odometer_->send_wheel_odometry(0, 0, baseVelocity);
    } catch (const rs2::error& e) {
        logger_->error("Failure when sending velocity to Tracking Camera: {0}", e.what());
        return false;
    }
    return true;
}

bool IntelT265Grabber::initializeOdometryInput(const std::string &calib_odom_file) {
    std::ifstream calibrationFile(calib_odom_file);
    const std::string json_str((std::istreambuf_iterator<char>(calibrationFile)),
        std::istreambuf_iterator<char>());
    const std::vector<uint8_t> wo_calib(json_str.begin(), json_str.end());

    try {
        odometer_->load_wheel_odometery_config(wo_calib);
    } catch (const rs2::error& e) {
        logger_->error("Format error in calibration_odometry file: {0}", e.what());
        return false;
    }
    logger_->info("Odometer sensor has been initialized");
    return true;
}

std::vector<cv::Mat> IntelT265Grabber::getVideoFrames() {
    std::vector<cv::Mat> newFrames;
    auto frames = pipe_.wait_for_frames();

    rs2::video_frame fisheye_frameL = frames.get_fisheye_frame(1);
    rs2::video_frame fisheye_frameR = frames.get_fisheye_frame(2);

    cv::Mat frameMatL;
    cv::Mat frameMatR;
    try {
        frameMatL = TrackingCameraUtils::frame_to_mat(fisheye_frameL);
        frameMatR = TrackingCameraUtils::frame_to_mat(fisheye_frameR);
    } catch (const rs2::error& e) {
        logger_->error("Error during conversion to cv::Mat: {0}", e.what());
        return newFrames;
    }
    newFrames.push_back(frameMatL);
    newFrames.push_back(frameMatR);

    return newFrames;
}

std::vector<cv::Mat> IntelT265Grabber::getRawVideoFrames() {
    std::vector<cv::Mat> newFrames;
    auto frames = pipe_.wait_for_frames();

    rs2::frame frameLeft = frames.get_fisheye_frame(1).get();
    rs2::frame frameRight = frames.get_fisheye_frame(2).get();
    cv::Mat frameMatL;
    cv::Mat frameMatR;
    try {
        frameMatL = TrackingCameraUtils::frame_to_mat(frameLeft);
        frameMatR = TrackingCameraUtils::frame_to_mat(frameRight);
    } catch (const rs2::error& e) {
        logger_->error("Error during conversion to cv::Mat: {0}", e.what());
        return newFrames;
    }
    newFrames.push_back(frameMatL);
    newFrames.push_back(frameMatR);
    return newFrames;
}

void IntelT265Grabber::configureStreams() {
    if (configCamera_.count("pose") > 0) {
        nlohmann::json poseConfig = configCamera_.at("pose");
        int width = poseConfig.at("width").get<int>();
        int height = poseConfig.at("height").get<int>();
        int fps = poseConfig.at("fps").get<int>();
        cfg_.enable_stream(RS2_STREAM_POSE, width, height, RS2_FORMAT_6DOF, fps);
        logger_->info("Enabled pose stream, width: {0}, height: {1}, fps {2}",
                width, height, fps);
    }
    if (configCamera_.count("accel") > 0) {
        nlohmann::json accelConfig = configCamera_.at("accel");
        int fps = accelConfig.at("fps").get<int>();
        cfg_.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, fps);
        logger_->info("Enabled accel stream, fps {0}", fps);
    }
    if (configCamera_.count("gyro") > 0) {
        nlohmann::json gyroConfig = configCamera_.at("gyro");
        int fps = gyroConfig.at("fps").get<int>();
        cfg_.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, fps);
        logger_->info("Enabled gyro stream. fps {0}", fps);
    }
    if (configCamera_.count("fisheye1") > 0) {
        nlohmann::json fisheye1 = configCamera_.at("fisheye1");
        int width = fisheye1.at("width").get<int>();
        int height = fisheye1.at("height").get<int>();
        int fps = fisheye1.at("fps").get<int>();
        cfg_.enable_stream(RS2_STREAM_FISHEYE, 1, width, height, RS2_FORMAT_Y8, fps);
        logger_->info("Enabled fisheye 1 stream, width: {0}, height: {1}, fps {2}",
                width, height, fps);
    }
    if (configCamera_.count("fisheye2") > 0) {
        nlohmann::json fisheye2 = configCamera_.at("fisheye2");
        int width = fisheye2.at("width").get<int>();
        int height = fisheye2.at("height").get<int>();
        int fps = fisheye2.at("fps").get<int>();
        cfg_.enable_stream(RS2_STREAM_FISHEYE, 2, width, height, RS2_FORMAT_Y8, fps);
        logger_->info("Enabled fisheye 2 stream, width: {0}, height: {1}, fps {2}",
                width, height, fps);
    }
}

}   // namespace trackingcamera
}   // namespace sensors
}   // namespace cern
