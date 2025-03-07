/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Álvaro García González CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include "RGBDCameras/RGBDCameraClient/RGBDCameraClient.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

RGBDCameraClient::RGBDCameraClient(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    const std::chrono::milliseconds& server_reply_timeout,
    const float& frequency,
    const uint32_t& priority):
    CameraClient(
        socket,
        server_reply_timeout,
        frequency, priority),
    logger_("RGBDCameraClient") {
    logger_->debug("CTOR");
    packetHandlers_.insert({
        communication::datapackets::RGBD_FRAME_PACKET,
        std::bind(&RGBDCameraClient::parseRGBDFramePacket, this, std::placeholders::_1)});
}

RGBDCameraClient::~RGBDCameraClient() {
    logger_->debug("DTor");
    deinitialize();
}

bool RGBDCameraClient::initialize() {
    if (!StatusStreamerClient::initialize()) return false;
    listDepthProfiles();
    if (depthProfiles_.size() == 0) {
        logger_->error("No depth profiles recovered for this camera");
        return false;
    }
    return setDepthProfile(depthProfiles_[0]);
}

bool RGBDCameraClient::deinitialize() {
    return CameraClient::deinitialize();
}

cv::Mat RGBDCameraClient::captureImage() {
    return CameraClient::captureImage();
}

cv::rgbd::RgbdFrame RGBDCameraClient::captureImageAndDepth() {
    if (!initialized_) {
        logger_->error("Not initialized");
        return cv::rgbd::RgbdFrame();
    }
    std::unique_lock<std::mutex> lock(imageMtx_);
    cv::rgbd::RgbdFrame frame;
    frame.depth = depthImage_.clone();
    frame.image = image_.clone();
    return frame;
}

RGBDPointCloud RGBDCameraClient::capturePointCloud() {
    throw std::logic_error("capturePointCloud() is not yet implemented");
}

bool RGBDCameraClient::setProfile(const Profile& profile) {
    return CameraClient::setProfile(profile);
}

bool RGBDCameraClient::setProfile(const Profile& streamprofile, const Profile& depthprofile) {
    if (!initialized_) {
        logger_->error("Not initialized");
        return false;
    }
    // No pointclouds for now
    return startFrameStream(streamprofile, depthprofile, false);
}

bool RGBDCameraClient::setDepthProfile(const Profile& profile) {
    if (!initialized_) {
        logger_->error("Not initialized");
        return false;
    }
    // We select the first image profile, no pointclouds for now
    return startFrameStream(profiles_[0], profile, false);
}

crf::expected<Profile> RGBDCameraClient::getProfile() {
    return CameraClient::getProfile();
}

crf::expected<Profile> RGBDCameraClient::getDepthProfile() {
    logger_->debug("getDepthProfile");
    if (!requestStatus()) return crf::Code::InternalServerError;
    return currentDepthProfile_;
}

std::vector<Profile> RGBDCameraClient::listProfiles() {
    return CameraClient::listProfiles();
}

std::vector<Profile> RGBDCameraClient::listDepthProfiles() {
    logger_->debug("listDepthProfiles");
    if (!requestStatus()) return std::vector<Profile>();
    return depthProfiles_;
}

crf::expected<bool> RGBDCameraClient::setProperty(const Property& property, const int& value) {
    return CameraClient::setProperty(property, value);
}

crf::expected<int> RGBDCameraClient::getProperty(const Property& property) {
    return CameraClient::getProperty(property);
}

crf::expected<cv::Mat> RGBDCameraClient::getColorCameraMatrix() {
    logger_->debug("getColorCameraMatrix");
    if (!requestStatus()) return crf::Code::InternalServerError;
    return colorCameraMatrix_;
}

crf::expected<cv::Mat> RGBDCameraClient::getColorDistortionMatrix() {
    logger_->debug("getColorDistortionMatrix");
    if (!requestStatus()) return crf::Code::InternalServerError;
    return colorDistortionMatrix_;
}

crf::expected<cv::Mat> RGBDCameraClient::getDepthCameraMatrix() {
    logger_->debug("getDepthCameraMatrix");
    if (!requestStatus()) return crf::Code::InternalServerError;
    return depthCameraMatrix_;
}

crf::expected<cv::Mat> RGBDCameraClient::getDepthDistortionMatrix() {
    logger_->debug("getDepthDistortionMatrix");
    if (!requestStatus()) return crf::Code::InternalServerError;
    return depthDistortionMatrix_;
}

crf::expected<cv::Mat> RGBDCameraClient::getDepth2ColorExtrinsics() {
    logger_->debug("getDepth2ColorExtrinsics");
    if (!requestStatus()) return crf::Code::InternalServerError;
    return extrinsic_;
}

// Private

bool RGBDCameraClient::startFrameStream(
    const Profile& streamprofile, const Profile& depthprofile, const bool& pointcloud) {
    logger_->debug("startFrameStream");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["image_profile"] = streamprofile;
    json.data["encoding_quality"] = 5;
    json.data["encoding_format"] = 1;
    json.data["depth_profile"] = depthprofile;
    json.data["depth_format"] = 1;
    if (pointcloud) {
        json.data["pointcloud_format"] = 1;
        json.data["pointcloud_subsampling"] = 40;
    }
    return sendPacket(json, receiverStartFrameStream_);
}

void RGBDCameraClient::parseStatus(const nlohmann::json& json) {
    logger_->debug("{}", json);
    CameraClient::parseStatus(json);
    try {
        depthProfiles_ = json["available_depth_profiles"].get<std::vector<Profile>>();
        currentDepthProfile_ = json["current_depth_profile"].get<crf::expected<Profile>>();
        colorCameraMatrix_ = json["color_intrinsic"].get<crf::expected<cv::Mat>>();
        colorDistortionMatrix_ = json["color_distortion"].get<crf::expected<cv::Mat>>();
        depthCameraMatrix_ = json["depth_intrinsic"].get<crf::expected<cv::Mat>>();
        depthDistortionMatrix_ = json["depth_distortion"].get<crf::expected<cv::Mat>>();
        extrinsic_ = json["extrinsic"].get<crf::expected<cv::Mat>>();
    } catch (const std::exception& e) {
        logger_->error("Error parsing status! {}", e.what());
    }
}

void RGBDCameraClient::parseRGBDFramePacket(const std::string& buffer) {
    communication::datapackets::RGBDFramePacket rgbdframe;
    if (!rgbdframe.deserialize(buffer)) {
        logger_->error("Failed to deserialize frame packet");
        return;
    }

    if (rgbdframe.containsRGB()) {
        if (rgbdframe.getRGBEncoding() != communication::datapackets::RGBDFramePacket::RGBEncoding::JPEG) {  // NOLINT
            logger_->warn("Frame encoding is not JPEG as requested");
            return;
        }
        if (!decoder_->addBytes(rgbdframe.getRGBBytes())) {
            logger_->warn("Failed to add bytes to decoder");
            return;
        }
        cv::Mat decoded = decoder_->getFrame();
        std::unique_lock<std::mutex> lock(imageMtx_);
        image_ = decoded;
    }
    if (rgbdframe.containsDepth()) {
        if (rgbdframe.getDepthEncoding() != communication::datapackets::RGBDFramePacket::DepthEncoding::CV_MAT) {  // NOLINT
            logger_->warn("Frame encoding is not CV_MAT as requested");
            return;
        }
        crf::utility::communicationutility::StreamReader reader(rgbdframe.getDepthBytes());
        std::unique_lock<std::mutex> lock(imageMtx_);
        reader.read<cv::Mat>(&depthImage_);
    }
}

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
