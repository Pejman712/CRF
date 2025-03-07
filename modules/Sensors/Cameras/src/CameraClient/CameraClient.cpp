/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Álvaro García González CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include "Cameras/CameraClient/CameraClient.hpp"

namespace crf {
namespace sensors {
namespace cameras {

CameraClient::CameraClient(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    const std::chrono::milliseconds& serverReplyTimeout,
    const float& frequency,
    const uint32_t& priority):
    crf::utility::devicemanager::PriorityAccessClient(
        socket,
        serverReplyTimeout,
        frequency,
        priority),
    socket_(socket),
    serverReplyTimeout_(serverReplyTimeout),
    priority_(priority),
    logger_("CameraClient") {
    logger_->debug("CTor");
        // When more decoders are available we should allow the user to select
        decoder_ = std::make_shared<vision::videocodecs::JPEGVideoDecoder>();
        packetHandlers_.insert({
            communication::datapackets::FRAME_PACKET,
            std::bind(&CameraClient::parseFramePacket, this, std::placeholders::_1)});

        receivers_.insert({"startFrameStream", &receiverStartFrameStream_});
        receivers_.insert({"stopFrameStream", &receiverStopFrameStream_});
        receivers_.insert({"setProperty", &receiverSetProperty_});
}

CameraClient::~CameraClient() {
    logger_->debug("DTor");
    deinitialize();
}

bool CameraClient::initialize() {
    if (!PriorityAccessClient::initialize()) return false;
    listProfiles();
    if (profiles_.size() == 0) {
        logger_->error("No profiles recovered for this camera");
        return false;
    }
    return setProfile(profiles_[0]);
}

bool CameraClient::deinitialize() {
    stopFrameStream();
    return PriorityAccessClient::deinitialize();
}

cv::Mat CameraClient::captureImage() {
    if (!initialized_) {
        logger_->error("Not initialized");
        return cv::Mat();
    }
    std::unique_lock<std::mutex> lock(imageMtx_);
    return image_;
}

bool CameraClient::setProfile(const Profile& profile) {
    if (!initialized_) {
        logger_->error("Not initialized");
        return false;
    }
    return startFrameStream(profile);
}

crf::expected<Profile> CameraClient::getProfile() {
    logger_->debug("getProfile");
    if (!initialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    if (!requestStatus()) return crf::Code::RequestToDeviceFailed;
    return currentProfile_;
}

std::vector<Profile> CameraClient::listProfiles() {
    logger_->debug("listProfiles");
    requestStatus();
    return profiles_;
}

crf::expected<bool> CameraClient::setProperty(const Property& property, const int& value) {
    logger_->debug("setProperty");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setProperty";
    json.data["priority"] = priority_;
    json.data["properties"][nameMap_.at(property)] = value;
    return sendPriorityPacket(json, receiverSetProperty_);
}

crf::expected<int> CameraClient::getProperty(const Property& property) {
    logger_->debug("getProperty");
    if (!requestStatus()) return crf::Code::RequestToDeviceFailed;
    if (nameMap_.find(property) == nameMap_.end())
        return crf::Code::NotAcceptable;
    return propertyMap_.at(nameMap_.at(property)).value;
}

// Private

bool CameraClient::startFrameStream(const Profile& profile) {
    logger_->debug("startFrameStream");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["image_profile"] = profile;
    json.data["encoding_quality"] = 5;
    json.data["encoding_format"] = 1;  // JPEG
    return sendPacket(json, receiverStartFrameStream_);
}

bool CameraClient::stopFrameStream() {
    logger_->debug("stopFrameStream");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "stopFrameStream";
    return sendPacket(json, receiverStopFrameStream_);
}

void CameraClient::parseStatus(const nlohmann::json& json) {
    try {
        currentProfile_ = json["current_profile"].get<crf::expected<Profile>>();
        nlohmann::json properties = json.at("property");
        for (auto& property : properties.items()) {
            propertyMap_[property.key()].value = property.value().get<crf::expected<int>>();
        }
        profiles_ = json["available_profiles"].get<std::vector<Profile>>();
    } catch (const std::exception& ex) {
        logger_->warn("Failed reading json response: {}", ex.what());
    }
}

void CameraClient::parseFramePacket(const std::string& buffer) {
    communication::datapackets::FramePacket frame;
    if (!frame.deserialize(buffer)) {
        logger_->error("Failed to deserialize frame packet");
        return;
    }
    if (frame.getEncodingType() != communication::datapackets::FramePacket::Encoding::JPEG) {
        logger_->error("Frame encodingis not JPEG as requested");
        return;
    }
    if (!decoder_->addBytes(frame.getBytes())) {
        logger_->warn("Failed to add bytes to decoder");
        return;
    }
    cv::Mat decoded = decoder_->getFrame();
    std::unique_lock<std::mutex> lock(imageMtx_);
    image_ = decoded;
}

bool CameraClient::sendPacket(const communication::datapackets::JSONPacket& json,
    const Receiver<bool>& receiver) {
    logger_->debug("sendPacket: {}", json.data);
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader(), false);
    socketLock.unlock();

    std::optional<bool> result = receiver.waitFor(serverReplyTimeout_);
    if (!result) {
        logger_->error("Could not receive answer for {}", json.data);
        return false;
    }
    return result.value();
}

bool CameraClient::sendPriorityPacket(const communication::datapackets::JSONPacket& json,
    const Receiver<bool>& receiver) {
    logger_->debug("sendPriorityPacket: {}", json.data);
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!lockControl()) {
        return false;
    }
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader(), false);
    socketLock.unlock();

    std::optional<bool> result = receiver.waitFor(serverReplyTimeout_);
    if (!result) {
        logger_->error("Could not receive answer for {}", json.data);
        return false;
    }
    unlockControl();
    return result.value();
}

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
