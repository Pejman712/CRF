/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 * Contributors: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
                 Alvaro Garcia Gonzalez CERN BE/CEM/MRO 2022
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>

#include "Cameras/CameraCommunicationPoint/CameraCommunicationPoint.hpp"
#include "VideoCodecs/JPEGVideoCodec/JPEGVideoEncoder.hpp"
#include "VideoCodecs/x264VideoCodec/x264VideoEncoder.hpp"
#include "VideoCodecs/cvMatVideoCodec/cvMatVideoEncoder.hpp"

namespace crf::sensors::cameras {

std::vector<bool> CameraCommunicationPoint::availableStreams_({
    true, true, true, true, true
});
std::mutex CameraCommunicationPoint::availableStreamsMutex_;

CameraCommunicationPoint::CameraCommunicationPoint(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<CameraManager> manager) :
    PriorityAccessCommunicationPoint(socket, manager),
    socket_(socket),
    streamID_(-1),
    streamProfile_(cv::Size(0, 0), 0),
    stopFrameStream_(true),
    encodingType_(communication::datapackets::FramePacket::Encoding::CV_MAT),
    manager_(manager),
    logger_("CameraCommunicationPoint") {
    logger_->debug("CTor");
    jsonCommandHandlers_.insert({"setProperty",
        std::bind(&CameraCommunicationPoint::setPropertyRequestHandler,
        this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({"setProfile",
        std::bind(&CameraCommunicationPoint::setProfileRequestHandler,
        this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({"startFrameStream",
        std::bind(&CameraCommunicationPoint::startFrameStreamRequestHandler,
        this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({"stopFrameStream",
        std::bind(&CameraCommunicationPoint::stopFrameStreamRequestHandler,
        this, std::placeholders::_1)});
}

CameraCommunicationPoint::~CameraCommunicationPoint() {
    logger_->debug("DTor");
    deinitialize();
}

bool CameraCommunicationPoint::deinitialize() {
    logger_->debug("deinitialize()");
    stopFrameStreamer();
    PriorityAccessCommunicationPoint::deinitialize();
    return true;
}

// Protected

void CameraCommunicationPoint::setProfileRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setProfileRequestHandler()");
    try {
        // Mandatory fields
        if (!packet.data.contains("image_profile"))
            throw std::invalid_argument("Field image_profile not found");

        Profile profile = packet.data["image_profile"].get<Profile>();
        if (profile.framerates.size() != 1) {
            sendJSONError(packet, "Can't set more or less than one framerate");
            return;
        }
        if (manager_->setProfile(streamID_, profile)) {
            streamFramerate_ = profile.framerate;
            sendJSONReply<bool>(packet, true);
        } else {
            sendJSONReply<bool>(packet, false);
        }
    } catch (const std::exception& e) {
        sendJSONError(packet, std::string("Wrong stream parameters: ") + e.what());
        return;
    }
}

void CameraCommunicationPoint::startFrameStreamRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("startFrameStreamRequestHandler()");
    nlohmann::json request = packet.data;
    try {
        // Mandatory fields
        if (!packet.data.contains("image_profile"))
            throw std::invalid_argument("Field image_profile not found");
        if (!packet.data.contains("encoding_quality"))
            throw std::invalid_argument("Field encoding_quality not found");
        if (!packet.data.contains("encoding_format"))
            throw std::invalid_argument("Field encoding_format not found");

        Profile profile = packet.data["image_profile"].get<Profile>();
        crf::vision::videocodecs::CompressionQuality quality =
            request["encoding_quality"].get<crf::vision::videocodecs::CompressionQuality>();

        if ((quality < 0) || (quality > 8)) {
            sendJSONError(packet, "Not permitted quality value: " + std::to_string(quality));
            return;
        }

        if (profile.framerate <= 0) {
            sendJSONError(packet, "Not permitted framerate value: " +
            std::to_string(profile.framerate));
            return;
        }

        crf::communication::datapackets::FramePacket::Encoding encoding =
            request["encoding_format"].get<communication::datapackets::FramePacket::Encoding>();

        std::shared_ptr<crf::vision::videocodecs::IVideoEncoder> encoder =
            getEncoder(encoding, profile.resolution, quality);

        {  // Acquire mutex for checking the streams
        std::scoped_lock lock(availableStreamsMutex_);
        if (streamID_ == -1) {  // If not having an ID already get the first available StreamID
            auto itr = std::find(availableStreams_.begin(), availableStreams_.end(), true);
            if (itr == availableStreams_.end()) {
                logger_->error("startFrameStream(): All streams are busy");
                sendJSONError(packet, "All streams are busy");
                return;
            }
            streamID_ = std::distance(availableStreams_.begin(), itr);
        }
        availableStreams_[streamID_] = false;
        }

        // Try to start stream
        if (!manager_->requestFrameStream(streamID_, profile)) {
            sendJSONError(packet, "Could not register stream with these parameters");
            // If there is not a frame streamer already running, free the stream ID
            if (stopFrameStream_) {
                std::scoped_lock lock(availableStreamsMutex_);
                availableStreams_[streamID_] = true;
                streamID_ = -1;
            }
            return;
        }

        streamFramerate_ = profile.framerate;
        startFrameStreamer(encoder);
        sendJSONReply<bool>(packet, true);
    } catch (const std::exception& e) {
        sendJSONError(packet, std::string("Wrong stream parameters: ") + e.what());
        return;
    }
}

void CameraCommunicationPoint::startFrameStreamer(
    std::shared_ptr<crf::vision::videocodecs::IVideoEncoder> encoder) {
    logger_->info("startFrameStreamer(id: {})", streamID_);
    if (!stopFrameStream_) return;
    stopFrameStream_ = false;
    if (frameStreamerThread_.joinable()) return;
    frameStreamerThread_ = std::thread(
        &CameraCommunicationPoint::frameStreamer, this, encoder);
}

void CameraCommunicationPoint::stopFrameStreamer() {
    logger_->info("stopFrameStreamer(id: {})", streamID_);
    if (stopFrameStream_) return;
    stopFrameStream_ = true;
    if (frameStreamerThread_.joinable()) frameStreamerThread_.join();
    manager_->removeFrameStream(streamID_);
    std::scoped_lock lock(availableStreamsMutex_);
    availableStreams_[streamID_] = true;
    streamID_ = -1;
}

void CameraCommunicationPoint::frameStreamer(
    std::shared_ptr<crf::vision::videocodecs::IVideoEncoder> encoder) {
    logger_->debug("frameStreamer()");
    cv::Mat frame;
    std::string bytes;
    while (!stopFrameStream_) {
        auto start = std::chrono::high_resolution_clock::now();

        frame = manager_->getFrame(streamID_);
        if (frame.empty()) continue;
        encoder->addFrame(frame);
        bytes = encoder->getBytes();
        if (bytes.length() == 0) continue;
        sendFrame(bytes, encodingType_);

        auto end = std::chrono::high_resolution_clock::now();
        auto loopDuration = end-start;
        if (loopDuration < std::chrono::milliseconds(1000/streamFramerate_)) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1000/streamFramerate_) - loopDuration);
        }
    }
    encoder->flush();
    logger_->debug("frameStreamer(): Exiting frameStreamer thread");
}

std::shared_ptr<crf::vision::videocodecs::IVideoEncoder> CameraCommunicationPoint::getEncoder(
    const crf::communication::datapackets::FramePacket::Encoding& encoding,
    const cv::Size& resolution,
    const crf::vision::videocodecs::CompressionQuality& quality) {
    std::shared_ptr<crf::vision::videocodecs::IVideoEncoder> encoder;
    if (encoding == crf::communication::datapackets::FramePacket::Encoding::JPEG) {
        encoder = std::make_shared<crf::vision::videocodecs::JPEGVideoEncoder>(quality);
    } else if (encoding == crf::communication::datapackets::FramePacket::Encoding::X264) {
        encoder = std::make_shared<crf::vision::videocodecs::x264VideoEncoder>(
            resolution, quality, true);
    } else {
        encoder = std::make_shared<crf::vision::videocodecs::cvMatVideoEncoder>();
    }
    encodingType_ = encoding;
    return encoder;
}

// Private

void CameraCommunicationPoint::stopFrameStreamRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("stopFrameStreamRequestHandler()");
    if (stopFrameStream_) {
        sendJSONError(packet, "The stream has not been started");
        return;
    }
    stopFrameStreamer();
    sendJSONReply<bool>(packet, true);
}

void CameraCommunicationPoint::setPropertyRequestHandler(
    const communication::datapackets::JSONPacket& packet)  {
    logger_->debug("setPropertyRequestHandler()");
    try {
        if (!packet.data.contains("priority"))
            throw std::invalid_argument("Field priority not found");

        uint32_t priority = packet.data.at("priority").get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
        if (!packet.data.contains("property")) {
            sendJSONError(packet, "Failed to set properties, properties field not found");
            return;
        }
        sendJSONReply<bool>(packet, manager_->setProperty(priority, packet.data["property"]));
    } catch (const std::exception& e) {
        sendJSONError(packet, std::string("Wrong stream parameters: ") + e.what());
        return;
    }
}

void CameraCommunicationPoint::sendFrame(
    const std::string& bytes, crf::communication::datapackets::FramePacket::Encoding encoding) {
    crf::communication::datapackets::FramePacket packet(bytes, encoding);
    socket_->write(packet, packet.getHeader(), false);
}

}   // namespace crf::sensors::cameras
