/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi & Carlos Prados Sesmero CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <lz4.h>
#include <memory>
#include <string>
#include <vector>

#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraCommunicationPoint.hpp"
#include "VisionUtility/Image/ImageJSONConverter.hpp"
#include "CommunicationUtility/StreamWriter.hpp"
#include "VideoCodecs/JPEGVideoCodec/JPEGVideoEncoder.hpp"
#include "VideoCodecs/x264VideoCodec/x264VideoEncoder.hpp"
#include "VideoCodecs/cvMatVideoCodec/cvMatVideoEncoder.hpp"
#include "VisionUtility/PointCloud/Communication.hpp"
#include "VisionUtility/PointCloud/Subsample.hpp"

using crf::communication::datapackets::RGBDFramePacket;
using crf::communication::datapackets::FramePacket;
using crf::utility::visionutility::pointcloud::communication::serializePointCloud;
using crf::utility::visionutility::pointcloud::subsample::voxelGridSubsample;

namespace crf {
namespace sensors {
namespace rgbdcameras {

RGBDCameraCommunicationPoint::RGBDCameraCommunicationPoint(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<RGBDCameraManager> manager) :
    CameraCommunicationPoint(socket, manager),
    logger_("RGBDCameraCommunicationPoint"),
    manager_(manager),
    depthRequested_(false),
    pointCloudRequested_(false),
    subsampling_(0) {
    logger_->debug("CTor");
    rgbEncoding_ = RGBDFramePacket::RGBEncoding::CV_MAT;
    depthEncoding_ = RGBDFramePacket::DepthEncoding::CV_MAT;
    pointcloudEncoding_ = RGBDFramePacket::PointCloudEncoding::PLY;
}

RGBDCameraCommunicationPoint::~RGBDCameraCommunicationPoint() {
    logger_->debug("DTor");
    CameraCommunicationPoint::deinitialize();
}

void RGBDCameraCommunicationPoint::startFrameStreamRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("startFrameStreamRequestHandler()");

    if (!stopFrameStream_) {
        sendJSONError(packet,
            "The stream has already been started, please stop it before starting a new one");
        return;
    }

    try {
        float subsampling = 0.0;
        bool depthRequested = false;
        bool pointCloudRequested = false;

        Profile depthProfile;
        nlohmann::json request = packet.data;
        logger_->info("{}", request);

        Profile imageProfile = request["image_profile"].get<Profile>();
        crf::vision::videocodecs::CompressionQuality quality =
            request["encoding_quality"].get<crf::vision::videocodecs::CompressionQuality>();
        rgbEncoding_ = request["encoding_format"].get<
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding>();

        std::shared_ptr<crf::vision::videocodecs::IVideoEncoder> imageEncoder = getEncoder(
            static_cast<crf::communication::datapackets::FramePacket::Encoding>(rgbEncoding_),
            imageProfile.resolution, quality);

        // Check depth
        if (request.contains("depth_profile")) {
            depthRequested = true;
            depthProfile = request["depth_profile"].get<Profile>();
            depthEncoding_ = request["depth_format"].get<
                    crf::communication::datapackets::RGBDFramePacket::DepthEncoding>();
        }

        // Check pointcloud
        if (request.contains("pointcloud_format") && request.contains("pointcloud_subsampling")) {
            pointCloudRequested = true;
            pointcloudEncoding_ = request["pointcloud_format"].get<
                crf::communication::datapackets::RGBDFramePacket::PointCloudEncoding>();

            subsampling = request["pointcloud_subsampling"].get<float>();
            if (subsampling <= 0.0f) {
                logger_->error("Pointcloud subsampling must be greater than 0.0");
                sendJSONError(packet, "Pointcloud subsampling must be greater than 0.0");
                return;
            }
        }

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
        bool streamStarted;
        if (depthRequested) {
            streamStarted = manager_->requestFrameStream(
                streamID_, imageProfile, depthProfile,  pointCloudRequested_);
        } else {
            streamStarted = manager_->requestFrameStream(streamID_, imageProfile);
        }

        // If fail send the error
        if (!streamStarted) {
            // Free the ID if the frame streamer is not running
            if (stopFrameStream_) {
                std::scoped_lock lock(availableStreamsMutex_);
                availableStreams_[streamID_] = true;
                streamID_ = -1;
            }
            sendJSONError(packet, "Unsupported image profile");
            return;
        }

        streamFramerate_ = imageProfile.framerate;
        pointCloudRequested_ = pointCloudRequested;
        depthRequested_ = depthRequested;
        subsampling_ = subsampling;

        startFrameStreamer(imageEncoder);
        sendJSONReply<bool>(packet, true);
    } catch (const std::exception& e) {
        logger_->error("startFrameStream(): Wrong stream parameters");
        sendJSONError(packet, "Wrong stream parameters");
        return;
    }
}

void RGBDCameraCommunicationPoint::setProfileRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setProfileRequestHandler({})", packet.data);
    try {
        bool pointCloudRequested = false;
        bool depthRequested = false;
        float subsampling = 0.0;
        Profile imageProfile;
        Profile depthProfile;
        if (!packet.data.contains("image_profile")) {
            sendJSONError(packet, "Field image_profile is not present");
            return;
        }
        imageProfile = packet.data["image_profile"].get<Profile>();
        if (imageProfile.framerates.size() != 1) {
            sendJSONError(packet, "Can't set more or less than one framerate");
            return;
        }
        if (packet.data.contains("depth_profile")) {
            depthRequested = true;
            depthProfile = packet.data["depth_profile"].get<Profile>();
            if (depthProfile.framerates.size() != 1) {
                sendJSONError(packet, "Can't set more or less than one framerate");
                return;
            }
            if (packet.data.contains("pointcloud_subsampling")) {
                pointCloudRequested = true;
                subsampling = packet.data["pointcloud_subsampling"].get<float>();
                if (subsampling <= 0.0f) {
                    logger_->error("Pointcloud subsampling must be greater than 0.0");
                    sendJSONError(packet, "Pointcloud subsampling must be greater than 0.0");
                    return;
                }
            }
        }

        if (depthRequested) {
            if (manager_->requestFrameStream(
                streamID_, imageProfile, depthProfile, pointCloudRequested)) {
                streamFramerate_ = imageProfile.framerate;
                pointCloudRequested_ = pointCloudRequested;
                depthRequested_ = depthRequested;
                subsampling_ = subsampling;
                sendJSONReply<bool>(packet, true);
            } else {
                sendJSONReply<bool>(packet, false);
            }
        } else {
            if (manager_->requestFrameStream(streamID_, imageProfile)) {
                streamFramerate_ = imageProfile.framerate;
                pointCloudRequested_ = pointCloudRequested;
                depthRequested_ = depthRequested;
                subsampling_ = subsampling;
                sendJSONReply<bool>(packet, true);
            } else {
                sendJSONReply<bool>(packet, false);
            }
        }
    } catch (const std::exception& e) {
        sendJSONError(packet, "setProfile(): Wrong parameters: {}", e.what());
        return;
    }
}

//  Private

void RGBDCameraCommunicationPoint::frameStreamer(
    std::shared_ptr<crf::vision::videocodecs::IVideoEncoder> encoder) {
    logger_->debug("frameStreamer");

    RGBDPointCloud rgbdFrame;
    RGBDFramePacket packet;
    std::string pointcloudBytes;

    while (!stopFrameStream_) {
        auto start = std::chrono::high_resolution_clock::now();

        // Get camera frame
        if (pointCloudRequested_) {
            rgbdFrame = manager_->getRGBDFrameAndPointCloud(streamID_);
        } else if (depthRequested_) {
            rgbdFrame.frame = manager_->getRGBDFrame(streamID_);
            rgbdFrame.pointcloud.reset();
        } else {
            rgbdFrame.frame.image = manager_->getFrame(streamID_);
        }

        if (rgbdFrame.frame.image.empty()) continue;

        // Add Image
        encoder->addFrame(rgbdFrame.frame.image);
        std::string encodedImage = encoder->getBytes();
        packet.setRGBBytes(rgbEncoding_, encodedImage);

        // If depth add it
        if (depthRequested_) {
            crf::utility::communicationutility::StreamWriter writer;
            writer.write(rgbdFrame.frame.depth);
            std::string bytes = writer.toString();
            if (depthEncoding_ == RGBDFramePacket::DepthEncoding::LZ4)
                bytes = compressLZ4(bytes);
            packet.setDepthBytes(depthEncoding_, bytes);
        }

        // If pointcloud add it
        if (pointCloudRequested_) {
            rgbdFrame.pointcloud = voxelGridSubsample<pcl::PointXYZRGBA>(
                rgbdFrame.pointcloud, {subsampling_, subsampling_, subsampling_});

            if (rgbdFrame.pointcloud != nullptr) {
                Eigen::Vector4f origin(0.0, 0.0, 0.0, 0.0);
                Eigen::Quaternionf orientation(Eigen::Matrix3f::Identity());
                std::ostringstream oss = serializePointCloud<pcl::PointXYZRGBA>(
                    rgbdFrame.pointcloud, origin, orientation, false);

                pointcloudBytes = oss.str();
                if (pointcloudEncoding_ == crf::communication::datapackets::
                    RGBDFramePacket::PointCloudEncoding::LZ4) {
                    pointcloudBytes = compressLZ4(pointcloudBytes);
                }
            }
            packet.setPointCloudBytes(pointcloudEncoding_, pointcloudBytes);
        }

        socket_->write(packet, packet.getHeader(), false);

        auto end = std::chrono::high_resolution_clock::now();
        auto loopDuration = end-start;
        if (loopDuration < std::chrono::milliseconds(1000/streamFramerate_)) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1000/streamFramerate_) - loopDuration);
        }
    }
    encoder->flush();
}

std::string RGBDCameraCommunicationPoint::compressLZ4(const std::string& bytes) {
    const int maxDstSize = LZ4_compressBound(bytes.length());
    char* compressedData = new char[maxDstSize];
    const int compressedDataSize = LZ4_compress_default(bytes.c_str(), compressedData,
        bytes.length(), maxDstSize);
    std::string compressedString;
    compressedString.resize(4);
    int32_t decompressedLength = static_cast<int32_t>(bytes.length());
    std::memcpy(const_cast<char*>(compressedString.c_str()), &decompressedLength, 4);
    compressedString.append(std::string(compressedData, compressedDataSize));
    delete[] compressedData;
    return compressedString;
}

}   // namespace rgbdcameras
}   // namespace sensors
}   // namespace crf
