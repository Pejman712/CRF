/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 * Contributor: Alvaro Garcia Gonzalez BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "DeviceManager/DeviceManagerCommunicationPoint/PriorityAccessCommunicationPoint.hpp"
#include "Cameras/CameraCommunicationPoint/CameraManager.hpp"
#include "DataPackets/FramePacket/FramePacket.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "VideoCodecs/IVideoEncoder.hpp"

namespace crf::sensors::cameras {

/**
 * @ingroup group_camera_communication_point
 * @brief Communication point for the cameras. One instance of this class
 * is generated per client.
 *
 * @details This class inherits from PriorityAccessCommunicationPoint
 * @{
 */

class CameraCommunicationPoint :
    public utility::devicemanager::PriorityAccessCommunicationPoint {
 public:
    CameraCommunicationPoint() = delete;
    /**
     * @brief Construct a new Camera Communication Point object
     *
     * @param socket Socket where the communication point will listen
     * @param manager Manager that controls the camera and access to it
     */
    CameraCommunicationPoint(std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<CameraManager> manager);
    ~CameraCommunicationPoint() override;
    bool deinitialize() override;

 protected:
    static std::vector<bool> availableStreams_;
    static std::mutex availableStreamsMutex_;
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    std::atomic<int> streamID_;
    Profile streamProfile_;

    std::atomic<bool> stopFrameStream_;
    std::atomic<uint64_t> streamFramerate_;
    std::thread frameStreamerThread_;
    communication::datapackets::FramePacket::Encoding encodingType_;

    /**
     * @brief Callback function that starts a frame streamer for the client
     *
     */
    virtual void startFrameStreamRequestHandler(const communication::datapackets::JSONPacket&);

    /**
     * @brief Callback function to change the profile of the stream
     *
     */
    virtual void setProfileRequestHandler(const communication::datapackets::JSONPacket&);

    /**
     * @brief Function to start the frame stream
     *
     * @param encoder Encoder selected by the user
     */
    void startFrameStreamer(std::shared_ptr<crf::vision::videocodecs::IVideoEncoder> encoder);

    /**
     * @brief Function to stop the frame stream
     */
    void stopFrameStreamer();

    /**
     * @brief Frame streamer that periodically sends frames to the user following the framerate
     * selected
     *
     * @param encoder Encoder selected
     */
    virtual void frameStreamer(std::shared_ptr<crf::vision::videocodecs::IVideoEncoder> encoder);

    /**
     * @brief Get the Encoder object
     *
     * @param encoding Type of encoder selected
     * @param resolution Resolution selected, some encoders need it
     * @param quality Compression quality selected
     * @return std::shared_ptr<crf::vision::videocodecs::IVideoEncoder> And instance of the encoder
     */
    std::shared_ptr<crf::vision::videocodecs::IVideoEncoder> getEncoder(
        const crf::communication::datapackets::FramePacket::Encoding& encoding,
        const cv::Size& resolution,
        const crf::vision::videocodecs::CompressionQuality& quality);

 private:
    std::shared_ptr<CameraManager> manager_;
    utility::logger::EventLogger logger_;

    void setPropertyRequestHandler(const communication::datapackets::JSONPacket&);
    void stopFrameStreamRequestHandler(const communication::datapackets::JSONPacket&);

    void sendFrame(
        const std::string& bytes, crf::communication::datapackets::FramePacket::Encoding encoding);
};

/**@}*/

}  // namespace crf::sensors::cameras
