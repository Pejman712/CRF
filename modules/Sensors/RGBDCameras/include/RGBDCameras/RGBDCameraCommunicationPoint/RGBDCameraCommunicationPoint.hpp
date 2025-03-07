/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
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

#include "Cameras/CameraCommunicationPoint/CameraCommunicationPoint.hpp"
#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraManager.hpp"
#include "DataPackets/RGBDFramePacket/RGBDFramePacket.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "VideoCodecs/IVideoEncoder.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

/**
 * @ingroup group_rgbdcamera_communication_point
 * @brief Communication point for the RGBD cameras, it extends the already exitsting
 * camera communication point and expands it.
 *
 * @details This class inherits from CameraCommunicationPoint
 *
 * @{
 */
class RGBDCameraCommunicationPoint : public cameras::CameraCommunicationPoint {
 public:
    RGBDCameraCommunicationPoint() = delete;
    RGBDCameraCommunicationPoint(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<RGBDCameraManager> manager);
    ~RGBDCameraCommunicationPoint() override;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<RGBDCameraManager> manager_;

    communication::datapackets::RGBDFramePacket::RGBEncoding rgbEncoding_;
    communication::datapackets::RGBDFramePacket::DepthEncoding depthEncoding_;
    communication::datapackets::RGBDFramePacket::PointCloudEncoding pointcloudEncoding_;

    std::atomic<bool> depthRequested_;
    std::atomic<bool> pointCloudRequested_;
    std::atomic<float> subsampling_;

    /**
     * @brief Frame streamer to send RGBD frames with depth and pointclouds
     *
     * @param encoder Selected encoder
     */
    void frameStreamer(
        std::shared_ptr<crf::vision::videocodecs::IVideoEncoder> encoder) override;

    /**
     * @brief Callback to start a frame stream for the client
     *
     */
    void startFrameStreamRequestHandler(const communication::datapackets::JSONPacket&) override;

    /**
     * @brief Callback to stop a frame stream for the client
     *
     */
    void setProfileRequestHandler(const communication::datapackets::JSONPacket&) override;

    /**
     * @brief Function to use the LZ4 algorightm to compress an string of bytes into a smaller one
     *
     * @param bytes Bytes to compress
     * @return std::string Compressed bytes
     */
    std::string compressLZ4(const std::string& bytes);
};

/**@}*/

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
