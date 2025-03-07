/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Álvaro García González CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <nlohmann/json.hpp>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <map>

#include "Cameras/ICamera.hpp"
#include "DeviceManager/DeviceManagerClient/PriorityAccessClient.hpp"
#include "DataPackets/FramePacket/FramePacket.hpp"
#include "DataPackets/RGBDFramePacket/RGBDFramePacket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "VideoCodecs/JPEGVideoCodec/JPEGVideoDecoder.hpp"
#include "VideoCodecs/IVideoDecoder.hpp"
#include "CommunicationUtility/ExpectedJSONConverter.hpp"
#include "Cameras/ProfileJSONConverter.hpp"

namespace crf {
namespace sensors {
namespace cameras {

/**
 * @ingroup group_camera_client
 * @brief Camera client class to communicate with the camera communication point. The
 * purpose is to be able to use a camera client or the real camera independently from the code.
 *
 * @details This class inherits from PriorityAccessClient
 *
 * @{
 */

class CameraClient :
    public crf::utility::devicemanager::PriorityAccessClient,
    public ICamera {
 public:
    CameraClient() = delete;
    /**
     * @brief Construct a new Camera Client object
     *
     * @param socket Socket to communicate with the communication  point
     * @param serverReplyTimeout Timeout for answers
     * @param frequency Frequency for the status streamer
     * @param priority Priority of this client
     */
    explicit CameraClient(std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
                const std::chrono::milliseconds& serverReplyTimeout,
                const float& frequency, const uint32_t& priority);
    CameraClient(const CameraClient&) = delete;
    CameraClient(CameraClient&&) = delete;
    ~CameraClient() override;

    /**
     * @brief Initializes the client and establishes the communcicaation
     *
     * @return true If successful
     * @return false Otherwise
     */
    bool initialize() override;

    /**
     * @brief Deinitializes the client and cuts the communcicaation
     *
     * @return true If successful
     * @return false Otherwise
     */
    bool deinitialize() override;

    /**
     * @brief Takes the latest image from the camera
     *
     * @return cv::Mat The frame of the camera
     */
    cv::Mat captureImage() override;

    /**
     * @brief Set the Profile for the camera
     *
     * @param profile Profile requested (resolution and framerate)
     * @return true If successful
     * @return false Otherwise
     */
    bool setProfile(const Profile& profile) override;

    /**
     * @brief Get the current camera profile
     *
     * @return crf::expected<Profile> current profile
     */
    crf::expected<Profile> getProfile() override;

    /**
     * @brief List all available profiles for the camera
     *
     * @return std::vector<Profile> A list of profiles
     */
    std::vector<Profile> listProfiles() override;

    /**
     * @brief Set the a property in the camera
     *
     * @param property Property to set
     * @param value Desired value
     * @return crf::expected<bool> End result
     */
    crf::expected<bool> setProperty(const Property& property, const int& value) override;

    /**
     * @brief Get the current value of a property
     *
     * @param property Desired property
     * @return crf::expected<int> Expected value
     */
    crf::expected<int> getProperty(const Property& property) override;

 protected:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    std::chrono::milliseconds serverReplyTimeout_;
    uint32_t priority_;

    std::shared_ptr<vision::videocodecs::IVideoDecoder> decoder_;
    crf::expected<Profile> currentProfile_;
    std::vector<Profile> profiles_;
    std::mutex imageMtx_;
    cv::Mat image_;

    Receiver<bool> receiverSetProperty_;
    Receiver<bool> receiverStartFrameStream_;
    Receiver<bool> receiverStopFrameStream_;

    utility::logger::EventLogger logger_;

    /**
     * @brief Struct to pair the property and its respective value
     *
     */
    struct PropertyValue {
        Property property;
        crf::expected<int> value;
        PropertyValue() = default;
        PropertyValue(
            const Property& _property, const int& _value) :
            property(_property), value(_value) {}
    };

    std::map<std::string, PropertyValue> propertyMap_ {
        {"brightness", PropertyValue(Property::BRIGHTNESS, 0)},
        {"contrast", PropertyValue(Property::CONTRAST, 0)},
        {"saturation", PropertyValue(Property::SATURATION, 0)},
        {"hue", PropertyValue(Property::HUE, 0)},
        {"gain", PropertyValue(Property::GAIN, 0)},
        {"exposure", PropertyValue(Property::EXPOSURE, 0)},
        {"focus", PropertyValue(Property::FOCUS, 0)},
        {"focus_mode", PropertyValue(Property::FOCUSMODE, 0)},
        {"shutter", PropertyValue(Property::SHUTTER, 0)},
        {"iso", PropertyValue(Property::ISO, 0)},
        {"zoom", PropertyValue(Property::ZOOM, 0)},
        {"pan", PropertyValue(Property::PAN, 0)},
        {"tilt", PropertyValue(Property::TILT, 0)},
        {"roll", PropertyValue(Property::ROLL, 0)}
    };

    const std::map<Property, std::string> nameMap_ {
        {Property::BRIGHTNESS, "brightness"},
        {Property::CONTRAST, "contrast"},
        {Property::SATURATION, "saturation"},
        {Property::HUE, "hue"},
        {Property::GAIN, "gain"},
        {Property::EXPOSURE, "exposure"},
        {Property::FOCUS, "focus"},
        {Property::FOCUSMODE, "focus_mode"},
        {Property::SHUTTER, "shutter"},
        {Property::ISO, "iso"},
        {Property::ZOOM, "zoom"},
        {Property::PAN, "pan"},
        {Property::TILT, "tilt"},
        {Property::ROLL, "roll"}
    };

    /**
     * @brief Method to start the frame stream
     *
     * @param profile Desired profile
     * @return true If successful
     * @return false Otherwise
     */
    bool startFrameStream(const Profile& profile);

    /**
     * @brief Method to stop the frame stream
     *
     * @return true If successful
     * @return false Otherwise
     */
    bool stopFrameStream();

    /**
     * @brief Method to parse the status received by the communicaiton point
     *
     * @param json JSON object received
     */
    void parseStatus(const nlohmann::json& json) override;

    /**
     * @brief Method to parse the frame received
     *
     * @param buffer Bytes with the frame encoded
     */
    void parseFramePacket(const std::string& buffer);

    /**
     * @brief Utility method to send a packet
     *
     * @param json JSON packet to send
     * @param receiver Receiver to wait for the confirmation that the packet was recieved
     * @return true If successful
     * @return false Otherwise
     */
    bool sendPacket(
        const communication::datapackets::JSONPacket& json,
        const Receiver<bool>& receiver);

    /**
     * @brief Utility method to send a packet with the assigned priority
     *
     * @param json JSON packet to send
     * @param receiver Receiver to wait for the confirmation that the packet was recieved
     * @return true If successful
     * @return false Otherwise
     */
    bool sendPriorityPacket(
        const communication::datapackets::JSONPacket& json,
        const Receiver<bool>& receiver);
};

/**@}*/

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
