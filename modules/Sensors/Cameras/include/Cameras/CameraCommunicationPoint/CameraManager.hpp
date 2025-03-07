/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alvaro Garcia Gonzalez BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <condition_variable>
#include <functional>
#include <map>
#include <vector>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <utility>

#include <nlohmann/json.hpp>

#include "Cameras/ICamera.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "CommunicationUtility/ExpectedJSONConverter.hpp"
#include "Cameras/ProfileJSONConverter.hpp"

namespace crf::sensors::cameras {

/**
 * @ingroup group_camera_communication_point
 * @brief Camera manager class for camera communication point. The camera manager class ensures
 *        thread safe access to the camera. It is also responsible to optimize the resources. If no
 *        communication point is requesting frames the camera is deinitialized to release resources
 *        on the running controller (e.g. USB bandwidth). The communication point requests a frame
 *        stream to the camera manager with a certain resolution. It is also responsible of
 *        changing the camera parameters.
 *
 * @{
 */
class CameraManager : public utility::devicemanager::DeviceManagerWithPriorityAccess {
 public:
    CameraManager() = delete;
    /**
     * @brief Construct a new Camera Manager object
     *
     * @param camera the pointer to the ICamera object
     * @param initializationTimeout Timeout until the camera deinitializes if not used
     * @param controlAccessTimeout Timeout until access is released if not used
     */
    CameraManager(
        std::shared_ptr<ICamera> camera,
        const std::chrono::milliseconds& initializationTimeout = std::chrono::seconds(20),
        const std::chrono::milliseconds& controlAccessTimeout = std::chrono::seconds(10));
    CameraManager(const CameraManager& other) = delete;
    CameraManager(CameraManager&& other) = delete;

    /**
     * @brief Destroy the Camera Manager object. It deinitializes the camera.
     */
    virtual ~CameraManager();

    /**
     * @brief Get the current camera parameters. It includes the resolution
     *
     * @return nlohmann::json the current camera parameters
     */
    nlohmann::json getStatus() override;

    /**
     * @brief Set camera parameters such as focus, zoom etc. The resolution will not be affected
     *
     * @param parameters the parameters to change
     */
    bool setProperty(const uint32_t &priority, const nlohmann::json& parameters);

    /**
     * @brief Set camera parameters such as focus, zoom etc. The resolution will not be affected
     *
     * @param parameters the parameters to change
     */
    bool setProfile(int stream_id, const Profile& profile);

    /**
     * @brief Requests a stream of frames with a specific resolution. The streamID is the
     *        identifier of the communication point. If the streamID has already an active stream
     *        the new resolution is updated.
     *
     * @param stream_id the id of the communication point
     * @param resolution the desired resolution
     * @param famerate the desired framerate
     * @return true if the stream correctly started
     * @return false if the camera stream did not start
     */
    virtual bool requestFrameStream(int stream_id, const Profile& profile);

    /**
     * @brief Stops a stream. If there are no streams left, deinitialize camera.
     *
     * @param stream_id the id of the communication point
     */
    virtual void removeFrameStream(int stream_id);

    /**
     * @brief Get the latest camera frame.
     *
     * @param stream_id the id of the communication point
     * @param timeout timeout in milliseconds after which the function return an empty frame if
     *                a new frame is not received
     * @return cv::Mat the frame.
     */
    virtual cv::Mat getFrame(int stream_id,
        const std::chrono::milliseconds& timeout = std::chrono::milliseconds(1000));

 protected:
    /**
     * @brief Method to stop the frame grabber
     *
     */
    virtual void stopFrameGrabber();

    /**
     * @brief Method to start the frame grabber
     *
     */
    virtual void startFrameGrabber();

    /**
     * @brief Struct to store the values requested by a client
     *
     */
    struct StreamRequest {
        StreamRequest() = default;
        /**
         * @brief Construct a new Stream Request object
         *
         * @param res Requested resolution
         * @param fr Requested framerate
         */
        explicit StreamRequest(const cv::Size& res, uint64_t fr) :
            resolution(res),
            framerate(fr) {}
        cv::Size resolution;
        uint64_t framerate;
    };

    const std::map<std::string, Property> propertyMap_ {
        {"brightness", Property::BRIGHTNESS},
        {"contrast", Property::CONTRAST},
        {"saturation", Property::SATURATION},
        {"hue", Property::HUE},
        {"gain", Property::GAIN},
        {"exposure", Property::EXPOSURE},
        {"focus", Property::FOCUS},
        {"focus_mode", Property::FOCUSMODE},
        {"shutter", Property::SHUTTER},
        {"iso", Property::ISO},
        {"zoom", Property::ZOOM},
        {"pan", Property::PAN},
        {"tilt", Property::TILT},
        {"roll", Property::ROLL}
    };

    std::vector<crf::sensors::cameras::Profile> cameraProfiles_;

    Profile currentProfile_;

    std::map<int, StreamRequest> requestedStreams_;
    std::shared_timed_mutex requestedStreamsMtx_;

    cv::Mat latestFrame_;
    std::shared_timed_mutex frameMtx_;

    std::mutex cameraMtx_;

    // Frame Grabber
    std::thread frameGrabberThread_;
    std::atomic<bool> stopGrabber_;
    std::atomic<bool> sleepGrabber_;

    /**
     * @brief This function is the thread that grabs frames from the camera.
     *        It also controlls resolution and framerate.
     *        It finishes when there are no more streams.
     */
    virtual void frameGrabber();

    /**
     * @brief Returns the maximum requested resolution.
     *
     * @return maximum requested resolution.
     */
    cv::Size getMaxRequestedResolution();

    /**
     * @brief Returns the maximum requested framerate.
     *
     * @return maximum requested framerate.
     */
    uint64_t getMaxRequestedFramerate();

    /**
     * @brief Compares the current resolution with the maximum requested resolution.
     *        If they are different, change the resolution to the max requested.
     * @return if there was an error during the update
     */
    virtual bool updateProfile();

 private:
    std::shared_ptr<ICamera> camera_;
    crf::utility::logger::EventLogger logger_;
};

/**@}*/

}  // namespace crf::sensors::cameras
