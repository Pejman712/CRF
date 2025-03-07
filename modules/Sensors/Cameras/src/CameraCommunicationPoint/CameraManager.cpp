/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO
 *         Alvaro Garcia Gonzalez BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

#include <opencv2/core.hpp>

#include "Cameras/CameraCommunicationPoint/CameraManager.hpp"

namespace crf::sensors::cameras {

CameraManager::CameraManager(std::shared_ptr<ICamera> camera,
    const std::chrono::milliseconds& initialization_timeout,
    const std::chrono::milliseconds& controlAccessTimeout) :
    DeviceManagerWithPriorityAccess(camera, initialization_timeout, controlAccessTimeout),
    currentProfile_(cv::Size(0, 0), 0),
    stopGrabber_(true),
    sleepGrabber_(false),
    camera_(camera),
    logger_("CameraManager") {
    logger_->debug("CTor");

    if (!initializeDevice()) {
        throw std::runtime_error("Device initialization failed");
    }

    cameraProfiles_ = camera_->listProfiles();
    sleepGrabber_ = true;
}

CameraManager::~CameraManager() {
    logger_->debug("DTor");
    stopFrameGrabber();
}

cv::Mat CameraManager::getFrame(int stream_id, const std::chrono::milliseconds& timeout) {
    if (stopGrabber_) {
        logger_->error("getFrame(): Frame grabber is not running!");
        return cv::Mat();
    }
    cv::Size resolution;
    cv::Mat frame;

    {
        // Requires shared ownership to read the map
        std::shared_lock<std::shared_timed_mutex> lock(requestedStreamsMtx_);
        resolution = requestedStreams_.at(stream_id).resolution;  // "at" will throw out of range
    }

    {  // Requires shared ownership to read the last frame
        std::shared_lock<std::shared_timed_mutex> lock(frameMtx_);
        frame = latestFrame_;
    }
    if (frame.empty()) return frame;
    cv::resize(frame, frame, resolution);
    return frame;
}

nlohmann::json CameraManager::getStatus() {
    nlohmann::json statusJSON;

    if (initializeDevice()) {
        statusJSON["status"] = "initialize";
    } else {
        statusJSON["status"] = "deinitialize";
        return statusJSON;
    }

    statusJSON["available_profiles"] = cameraProfiles_;
    std::scoped_lock<std::mutex> lock(cameraMtx_);
    for (auto& pair : propertyMap_) {
        statusJSON["property"][pair.first] = camera_->getProperty(pair.second);
    }
    statusJSON["current_profile"] = camera_->getProfile();
    return statusJSON;
}

bool CameraManager::setProperty(const uint32_t &priority, const nlohmann::json& properties) {
    logger_->debug("setProperty()");
    if (!checkCommandPriority(priority)) return false;
    try {
        for (auto& command : properties.items()) {
            crf::expected<bool> res =
                camera_->setProperty(propertyMap_.at(command.key()), command.value().get<int>());
            if (!res) {
                logger_->error("setProperty(): Could not set property \"{}\"", command.key());
                return false;
            }
        }
    } catch (const nlohmann::json::type_error& e) {
        logger_->error("setProperty(): Error parsing properties: {}", e.what());
        return false;
    } catch (const std::exception& e) {
        logger_->error("setProperty(): Not contemplated exception: {}", e.what());
        return false;
    }
    return true;
}

bool CameraManager::setProfile(int stream_id, const Profile& profile) {
    logger_->debug("setProfile({}, {}, {}", stream_id, profile.resolution, profile.framerate);
    // Requires unique ownership to write into the map
    std::unique_lock<std::shared_timed_mutex> lock(requestedStreamsMtx_);
    requestedStreams_[stream_id] = StreamRequest(profile.resolution, profile.framerate);
    updateProfile();
    return true;
}

bool CameraManager::requestFrameStream(int stream_id, const Profile& profile) {
    logger_->debug("requestFrameStream({}, {}x{})",
        stream_id, profile.resolution, profile.framerate);
    auto it = std::find_if(cameraProfiles_.begin(), cameraProfiles_.end(),
        [profile](Profile p){
            return p.resolution == profile.resolution;
        });
    if (it == cameraProfiles_.end()) {
        logger_->error("requestFrameStream(): Resolution not valid");
        return false;
    }
    uint64_t framerate = profile.framerate;
    if (std::find(it->framerates.begin(), it->framerates.end(), profile.framerate) ==
        it->framerates.end()) {
        uint64_t maxFramerate = *std::max_element(it->framerates.begin(), it->framerates.end());
        logger_->error("requestFrameStream(): Framerate ({}) not valid for the resolution, using "
            "the max ({})", profile.framerate, maxFramerate);
        framerate = maxFramerate;
    }

    // Requires unique ownership to write into the map
    std::unique_lock<std::shared_timed_mutex> lock(requestedStreamsMtx_);
    requestedStreams_[stream_id] = StreamRequest(profile.resolution, framerate);
    updateProfile();
    sleepGrabber_ = false;
    startFrameGrabber();
    return true;
}

void CameraManager::removeFrameStream(int stream_id) {
    logger_->debug("removeFrameStream()");

    // Requires unique ownership to erase
    std::unique_lock<std::shared_timed_mutex> lock(requestedStreamsMtx_);
    requestedStreams_.erase(stream_id);
    if (requestedStreams_.size() == 0) {
        logger_->info("removeFrameStream(): No more streams active, sleeping grabber");
        sleepGrabber_ = true;
        return;
    }
    updateProfile();
}

// Protected

void CameraManager::startFrameGrabber() {
    logger_->debug("startFrameGrabber()");
    if (frameGrabberThread_.joinable()) return;
    stopGrabber_ = false;
    frameGrabberThread_ = std::thread(&CameraManager::frameGrabber, this);
    logger_->info("startFrameGrabber(): Starting frame grabber thread");
}

void CameraManager::stopFrameGrabber() {
    logger_->debug("startFrameGrabber(): stopFrameGrabber");
    if (!frameGrabberThread_.joinable()) return;
    stopGrabber_ = true;
    frameGrabberThread_.join();
    logger_->info("startFrameGrabber(): Stopping frame grabber thread");
}

void CameraManager::frameGrabber() {
    logger_->debug("frameGrabber()");
    cv::Mat frame;
    while (!stopGrabber_) {
        if (sleepGrabber_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        /**
         * Even thought he cameraMtx_ should be explicitly unlocked,
         * the compiler optimizes the code to the point where the mutex barely gets
         * unlocked. This creates timeouts and long waits to access the camera
         * Adding a small delay of 1 ms ensures that the mutex gets unlocked, there
         * are probably better ways of doing this.
         * (jplayang)
         *
         */
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if (!initializeDevice()) {
            logger_->error("frameGrabber(): Cannot initialize the camera");
            return;
        }

        std::unique_lock<std::mutex> lock(cameraMtx_);
        frame = camera_->captureImage();
        lock.unlock();

        if (frame.empty()) {
            logger_->warn("frameGrabber(): Empty frame");
            continue;
        }

        // Requires unique ownership to write the last frame
        std::scoped_lock<std::shared_timed_mutex> lockfr(frameMtx_);
        latestFrame_ = frame;
    }
    logger_->debug("frameGrabber(): Exiting frame grabber");
}

bool CameraManager::updateProfile() {
    logger_->debug("updateProfile()");
    cv::Size reqResolution = getMaxRequestedResolution();
    uint64_t reqFramerate = getMaxRequestedFramerate();

    if ((currentProfile_.resolution == reqResolution) &&
        (currentProfile_.framerate == reqFramerate))
        return true;

    // Search the requested resolution
    auto it = std::find_if(cameraProfiles_.begin(), cameraProfiles_.end(),
        [reqResolution](Profile p){
            return p.resolution == reqResolution;
        });

    // For that resolution try to get that framerate, otherwise use the maximum
    if(std::find(it->framerates.begin(), it->framerates.end(), reqFramerate) == it->framerates.end()) {  // NOLINT
        int maxFramerate = *std::max_element(it->framerates.begin(), it->framerates.end());
        logger_->warn(
            "updateProfile(): Requested framerate ({}) not available, using the maximum one ({})",
            reqFramerate, maxFramerate);
        reqFramerate = maxFramerate;
    }

    std::scoped_lock<std::mutex> lockCam(cameraMtx_);
    if (!camera_->setProfile(Profile(reqResolution, reqFramerate))) {
        logger_->warn("updateProfile(): Set resolution and framerate failed");
        return false;
    }
    currentProfile_.resolution = reqResolution;
    currentProfile_.framerate = reqFramerate;
    return true;
}

cv::Size CameraManager::getMaxRequestedResolution() {
    cv::Size maxResolution(0, 0);
    for (auto& stream : requestedStreams_) {
        if ((stream.second.resolution.width * stream.second.resolution.height) >
            (maxResolution.width * maxResolution.height)) {
            maxResolution = stream.second.resolution;
        }
    }
    return maxResolution;
}

uint64_t CameraManager::getMaxRequestedFramerate() {
    uint64_t maxFramerate = 0;
    for (auto& stream : requestedStreams_) {
        if (stream.second.framerate > maxFramerate) maxFramerate = stream.second.framerate;
    }
    return maxFramerate;
}

}   // namespace crf::sensors::cameras
