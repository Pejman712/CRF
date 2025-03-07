/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author:  Alvaro Garcia Gonzalez BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <mutex>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#include "Cameras/ICamera.hpp"
#include "EventLogger/EventLogger.hpp"
#include "crf/expected.hpp"

#define DEFAULT_V4L2_BUFFERS 4

namespace crf {
namespace sensors {
namespace cameras {

/**
 * @ingroup group_uvc_cameras
 * @brief Struct to represent a profile. It is composed of
 * a resolution and a framerate. Several framerates can be
 * added if the resolution allows it
 *
 * @{
 */

class UVCCamera : public ICamera {
 public:
    explicit UVCCamera(const std::string& device_name);
    UVCCamera(const std::string& device_name, const uint32_t& encoding);
    ~UVCCamera();

    bool initialize() override;
    bool deinitialize() override;

    cv::Mat captureImage() override;

    bool setProfile(const cv::Size& resolution);
    bool setProfile(const Profile& profile) override;
    crf::expected<Profile> getProfile() override;
    std::vector<Profile> listProfiles() override;

    crf::expected<bool> setProperty(const Property& property, const int& value) override;
    crf::expected<int> getProperty(const Property& property) override;

 private:
    class mmap_deleter {
        std::size_t m_size;
     public:
        mmap_deleter() : m_size(0) {}
        explicit mmap_deleter(std::size_t size) : m_size{size} {}
        void operator()(void *ptr) const {
            munmap(ptr, m_size);
        }
    };

    utility::logger::EventLogger logger_;
    std::string dev_name_;
    int camera_fd_;
    bool stream_on_;
    bool initialized_;
    cv::Size resolution_;
    unsigned int pixel_format_;

    std::array<std::unique_ptr<char, mmap_deleter>, DEFAULT_V4L2_BUFFERS> buffers_;
    std::vector<struct v4l2_buffer> buffersinfo_;
    int currentBuffer_;

    bool start();
    bool stop();
    bool prepareBuffers();
    bool getV4L2Property(const Property& property, int32_t& v4l2_property);  // NOLINT
};

/**@}*/

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
