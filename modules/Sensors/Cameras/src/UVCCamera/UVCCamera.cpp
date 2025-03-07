/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author:  Alvaro Garcia Gonzalez BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */


#include "Cameras/UVCCamera/UVCCamera.hpp"

namespace crf {
namespace sensors {
namespace cameras {

UVCCamera::UVCCamera(const std::string& device_name) :
  logger_("UVCCamera"),
  dev_name_(device_name),
  camera_fd_(-1),
  stream_on_(false),
  initialized_(false),
  resolution_(),
  pixel_format_(0),
  buffers_(),
  buffersinfo_(),
  currentBuffer_(0) {
    logger_->debug("CTor");
}

UVCCamera::UVCCamera(const std::string& device_name, const unsigned int& encoding) :
    UVCCamera(device_name) {
        pixel_format_ = encoding;
    }

UVCCamera::~UVCCamera() {
    logger_->debug("DTor");
    deinitialize();
}

bool UVCCamera::initialize() {
    if (initialized_) {
        return true;
    }
    logger_->debug("Status: Initializing");

    // Open the camera
    camera_fd_ = ::open(dev_name_.c_str(), O_RDWR, 0);

    if (camera_fd_ < 0) {
        logger_->error("Error opening the device: {} ", std::strerror(errno));
        return false;
    }

    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));
    if (-1 == ioctl(camera_fd_, VIDIOC_QUERYCAP, &cap)) {
        logger_->error("VIDIOC_QUERYCAP: {}", std::strerror(errno));
        return false;
    }

    logger_->info("Driver: {}", cap.driver);
    logger_->info("Version: {}", std::to_string(cap.version));
    logger_->info("Device: {}", cap.card);
    logger_->info("Location: {}", cap.bus_info);

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        logger_->warn("The device does not handle single-planar video capture.");
        return false;
    }

    auto videoOutputInterface = cap.device_caps & V4L2_CAP_VIDEO_OUTPUT;
    auto m2mInterface = cap.device_caps & V4L2_CAP_VIDEO_M2M;
    auto canRead = cap.device_caps & V4L2_CAP_READWRITE;
    auto canStream = cap.device_caps & V4L2_CAP_STREAMING;

    logger_->info("Capabilities:");
    logger_->info("\tVideo Capture: YES");
    logger_->info("\tRead/write: {}", (canRead ? "YES" : "NO"));
    logger_->info("\tVideo Output: {}", (videoOutputInterface ? "YES" : "NO"));
    logger_->info("\tStreaming: {}", (canStream ? "YES" : "NO"));
    logger_->info("\tMemory-To-Memory: {}", (m2mInterface ? "YES" : "NO"));

    // Choose pixel format
    std::vector<uint32_t> formats;
    struct v4l2_fmtdesc formatDesc = {};
    formatDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while (ioctl(camera_fd_, VIDIOC_ENUM_FMT, &formatDesc) == 0) {
        formatDesc.index++;
        formats.push_back(formatDesc.pixelformat);
    }

    if (std::find(formats.begin(), formats.end(), V4L2_PIX_FMT_MJPEG) != formats.end()) {
        pixel_format_ = V4L2_PIX_FMT_MJPEG;
    } else if (std::find(formats.begin(), formats.end(), V4L2_PIX_FMT_YUYV) != formats.end()) {
        pixel_format_ = V4L2_PIX_FMT_YUYV;
    } else if (std::find(formats.begin(), formats.end(), V4L2_PIX_FMT_GREY) != formats.end()) {
        pixel_format_ = V4L2_PIX_FMT_GREY;
    } else {
        logger_->error("No format supported");
        return false;
    }

    initialized_ = true;
    std::vector<Profile> profiles = listProfiles();
    cv::Size resolution = profiles[0].resolution;
    if (!setProfile(resolution)) {
        initialized_ = false;
        return false;
    }

    if (!start()) {
        logger_->error("Failed to start stream");
        initialized_ = false;
        return false;
    }

    return true;
}

bool UVCCamera::deinitialize() {
    logger_->debug("deinitialize()");

    if (!initialized_)
        return true;

    if (!stop()) {
        logger_->warn("Failed to stop stream");
        return false;
    }

    ::close(camera_fd_);
    initialized_ = false;
    return true;
}

cv::Mat UVCCamera::captureImage() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return cv::Mat();
    }

    auto buf = v4l2_buffer{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

        // Dequeue buffer with new image
    if (-1 == ioctl(camera_fd_, VIDIOC_DQBUF, &buf)) {
        logger_->error("Error dequeueing buffer: {} ({})", strerror(errno), std::to_string(errno));
        return cv::Mat();
    }

    auto const & buffer = buffers_[buf.index];
    auto bytesused = buf.bytesused;

    // Requeue buffer to be reused for new captures
    if (-1 == ioctl(camera_fd_, VIDIOC_QBUF, &buf)) {
        logger_->error("Error re-queueing buffer: {} ({})", strerror(errno), std::to_string(errno));
    }

    cv::Mat res;
    if (pixel_format_== V4L2_PIX_FMT_YUYV) {  // YUYV conversion to RGB
        cv::Mat mat = cv::Mat(resolution_, CV_8UC2, buffer.get());
        cvtColor(mat, res, cv::COLOR_YUV2BGR_YUY2);
    } else if (pixel_format_== V4L2_PIX_FMT_MJPEG) {  // MJPEG to RGB
        std::vector<char> buffer_vec(buffer.get(), buffer.get() + bytesused);
        res = cv::imdecode(buffer_vec, cv::IMREAD_COLOR);
    } else if (pixel_format_ == V4L2_PIX_FMT_GREY) {
        res = cv::Mat(resolution_, CV_8UC1, buffer.get());
    } else {
        logger_->error("Current pixel format is not supported yet");
    }
    return res;
}

bool UVCCamera::setProfile(const cv::Size& resolution) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (stream_on_) {
        if (!stop()) {
            logger_->error("Failed to stop stream ({})", std::strerror(errno));
            return false;
        }
    }
    v4l2_format format;
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = resolution.width;
    format.fmt.pix.height = resolution.height;
    format.fmt.pix.pixelformat = pixel_format_;

    if (ioctl(camera_fd_, VIDIOC_S_FMT, &format) == -1) {
        logger_->error("Setting format failed. ({})", std::strerror(errno));
        if (!start())
            logger_->error("Failed to start stream ({})", std::strerror(errno));
        return false;
    }

    if (!start()) {
        logger_->error("Failed to start stream ({})", std::strerror(errno));
        return false;
    }
    resolution_ = resolution;
    return true;
}

bool UVCCamera::setProfile(const Profile& profile) {
    logger_->debug("setProfile({}, {})", profile.resolution, profile.framerate);
    if (!setProfile(profile.resolution)) {
        return false;
    }
    struct v4l2_streamparm stream_param;
    memset(&stream_param, 0, sizeof(stream_param));
    stream_param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stream_param.parm.capture.timeperframe.numerator = 1;
    stream_param.parm.capture.timeperframe.denominator = profile.framerate;
    if (-1 == ioctl(camera_fd_, VIDIOC_S_PARM, &stream_param))
        logger_->warn("Failed to set framerate");
    return true;
}

crf::expected<Profile> UVCCamera::getProfile() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return crf::Code::NotInitialized;
    }
    v4l2_format format;
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(camera_fd_, VIDIOC_G_FMT, &format) < 0) {
        logger_->error("Failed to get current format");
        return crf::Code::RequestToDeviceFailed;
    }

    cv::Size resolution;
    resolution.width = format.fmt.pix.width;
    resolution.height = format.fmt.pix.height;

    v4l2_streamparm stream_parm;;
    memset(&stream_parm, 0, sizeof(stream_parm));
    stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl(camera_fd_, VIDIOC_G_PARM, &stream_parm)) {
        logger_->error("Failed to get current framerate");
        return crf::Code::RequestToDeviceFailed;
    }

    uint64_t framerate = stream_parm.parm.capture.timeperframe.denominator;
    return Profile(resolution, {framerate});
}

std::vector<Profile> UVCCamera::listProfiles() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::vector<Profile>();
    }
    std::vector<Profile> profiles;
    v4l2_frmsizeenum size_enum;
    memset(&size_enum, 0, sizeof(size_enum));
    size_enum.pixel_format = pixel_format_;
    size_enum.index = 0;
    while (ioctl(camera_fd_, VIDIOC_ENUM_FRAMESIZES, &size_enum) == 0) {
        if (size_enum.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
            v4l2_frmivalenum interval_enum = {0};
            interval_enum.pixel_format = pixel_format_;
            interval_enum.width = size_enum.discrete.width;
            interval_enum.height = size_enum.discrete.height;
            std::vector<uint64_t> framerates;
            while (ioctl(camera_fd_, VIDIOC_ENUM_FRAMEINTERVALS, &interval_enum) == 0) {
                if (interval_enum.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
                    framerates.push_back(interval_enum.discrete.denominator);
                }
                interval_enum.index++;
            }
            profiles.emplace_back(
                cv::Size(size_enum.discrete.width, size_enum.discrete.height), framerates);
        }
        size_enum.index++;
    }
    return profiles;
}

crf::expected<bool> UVCCamera::setProperty(const Property& property, const int& value) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return crf::Code::NotInitialized;
    }
    int32_t v4l2_property;
    if (!getV4L2Property(property, v4l2_property)) {
        logger_->error("Property unsupported by UVC camera: {}", property);
        return crf::Code::NotAcceptable;
    }

    // Query control
    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = v4l2_property;

    if (-1 == ioctl(camera_fd_, VIDIOC_QUERYCTRL, &queryctrl)) {
        if (errno != EINVAL) {
            logger_->error("VIDIOC_QUERYCTRL Error");
            return crf::Code::RequestToDeviceFailed;
        } else {
            logger_->error("Property is not supported by the camera");
            return crf::Code::NotAcceptable;
        }
    }
    if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
        logger_->error("Property is not supported by the camera");
        return crf::Code::NotAcceptable;
    }

    // Set control
    struct v4l2_control control;
    memset(&control, 0, sizeof (control));
    control.id = v4l2_property;
    control.value = value;
    if (-1 == ioctl(camera_fd_, VIDIOC_S_CTRL, &control)) {
        logger_->error("Error setting the value");
        return crf::Code::RequestToDeviceFailed;
    }
    return true;
}

crf::expected<int> UVCCamera::getProperty(const Property& property) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return crf::Code::NotInitialized;
    }
    int32_t v4l2_property;
    if (!getV4L2Property(property, v4l2_property)) {
        logger_->error("Property unsupported by UVC camera: {}", property);
        return crf::Code::NotAcceptable;
    }

    // Query control
    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = v4l2_property;

    if (-1 == ioctl(camera_fd_, VIDIOC_QUERYCTRL, &queryctrl)) {
        if (errno != EINVAL) {
            logger_->error("VIDIOC_QUERYCTRL Error");
            return crf::Code::RequestToDeviceFailed;
        } else {
            logger_->error("Property is not supported by the camera");
            return crf::Code::NotAcceptable;
        }
    }
    if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
        logger_->error("Property is not supported by the camera");
        return crf::Code::NotAcceptable;
    }
    // Get control
    struct v4l2_control control;
    memset(&control, 0, sizeof (control));
    control.id = v4l2_property;
    if (-1 == ioctl(camera_fd_, VIDIOC_G_CTRL, &control)) {
        logger_->error("Error getting the value");
        return crf::Code::RequestToDeviceFailed;
    }
    return control.value;
}

// Private

bool UVCCamera::start() {
    logger_->info("Starting camera stream");
    if (stream_on_) return true;
    if (!prepareBuffers()) {
        for (auto & buffer : buffers_) {
            buffer.reset();
        }
        logger_->error("Could not start the stream. Error preparing the buffers");
        return false;
    }

    // Queue the buffers
    unsigned int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    std::vector<std::unique_ptr<char, crf::sensors::cameras::UVCCamera::mmap_deleter>>::size_type i;  // NOLINT
    for (i = 0; i < buffers_.size(); i++) {
        auto buf = v4l2_buffer{};
        buf.type = type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == ioctl(camera_fd_, VIDIOC_QBUF, &buf)) {
            logger_->info("Buffer failure on capture start: {} ({})",
                strerror(errno), std::to_string(errno));
            return false;
        }
    }

    // Start stream
    if (-1 == ioctl(camera_fd_, VIDIOC_STREAMON, &type)) {
        logger_->info("Failed stream start: {} ({})", strerror(errno), std::to_string(errno));
        return false;
    }
    stream_on_ = true;
    return true;
}

bool UVCCamera::stop() {
    logger_->info("Stopping camera stream");

    unsigned int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl(camera_fd_, VIDIOC_STREAMOFF, &type)) {
        logger_->error("Stream stop failed ({}).", strerror(errno));
        return false;
    }

    // Release the buffers (count = 0)
    struct v4l2_requestbuffers reqbuf;
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    reqbuf.count = 0;
    if (-1 == ioctl(camera_fd_, VIDIOC_REQBUFS, &reqbuf)) {
        logger_->error("Buffer release failed ({}).", strerror(errno));
        return false;
}

    for (auto & buffer : buffers_) {
        buffer.reset();
    }

    stream_on_ = false;

    return true;
}

bool UVCCamera::prepareBuffers() {
    logger_->debug("prepareBuffers()");

    v4l2_requestbuffers reqbuf;
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    reqbuf.count = DEFAULT_V4L2_BUFFERS;

    if (ioctl(camera_fd_, VIDIOC_REQBUFS, &reqbuf) == -1) {
        if (errno == EINVAL)
            logger_->error("Video capturing or mmap-streaming is not supported.");
        else
            logger_->error("Requesting buffers error: ({}).", strerror(errno));
        return false;
    }

    if (reqbuf.count < DEFAULT_V4L2_BUFFERS) {
        logger_->error("Requesting buffers error: Not enought buffer memory.");
        return false;
    }

    for (uint i = 0; i < reqbuf.count; i++) {
        struct v4l2_buffer buffer;
        memset(&buffer, 0, sizeof(buffer));
        buffer.type = reqbuf.type;
        buffer.memory = reqbuf.memory;
        buffer.index = i;

        if (ioctl(camera_fd_, VIDIOC_QUERYBUF, &buffer) == -1) {
            logger_->error("Error querying buffers. Exiting");
            return false;
        }

        buffers_[i] = std::unique_ptr<char, mmap_deleter>(
            static_cast<char *>(
                mmap(
                NULL,
                buffer.length,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                camera_fd_,
                buffer.m.offset)),
            mmap_deleter(buffer.length));

        if (MAP_FAILED == &buffers_[i]) {
            logger_->error("Error memory mapping. Exiting");
            return false;
        }
    }
    return true;
}

bool UVCCamera::getV4L2Property(const Property& property, int32_t& v4l2_property) {
    switch (property) {
        case Property::BRIGHTNESS:
            v4l2_property = V4L2_CID_BRIGHTNESS;
            return true;
        case Property::CONTRAST:
            v4l2_property = V4L2_CID_CONTRAST;
            return true;
        case Property::SATURATION:
            v4l2_property = V4L2_CID_SATURATION;
            return true;
        case Property::HUE:
            v4l2_property = V4L2_CID_HUE;
            return true;
        case Property::EXPOSURE:
            v4l2_property = V4L2_CID_EXPOSURE_ABSOLUTE;  // TODO(any): Have a look autoexposure
            return true;
        case Property::GAIN:
            v4l2_property = V4L2_CID_GAIN;
            return true;
        case Property::FOCUS:
            v4l2_property = V4L2_CID_FOCUS_ABSOLUTE;
            return true;
        case Property::FOCUSMODE:
            v4l2_property = V4L2_CID_FOCUS_AUTO;
            return true;
        case Property::ZOOM:
            v4l2_property = V4L2_CID_ZOOM_ABSOLUTE;
            return true;
        case Property::PAN:
            v4l2_property = V4L2_CID_PAN_ABSOLUTE;
            return true;
        case Property::TILT:
            v4l2_property = V4L2_CID_TILT_ABSOLUTE;
            return true;
        default:
            return false;
    }
}

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
