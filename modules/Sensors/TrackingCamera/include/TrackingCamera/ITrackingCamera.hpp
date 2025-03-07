#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <array>
#include <vector>

#include <librealsense2/rs.hpp>
#include <boost/optional.hpp>

#include "CommonInterfaces/IInitializable.hpp"

namespace cv {
class Mat;
}  // namespace cv

namespace crf {
namespace sensors {
namespace trackingcamera {

struct TrackingData {
    //  gives acceleration in X, Y, Z (in meters/second^2)
    rs2_vector acceleration;
    //  gives Euler angles - Pitch, Yaw, Roll
    rs2_vector gyro;
    //  gives tracker output (translation (in meters relative to initial position),
    //  velocity, acceleration, angular velocity, angular acceleration)
    rs2_pose pose;
};

class ITrackingCamera: public utility::commoninterfaces::IInitializable {
 public:
    virtual ~ITrackingCamera() = default;
    virtual TrackingData getTrackingData() = 0;
    virtual std::vector<cv::Mat> getVideoFrames() = 0;
    virtual std::vector<cv::Mat> getRawVideoFrames() = 0;
    virtual bool feedBaseVelocity(const std::array<float, 3>& velocity) = 0;
    virtual bool resetPose() = 0;
    virtual boost::optional<rs2_intrinsics> getCameraIntrinsics(bool cameraSelection) = 0;
    virtual boost::optional<rs2_extrinsics> getCameraExtrinsics(bool cameraSelection) = 0;
};

}  // namespace trackingcamera
}  // namespace sensors
}  // namespace crf
