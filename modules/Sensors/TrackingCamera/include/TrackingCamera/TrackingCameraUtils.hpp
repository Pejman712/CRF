#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "opencv2/core.hpp"
#include "librealsense2/rs.hpp"

namespace crf {
namespace sensors {
namespace trackingcamera {

class TrackingCameraUtils {
 public:
    static cv::Mat frame_to_mat(const rs2::frame& f);
};

}  // namespace trackingcamera
}  // namespace sensors
}  // namespace crf

