/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

#include "TrackingCamera/TrackingCameraUtils.hpp"

namespace crf {
namespace sensors {
namespace trackingcamera {

cv::Mat TrackingCameraUtils::frame_to_mat(const rs2::frame& f) {
    auto vf = f.as<rs2::video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8) {
        return  cv::Mat(cv::Size(w, h),  CV_8UC3, (void*)f.get_data(),  cv::Mat::AUTO_STEP);  // NOLINT
    }
    if (f.get_profile().format() == RS2_FORMAT_RGB8) {
        auto r =  cv::Mat(cv::Size(w, h),  CV_8UC3, (void*)f.get_data(),  cv::Mat::AUTO_STEP);  // NOLINT
         cvtColor(r, r, cv::COLOR_RGB2BGR);
        return r;
    }
    if (f.get_profile().format() == RS2_FORMAT_Z16) {
        return  cv::Mat(cv::Size(w, h),  CV_16UC1, (void*)f.get_data(),  cv::Mat::AUTO_STEP);  // NOLINT
    }
    if (f.get_profile().format() == RS2_FORMAT_Y8) {
        return  cv::Mat(cv::Size(w, h),  CV_8UC1, (void*)f.get_data(),  cv::Mat::AUTO_STEP);  // NOLINT
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

}   // namespace trackingcamera
}   // namespace sensors
}   // namespace cern
