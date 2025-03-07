#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <cstdint>
#include <memory>
#include <vector>

#include "EventLogger/EventLogger.hpp"
// #include "ScrewDetection/TrackingArea.hpp"
// #include "IPC/IPC.hpp"
// #include "SerialCommunication/SerialCommunication.hpp"

#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"    // for cvtColor and canny
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
// #include "opencv2/xfeatures2d.hpp"
// #include "opencv2/features2d.hpp"

#define THRESH     200

namespace crf {
namespace vision {
namespace depthEstimation {

class ScrewDetection {
 public:
    ScrewDetection():logger_("Scre Detection") { this->logger_->debug("constructor");}
    std::vector<cv::Vec3f> searchTheScrews();
    std::vector<cv::Vec3f> findCircles();
    /** 
    * @brief It draws circles in the original frame
    * @param[in] circles is an array with all circles found in frame
    **/
    void drawCircles(cv::Mat &frhame, const std::vector<cv::Vec3f> &circles);  // NOLINT
    /**
     * @brief looking for the biggest one
     * @param[in] vector for circles
     */
    int biggestScrew(std::vector<cv::Vec3f> &circles);  // NOLINT

    void setFrame(const cv::Mat &frame) { this->frame_ = frame.clone(); }
    cv::Mat getFrame() const { return this->frame_; }

 private:
    cv::Mat frame_, cannyFrame_;
    cv::Point center_;
    crf::utility::logger::EventLogger logger_;

    /**
    * @brief Applying CANNY algorithm to get the edges map from the image
    **/
    void edgesCanny() { cv::Canny(this->frame_, this->cannyFrame_, THRESH / 1.25, THRESH * 1.25); }
    /** 
    * @brief It draws circles in the original frame
    * @param[in] circles is an array with all circles found in frame
    **/
    void drawCircles(const std::vector<cv::Vec3f> &circles);
};

}  // namespace depthEstimation
}  // namespace vision
}  // namespace crf
