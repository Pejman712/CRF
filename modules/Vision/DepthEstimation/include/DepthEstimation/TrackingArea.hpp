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
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>              // for cvtColor and canny
#include <opencv2/tracking.hpp>             // for tracker
#include <opencv2/tracking/tracker.hpp>

#include <IPC/IPC.hpp>
#include <Logger/Logger.hpp>
#include <CommUtility/CommunicationPacket.hpp>

namespace crf {
namespace vision {  // I don't want to use "namespace TrackingArea" because it is a c++ keyword
namespace depthEstimation {

class TrackingArea {
 public:
    TrackingArea(): logger_("TrackingArea"), roiIndex_(-1) {
        this->logger_->debug("Tracking constructor");
    }
    explicit TrackingArea(const cv::Mat &frame);
    void setFrame(const cv::Mat &frame) {
        this->frame_ = frame.clone();
    }
    void setFromScratch(const cv::Mat &frame, const int numRois, std::vector<cv::Vec3f> &circles,  // NOLINT
        int radius);
    void drawRois(cv::Mat& frame);  // NOLINT
    void upgradeTracker(const cv::Point &center);
    void deleteTracker(const cv::Point &center);
    /**
     * @brief      To update the trackers of each frame
     * @param[in]  The current frame of the system
     */
    void updateTracker(cv::Mat &frame);  // NOLINT
    ~TrackingArea() {
        this->tracker_.clear();
        this->roi_.clear();
        this->logger_->debug("memory deleted");
    }

 private:
    cv::Mat frame_;
    /** To choose in red the specific Roi, if it is -1 means there is no roi chosen*/
    int roiIndex_;
    int widthMarc_, heightMarc_;
    utility::logger::EventLogger logger_;
    /** vector of Trackers used to track each screw */
    std::vector<cv::Ptr<cv::Tracker>> tracker_;
    /** vector of Rois used to screw enumeration and tracking */
    std::vector<cv::Rect_<double>> roi_;

    int scrollsVector(const cv::Point &center);

    /**
     * @brief      This function takes care the whole interest area be on the scene.
     * @param      ptLeftUp     The point left up
     * @param      ptRightDown  The point right down
     */
    inline void correctPositionPoints(cv::Point &ptLeftUp, cv::Point &ptRightDown);  // NOLINT

    /**
     * @brief      Call a calculateRoi from the origin position, set the choosedArea on true and set the origin position on myHomography.
     * @param      center     Area of interest's center point
     * @param      centerRoi  Area of interest's ROI
     */
    void createRoiFromCenter(const cv::Point center);   //, cv::Rect_<double> &centerRoi);

    /**
     * @brief      Calculates the main roi, create the Tracker and start it.
     * @param      ptLeftUp     The point left up
     * @param      ptRightDown  The point right down
     */
    void createTracker(const cv::Point &ptLeftUp, const cv::Point &ptRightDown);
};

}  // namespace depthEstimation
}  // namespace vision
}  // namespace crf
