/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <DepthEstimation/TrackingArea.hpp>

namespace crf {
namespace vision {
namespace depthEstimation {

TrackingArea::TrackingArea(const cv::Mat &frame): logger_("TrackingArea"), roiIndex_(-1) {
    this->setFrame(frame);
    this->logger_->debug("CTor(arg)");
}

inline void TrackingArea::correctPositionPoints(cv::Point &ptLeftUp, cv::Point &ptRightDown) {
    if (ptLeftUp.x < 0 || ptLeftUp.y < 0) {
        if (ptLeftUp.x < 0) {
            ptRightDown.x -= ptLeftUp.x;       // - * - == +
            ptLeftUp.x = 0;
        }
        if (ptLeftUp.y < 0) {
            ptRightDown.y -= ptLeftUp.y;
            ptLeftUp.y = 0;
        }
    }
    if (ptRightDown.x > this->frame_.cols || ptRightDown.y > this->frame_.rows) {
        if (ptRightDown.x > this->frame_.cols) {
            ptLeftUp.x -= (abs(ptRightDown.x - this->frame_.cols));
            ptRightDown.x = this->frame_.cols;
        }
        if (ptRightDown.y > this->frame_.rows) {
            ptLeftUp.y -= (abs(ptRightDown.y - this->frame_.rows));
            ptRightDown.y = this->frame_.rows;
        }
    }
}
void TrackingArea::updateTracker(cv::Mat &frame) {
    for (uint16_t i = 0; i < this->tracker_.size(); i++) {
        this->tracker_[i]->update(frame, this->roi_[i]);
        cv::rectangle(frame, this->roi_[i], this->roiIndex_ == i ? cv::Scalar(0, 0, 255) :
            cv::Scalar(255, 0, 0), 2, 1);
    } this->logger_->debug("tracking updated");
}

void TrackingArea::createTracker(const cv::Point &ptLeftUp, const cv::Point &ptRightDown) {
    this->tracker_.push_back(cv::TrackerKCF::create());
    this->roi_.push_back(cv::Rect_<double> (ptLeftUp.x, ptLeftUp.y,
        abs(ptLeftUp.x - ptRightDown.x), abs(ptLeftUp.y - ptRightDown.y)));
    this->tracker_.back()->init(this->frame_, this->roi_.back());
}

void TrackingArea::createRoiFromCenter(const cv::Point center) {
    auto ptLeftUp      = cv::Point(center.x - this->widthMarc_, center.y - this->heightMarc_);
    auto ptRightDown   = cv::Point(center.x + this->widthMarc_, center.y + this->heightMarc_);

    correctPositionPoints(ptLeftUp, ptRightDown);
    try {
        createTracker(ptLeftUp, ptRightDown);
    } catch(cv::Exception e) { this->logger_->error(e.what()); }
}

int TrackingArea::scrollsVector(const cv::Point &center) {
    for (int i = 0; i < this->roi_.size(); i++)
        if (center.x >= this->roi_[i].x && center.x <= (this->roi_[i].x + this->roi_[i].width))
            if (center.y >= this->roi_[i].y && center.y <= (this->roi_[i].y + this->roi_[i].height))
                return i;
    return -1;
}

void TrackingArea::upgradeTracker(const cv::Point &center) {
    this->roiIndex_ = this->scrollsVector(center);
    if (this->roiIndex_ == -1) this->createRoiFromCenter(center);
}

void TrackingArea::deleteTracker(const cv::Point &center) {
    auto indexToDelete = this->scrollsVector(center);
    if (indexToDelete != -1) {
        this->logger_->warn("You are removing the screw");
        this->roi_.erase(this->roi_.begin() + indexToDelete);
        this->tracker_.erase(this->tracker_.begin() + indexToDelete);
        this->roiIndex_ = -1;
        // when we remove a tracker, the index could change, now this issue disappears
    }
}

void TrackingArea::drawRois(cv::Mat& frame) {
    for (const auto& roi : this->roi_)
        cv::rectangle(frame, roi, cv::Scalar(255, 0, 0), 2, 1);
}

void TrackingArea::setFromScratch(const cv::Mat &frame, const int numRois,
    std::vector<cv::Vec3f> &circles, int radius) {
    this->frame_ = frame.clone();
    this->widthMarc_ = this->heightMarc_ = radius;

    if (!this->tracker_.empty()) this->~TrackingArea();

    for (const auto& centerPoint : circles) this->createRoiFromCenter(
        cv::Point(centerPoint[0], centerPoint[1]));
}

}  // namespace depthEstimation
}  // namespace vision
}  // namespace crf
