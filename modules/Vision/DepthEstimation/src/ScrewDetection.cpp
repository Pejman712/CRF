
/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include "DepthEstimation/ScrewDetection.hpp"

#define THRESH     200

namespace crf {
namespace vision {
namespace depthEstimation {

std::vector<cv::Vec3f> ScrewDetection::searchTheScrews() {
    std::vector<cv::Vec3f> circles;

    this->logger_->debug("Looking for screws...");
    return this->findCircles();

    // this->wantToFollowIt = ( this->negativeFrames > 10) ? true : false;

    /** thread to publish data */
    /*std::thread t(&SelectArea::publish_on_the_GUI, this);
    t.join();*/

    /*Matrix<float> auxMatrix;
    try { auxMatrix = updateTracker(); }
    catch (const std::invalid_argument& e){sleep(2); throw e;}*/

    // return circles;
}

std::vector<cv::Vec3f> ScrewDetection::findCircles() {
    cv::Mat grayHalfFrame;
    cv::cvtColor(this->frame_, grayHalfFrame, CV_BGR2GRAY);

    std::vector<cv::Vec3f> circles;
    try {  // PARAMETERS: (1, 12, 70, 16, 2, 16)
        /** This method use internal Canny */
        cv::HoughCircles(grayHalfFrame, circles, CV_HOUGH_GRADIENT,
            /*The inverse ratio of resolution*/             1,
            /*Minimum distance between detected centers*/   grayHalfFrame.rows / 12,
            /*Upper threshold for Canny*/                   THRESH * 1.25,
            /*Threshold for center detection*/              16,
            /*Minimum radio*/                               8,
            /*Maximum radio*/                               17);

        this->drawCircles(circles);
    } catch(cv::Exception e) { this->logger_->error(e.what()); }

    return circles;
}

int ScrewDetection::biggestScrew(std::vector<cv::Vec3f> &circles) {
    int index = -1;
    if (circles.size() > 1) {
        double maxArea = -1;
        for (int i = 0; i < circles.size(); i++) {
            if (maxArea < (M_PI * pow(cvRound(circles[i][2]), 2))) {
                maxArea = M_PI * pow(cvRound(circles[i][2]), 2);
                index = i;
            }
        }
    }
    return index;
}

void ScrewDetection::drawCircles(cv::Mat &frame, const std::vector<cv::Vec3f> &circles) {
    for (const auto& crcl : circles) {
        cv::Point centerP(crcl[0], crcl[1]);
        // circle center
        cv::circle(frame, centerP, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // circle outline
        cv::circle(frame, centerP, crcl[2]/*radius*/, cv::Scalar(0, 0, 255), 3, 8, 0);
    }
}

void ScrewDetection::drawCircles(const std::vector<cv::Vec3f> &circles) {
    for (size_t i = 0; i < circles.size(); i++) {
        this->center_.x = cvRound(circles[i][0]);
        this->center_.y = cvRound(circles[i][1]);
        // circle center
        cv::circle(this->frame_, this->center_, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // circle outline
        cv::circle(this->frame_, this->center_, cvRound(circles[i][2]), cv::Scalar(0, 0, 255),
            3, 8, 0);
    }
}

}  // namespace depthEstimation
}  // namespace vision
}  // namespace crf
