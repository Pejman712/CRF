/* Â© Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>

#include "CommUtility/StreamReader.hpp"
#include "CommUtility/StreamWriter.hpp"
#include "PersonFollower/PersonDetector.hpp"
#include "VisionTypes/VisionTypes.hpp"


namespace crf {
namespace applications {
namespace personfollower {

PersonDetector::PersonDetector(std::shared_ptr<crf::sensors::cameras::ICamera> camera,
    std::shared_ptr<crf::vision::objectDetection::IObjectDetector> detector):
    camera_(camera),
    detector_(detector),
    initialized_(false),
    logger_("PersonDetector") {
    logger_->debug("CTor");
}

PersonDetector::~PersonDetector() {
    logger_->debug("DTor");
    deinitialize();
}

bool PersonDetector::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->error("PersonDetector already initialized");
        return false;
    }
    if (!detector_->initialize()) {
        logger_->error("Client could not be opened");
        return false;
    }
    if (!camera_->initialize()) {
        logger_->error("Camera could not be opened");
        return false;
    }
    initialized_ = true;
    return true;
}

bool PersonDetector::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("PersonDetector is not running");
        return false;
    }
    if (!camera_->deinitialize()) {
        logger_->warn("The camera didn't deinitialize correctly");
        return false;
    }
    if (!detector_->deinitialize()) {
        logger_->warn("The requester didn't close correctly");
        return false;
    }
    logger_->debug("PersonDetector stopped");
    initialized_ = false;
    return true;
}

cv::Rect2d PersonDetector::getPersonBoundingBox() {
    logger_->debug("getPersonBoundingBox");
    cv::Rect2d roi;
    cv::Mat colorFrame(camera_->getFrame());
    roi = requestPersonBoundingBoxFromServer(colorFrame);
    return roi;
}

int PersonDetector::getImageCenterWidth() {
    logger_->debug("getImageCenterWidth");
    cv::Mat colorFrame(camera_->getFrame());
    return std::abs(colorFrame.cols/2);
}

cv::Rect2d PersonDetector::requestPersonBoundingBoxFromServer(cv::Mat colorFrame) {
    logger_->debug("requestPersonBoundingBoxFromServer");
    cv::Rect2d roi;
    if (colorFrame.empty()) {
        return roi;
    }
    auto bboxes = detector_->getBoundingBoxes(colorFrame);
    for (const auto& box : bboxes.get()) {
        if (box.classif == "person") {
            return box.box;
        } else {
            logger_->info("No person in FOV");
            return roi;
        }
    }
    logger_->info("No person in FOV");
    return roi;
}

}  // namespace personfollower
}  // namespace applications
}  // namespace crf
