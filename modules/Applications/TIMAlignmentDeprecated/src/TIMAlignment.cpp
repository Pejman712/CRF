/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <boost/optional.hpp>
#include <opencv2/core.hpp>

#include "CommUtility/StreamReader.hpp"
#include "TIMAlignment/TIMAlignment.hpp"
#include "VisionTypes/VisionTypes.hpp"

using crf::communication::zmqcomm::Request;
using crf::vision::types::BoundingBox;

namespace crf {
namespace applications {
namespace timalignment {

TIMAlignment::TIMAlignment(std::shared_ptr<communication::networkclient::INetworkClient> client,
    std::shared_ptr<vision::objectDetection::IObjectDetector> objectDetector,
    std::shared_ptr<Request> cameraRequester, const Packets::FetchWritePacket& packet,
    const std::chrono::milliseconds& timeOut):
    logger_("TIMAlignment"),
    client_(client),
    objectDetector_(objectDetector),
    cameraRequester_(cameraRequester),
    packet_(packet),
    timeOut_(timeOut),
    stopSignal_(false) {
    logger_->debug("Constructor");
}

bool TIMAlignment::initialize() {
    logger_->debug("initialize");
    if (app_.joinable()) {
        logger_->warn("The Object Detector has been already initialized before");
        return false;
    }
    if (!objectDetector_->initialize()) {
        logger_->error("Problem to initialize the Object Detector");
        return false;
    }
    if (!client_->connect()) {
        logger_->error("Problem to connect to the PCL");
        return false;
    }
    if (!cameraRequester_->open()) {
        logger_->error("There is a problem to open the Camera Requester");
        return false;
    }
    if (!cameraRequester_->setRcvTimeOut(timeOut_)) {
        logger_->error("There is a problem to set the receiver timeout of Camera Requester");
        return false;
    }
    if (!cameraRequester_->setSndTimeOut(timeOut_)) {
        logger_->error("There is a problem to set the sender timeout of Camera Requester");
        return false;
    }
    try {
        app_ = std::thread(&TIMAlignment::run, this);
    } catch (const std::system_error& e) {
        logger_->error("There is a problem to initialize the tim alignment thread: {0}", e.what());
        return false;
    }
    logger_->info("The TIM Alignment thread is now running");
    return true;
}

bool TIMAlignment::deinitialize() {
    logger_->debug("deinitialize");
    if (!app_.joinable()) {
        logger_->warn("TIM Alignment has not been initialized, so it can't be deinitialize");
        return false;
    }
    stopSignal_ = true;
    try {
        app_.join();
    } catch (const std::system_error& e) {
        logger_->error("There is a problem to join alignment thread to the main: {0}", e.what());
        return false;
    }
    if (!objectDetector_->deinitialize()) {
        logger_->error("There is a problem to deinitialize the Object Detector");
        return false;
    }
    if (!cameraRequester_->close()) {
        logger_->error("There is a problem to close the Camera Requester");
        return false;
    }
    if (!client_->disconnect()) {
        logger_->error("There is a problem to disconnec the Network Client");
        return false;
    }
    logger_->debug("The TIM Alignment thread has been stopped");
    return true;
}

void TIMAlignment::run() {
    logger_->debug("run");
    while (!stopSignal_) {
        auto state = checkingDetection();
        switch (state.first) {
            case guides2            : moveWith2Guides(state.second);            break;
            case collimator1        : moveWith1Collimator(state.second);        break;
            case guide1NoCollimator : moveWith1Guide0Collimator(state.second);  break;
            default                 : noDetection();
        }
    }
    packet_.data_ = std::vector<uint8_t>({0, 0});
    client_->send(packet_.getHeader(), packet_.serialize());
    stopSignal_ = false;
}

int TIMAlignment::getCenter(int leftPosition, int rightPosition) {
    logger_->debug("getCenter");
    int center = leftPosition + (rightPosition - leftPosition) / 2;
    return center;
}

boost::optional<cv::Mat> TIMAlignment::getFrameFromServer() {
    logger_->debug("getFrameFromServer");
    std::string msg = "ColorFrame";
    if (!cameraRequester_->write(msg)) {
        logger_->warn("There was a problem to write a request for a Color Frame");
        return boost::none;
    }
    if (!cameraRequester_->read(&msg)) {
        logger_->warn("There was a problem to read a reply for a Color Frame");
        return boost::none;
    }
    cv::Mat frame;
    Packets::StreamReader reader(msg);
    reader.read(&frame);
    return frame;
}

std::pair<TIMAlignment::State, TIMAlignment::Positions> TIMAlignment::checkingDetection() {
    logger_->debug("checkingDetection");
    Positions positions;
    auto colorFrame = getFrameFromServer();
    if (!colorFrame.is_initialized()) return std::make_pair(noDeviceDetected, positions);
    auto bboxes = objectDetector_->getBoundingBoxes(*colorFrame);
    if (!bboxes.is_initialized()) return std::make_pair(noDeviceDetected, positions);

    int leftGuide = 0;
    int rightGuide = 0;
    int collimatorPosition = 0;
    int numCollimators = 0;
    int numGuides = 0;

    for (const auto& bbox : bboxes.get()) {
        if (bbox.classif == "Collimator") {
            numCollimators++;
            int position = getCenter(bbox.box.tl().x, bbox.box.br().x);
            collimatorPosition = position;
        }
        if (bbox.classif == "Guide") {
            numGuides++;
            int position = getCenter(bbox.box.tl().x, bbox.box.br().x);
            if (leftGuide == 0) {
                leftGuide = position;
            } else if (position < leftGuide) {
                rightGuide = leftGuide;
                leftGuide = position;
            } else {
                rightGuide = position;
            }
            logger_->debug("leftGuide: {0}    rightGuide: {1}", leftGuide, rightGuide);
        }
    }
    positions.leftGuide = leftGuide;
    positions.rightGuide = rightGuide;
    positions.collimatorPosition = collimatorPosition;
    positions.mediumOfFrame = colorFrame->cols / 2;

    if (numCollimators == 1 && numGuides == 2) return std::make_pair(guides2, positions);
    if (numCollimators == 0 && numGuides == 2) return std::make_pair(guides2, positions);
    if (numCollimators == 1 && numGuides != 2) return std::make_pair(collimator1, positions);
    if (numCollimators == 0 && numGuides == 1) return std::make_pair(guide1NoCollimator, positions);
    return std::make_pair(noDeviceDetected, positions);
}

void TIMAlignment::moveWith2Guides(Positions positions) {
    logger_->debug("moveWith2Guides");
    int position = getCenter(positions.leftGuide, positions.rightGuide);
    logger_->info("Two Guides");
    if (position > (positions.mediumOfFrame + positions.mediumOfFrame * THRESHOLD)) {
        logger_->debug("Position: {0} > {1}", position,
            positions.mediumOfFrame + positions.mediumOfFrame * THRESHOLD);
        logger_->info("Move Forward");
        packet_.data_ = std::vector<uint8_t>({0, FORWARD});
    } else if (position < (positions.mediumOfFrame - positions.mediumOfFrame * THRESHOLD)) {
        logger_->debug("Position: {0} < {1}", position,
            positions.mediumOfFrame - positions.mediumOfFrame * THRESHOLD);
        logger_->info("Move Backward");
        packet_.data_ = std::vector<uint8_t>({0, BACKWARD});
    } else {
        logger_->info("STOP");
        packet_.data_ = std::vector<uint8_t>({0, STOP});
    }
    logger_->info("Position: {0}", position);
    client_->send(packet_.getHeader(), packet_.serialize());
}

void TIMAlignment::moveWith1Collimator(Positions positions) {
    logger_->debug("moveWith1Collimator");
    logger_->info("Collimator");
    if (positions.collimatorPosition > positions.mediumOfFrame) {
        logger_->debug("Position: {0} < {1}", positions.collimatorPosition,
            positions.mediumOfFrame);
        logger_->info("Move Forward");
        packet_.data_ = std::vector<uint8_t>({0, FORWARD});
    } else {
        logger_->info("Move Backward");
        logger_->debug("Position: {0} > {1}", positions.collimatorPosition,
            positions.mediumOfFrame);
        packet_.data_ = std::vector<uint8_t>({0, BACKWARD});
    }
    logger_->info("Position: {0}", positions.collimatorPosition);
    client_->send(packet_.getHeader(), packet_.serialize());
}

void TIMAlignment::moveWith1Guide0Collimator(Positions positions) {
    logger_->debug("moveWith1Guide0Collimator");
    logger_->info("One Guide");
    int position = positions.leftGuide;
    if (position > (positions.mediumOfFrame + positions.mediumOfFrame * THRESHOLD)) {
        logger_->info("Move Forward");
        packet_.data_ = std::vector<uint8_t>({0, FORWARD});
    } else if (position < (positions.mediumOfFrame - positions.mediumOfFrame * THRESHOLD)) {
        logger_->info("Move Backward");
        packet_.data_ = std::vector<uint8_t>({0, BACKWARD});
    } else {
            logger_->info("STOP");
        packet_.data_ = std::vector<uint8_t>({0, STOP});
    }
    logger_->info("Position: {0}", position);
    client_->send(packet_.getHeader(), packet_.serialize());
}

void TIMAlignment::noDetection() {
    logger_->debug("noDetection");
    logger_->info("It detects nothing");
    packet_.data_ = std::vector<uint8_t>({0, IDLE});
    client_->send(packet_.getHeader(), packet_.serialize());
}

}  // namespace timalignment
}  // namespace applications
}  // namespace crf
