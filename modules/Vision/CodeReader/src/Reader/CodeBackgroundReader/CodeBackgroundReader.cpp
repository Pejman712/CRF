/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include "CodeReader/Reader/CodeBackgroundReader/CodeBackgroundReader.hpp"

namespace crf::vision::codereader {

CodeBackgroundReader::CodeBackgroundReader(
    std::shared_ptr<crf::sensors::cameras::ICamera> camera,
    std::shared_ptr<IDecoder> decoder,
    std::shared_ptr<IDetector> detector):
    camera_(camera),
    detector_(detector),
    decoder_(decoder),
    detected_(false),
    logger_("CodeBackgroundReader") {
        logger_->debug("CTor");
        camera_->initialize();
        stopThreads_ = false;
        QRThread_ = std::thread(&CodeBackgroundReader::execute, this);
}

CodeBackgroundReader::~CodeBackgroundReader() {
    logger_->debug("DTor");
    stopThreads_ = true;
    record_ = false;
    recordCV_.notify_one();
    if (QRThread_.joinable()) {
        QRThread_.join();
    }
    camera_->deinitialize();
}

void CodeBackgroundReader::start() {
    logger_->debug("start");
    record_ = true;
    recordCV_.notify_one();
}

void CodeBackgroundReader::stop() {
    logger_->debug("stop");
    record_ = false;
}

bool CodeBackgroundReader::isDetected() {
    logger_->debug("isDetected");
    return detected_;
}

std::string CodeBackgroundReader::getCode() {
    logger_->debug("getCode");
    std::scoped_lock<std::mutex> lck(mtxContent_);
    return content_;
}

crf::utility::types::TaskPose CodeBackgroundReader::getCodeTaskPose() {
    logger_->debug("getCodePosition");
    // If it's not detected then the detector will throw an exception
    return detector_->getCodeTaskPose()[0];
}

void CodeBackgroundReader::execute() {
    logger_->debug("execute");
    std::array<float, 8> positions;
    while (!stopThreads_) {
        std::unique_lock<std::mutex> lck(mtx_);
        recordCV_.wait(lck);
        detected_ = false;
        while (record_) {
            std::this_thread::sleep_for(loopTime_);
            frame_ = camera_->captureImage();
            if (frame_.empty()) {
                logger_->warn("Could not capture a frame from the camera");
                continue;
            }
            std::scoped_lock<std::mutex> lckcont(mtxContent_);
            crf::expected<std::string> content;
            if (detector_ != nullptr) {
                if (!detector_->detect(frame_)) {
                    continue;
                }
                positions = detector_->getPositionInImage();
                if (positions.empty()) {
                    continue;
                }
                content = decoder_->decode(frame_, positions);
            } else {
                content = decoder_->decode(frame_);
            }
            if (!content) continue;
            content_ = content.value();
            detected_ = true;
            record_ = false;
            result_ = positions;
            logger_->info("QR code found");
        }
    }
}

}  // namespace crf::vision::codereader
