/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <memory>
#include <thread>
#include <vector>

#include "RobotUtility/MovementRecorder/RobotArmMovementRecorder.hpp"

namespace crf {
namespace utility {
namespace movementrecorder {

RobotArmMovementRecorder::RobotArmMovementRecorder(
    std::shared_ptr<crf::control::robotarmcontroller::IRobotArmController> armController,
    float margin):
    movementThread_(),
    stopThreads_(false),
    recording_(false),
    mtx_(),
    moveCV_(),
    manualPath_(),
    armController_(armController),
    margin_(margin),
    logger_("RobotArmMovementRecorder") {
    logger_->debug("CTor");
    if (!movementThread_.joinable()) {
        movementThread_ = std::thread(&RobotArmMovementRecorder::execute, this);
    }
}

RobotArmMovementRecorder::~RobotArmMovementRecorder() {
    logger_->debug("DTor");
    stopThreads_ = true;
    if (movementThread_.joinable()) {
        movementThread_.join();
    }
}

void RobotArmMovementRecorder::startRecording() {
    logger_->debug("startRecording");
    manualPath_.clear();
    recording_ = true;
    moveCV_.notify_one();
}

void RobotArmMovementRecorder::stopRecording() {
    logger_->debug("stopRecording");
    recording_ = false;
}

bool RobotArmMovementRecorder::isRecording() {
    logger_->debug("isRecording");
    return recording_;
}

std::vector<crf::utility::types::JointPositions> RobotArmMovementRecorder::getRecordedPath() {
    logger_->debug("getRecordedPath");
    return manualPath_;
}

void RobotArmMovementRecorder::execute() {
    logger_->debug("execute");
    while (!stopThreads_) {
        std::unique_lock<std::mutex> lck(mtx_);
        moveCV_.wait(lck);
        crf::utility::types::JointPositions prev_position = armController_->getJointPositions();
        bool record;
        bool first = true;
        while (recording_) {
            crf::utility::types::JointPositions currentPosition =
                armController_->getJointPositions();
            if (first) {
                record = true;
                first = false;
            } else {
                record = crf::utility::types::areAlmostEqual(
                    currentPosition,
                    prev_position,
                    margin_);
            }
            if (record) {
                logger_->info("Recording Position");
                prev_position = currentPosition;
                manualPath_.push_back(currentPosition);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

}  // namespace movementrecorder
}  // namespace utility
}  // namespace crf
