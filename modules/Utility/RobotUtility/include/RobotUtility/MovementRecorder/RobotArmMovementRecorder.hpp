/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <thread>
#include <vector>

#include "RobotArmController/IRobotArmController.hpp"
#include "Types/Types.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace utility {
namespace movementrecorder {

class RobotArmMovementRecorder {
 public:
    RobotArmMovementRecorder() = delete;
    explicit RobotArmMovementRecorder(
        std::shared_ptr<crf::control::robotarmcontroller::IRobotArmController> armController,
        float margin);
    ~RobotArmMovementRecorder();

    void startRecording();
    void stopRecording();
    bool isRecording();
    std::vector<crf::utility::types::JointPositions> getRecordedPath();

 private:
    std::thread movementThread_;
    std::atomic<bool> stopThreads_;
    std::atomic<bool> recording_;
    std::mutex mtx_;
    std::condition_variable moveCV_;
    std::vector<crf::utility::types::JointPositions> manualPath_;
    std::shared_ptr<crf::control::robotarmcontroller::IRobotArmController> armController_;
    float margin_;
    crf::utility::logger::EventLogger logger_;

    void execute();
};

}  // namespace movementrecorder
}  // namespace utility
}  // namespace crf
