/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <thread>
#include <memory>
#include <iostream>
#include <mutex>

#include "GraphPlot/MotionControllerPlotter/MotionControllerPlotter.hpp"

namespace crf::utility::graphplot {

MotionControllerPlotter::MotionControllerPlotter(
    std::shared_ptr<crf::control::motioncontroller::IMotionController> motion,
    const std::chrono::milliseconds& ts,
    const bool& logIntoAFile):
    motion_(motion),
    ts_(ts),
    logIntoAFile_(logIntoAFile),
    recording_(false),
    logger_("MotionControllerPlotter") {
        logger_->debug("CTor");
}

MotionControllerPlotter::~MotionControllerPlotter() {
    logger_->debug("DTor");
    stop();
}

void MotionControllerPlotter::start() {
    logger_->debug("start");
    recording_ = true;
    if (!signalThread_.joinable()) {
        signalThread_ = std::thread(&MotionControllerPlotter::requestSignals, this);
    }
}

void MotionControllerPlotter::stop() {
    logger_->debug("stop");
    recording_ = false;
    if (signalThread_.joinable()) {
        signalThread_.join();
    }
}

void MotionControllerPlotter::requestSignals() {
    logger_->debug("requestSignals");
    std::vector<crf::utility::types::JointSignals> signals;
    signals.reserve(startingMaxSize_);
    while (recording_) {
        signals.push_back(motion_->getSignals().joints);
        std::this_thread::sleep_for(ts_);
    }
    if (logIntoAFile_) {
        logger_->info("Logging into a file");
        std::ofstream logFile;
        logFile.open(logName_);
        // TODO(jplayang): A change of names following a standard would be the best solution
        // TODO(jplayang): Add logging for task signals and plotting
        logFile << "Position, Velocity, Acceleration, Torque\n";
        for (uint64_t i = 0; i < signals.size(); i++) {
            if (signals[i].positions) {
                logFile << signals[i].positions.value();
            } else {
                logFile << signals[i].positions.get_response();
            }
            logFile << ", ";
            if (signals[i].velocities) {
                logFile << signals[i].velocities.value();
            } else {
                logFile << signals[i].velocities.get_response();
            }
            logFile << ", ";
            if (signals[i].accelerations) {
                logFile << signals[i].accelerations.value();
            } else {
                logFile << signals[i].accelerations.get_response();
            }
            logFile << ", ";
            if (signals[i].forceTorques) {
                logFile << signals[i].forceTorques.value();
            } else {
                logFile << signals[i].forceTorques.get_response();
            }
            logFile << "\n";
        }
        logFile.close();
    }
    logger_->info("Plotting");
    PlotJointSignals(signals);
}

}  // namespace crf::utility::graphplot
