/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <thread>
#include <memory>
#include <atomic>
#include <condition_variable>
#include <string>

#include "MotionController/IMotionController.hpp"
#include "GraphPlot/GraphPlot.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::utility::graphplot {

/**
 * @brief Class that records signals from a motion controller and plots them once
 * it's stopped or destroyed. One graph will be created per joint. It will display
 * the values of position, velocity, acceleration, and torque over time for that joint.
 *
 * If the option to log into a file is selected, then a file with the name
 * "ControllerSignalsLog.csv" is created and filled following the next structure:
 *
 *    1 - Position, Velocity, Acceleration, Torque
 *    2 - jointPositions, jointVelocities, ...    -> in t0
 *    ....
 *    n - jointPositions, jointVelocities, ...    -> in tf
 *
 * Where n is the number of samples taken, t0 is the starting time of recording and
 * tf is the time where the final sample was taken before stopping the log.
 *
 * If one of the samples could not be taken then the correspondent error code gets printed
 */

class MotionControllerPlotter {
 public:
    MotionControllerPlotter() = delete;
    MotionControllerPlotter(
      std::shared_ptr<crf::control::motioncontroller::IMotionController> motion,
      const std::chrono::milliseconds& ts,
      const bool& logIntoAFile = false);
    ~MotionControllerPlotter();

    /**
     * @brief Starts the logging. It starts a thread that continously requests signals from
     * the controller and saves the data.
     *
     */
    void start();

    /**
     * @brief Stops the logging and plots the results.
     *
     */
    void stop();

 private:
    std::shared_ptr<crf::control::motioncontroller::IMotionController> motion_;
    std::chrono::milliseconds ts_;
    bool logIntoAFile_;

    std::thread signalThread_;
    std::atomic<bool> recording_;

    crf::utility::logger::EventLogger logger_;

    /**
     * @brief Starting size to initialize the vectors. For a control loop of 10ms,
     * it gives enough memory for the starting 100s.
     *
     */
    const int startingMaxSize_ = 10000;

    /**
     * @brief Name of the logging file.
     *
     */
    const std::string logName_ = "ControllerSignalsLog.csv";

    void requestSignals();
};

}  // namespace crf::utility::graphplot

