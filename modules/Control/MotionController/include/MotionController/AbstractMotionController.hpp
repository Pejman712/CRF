/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <set>

#include "Controller/IController.hpp"
#include "Robot/IRobot.hpp"
#include "Robot/RobotConfiguration.hpp"
#include "TrajectoryGenerator/IJointTrajectoryGenerator.hpp"
#include "TrajectoryGenerator/ITaskTrajectoryGenerator.hpp"
#include "InverseKinematics/IClosedLoopInverseKinematics.hpp"
#include "InverseKinematics/IKinematicObjectiveFunction.hpp"
#include "ForwardKinematics/IForwardKinematics.hpp"
#include "Types/Types.hpp"
#include "MotionController/IMotionController.hpp"
#include "GeometricMethods/Sinusoid/Sinusoid.hpp"

#include "EventLogger/EventLogger.hpp"

using crf::actuators::robot::IRobot;
using crf::control::controller::IController;
using crf::actuators::robot::RobotConfiguration;
using crf::math::geometricmethods::Sinusoid;
using crf::math::geometricmethods::ComputationMethod;

namespace crf::control::motioncontroller {

/**
 * @ingroup group_motion_controller
 * @brief Abstract class to serve as base for the motion controllers
 *
 * @details a commandTimeInterval of 0 means deactivated
 */
class AbstractMotionController: public IMotionController {
 public:
    explicit AbstractMotionController(
        std::shared_ptr<IRobot> robot,
        std::shared_ptr<IController> controller,
        const std::chrono::milliseconds& commandTimeInterval = std::chrono::milliseconds(0));
    ~AbstractMotionController() override;

    bool initialize() override;
    bool deinitialize() override;

    void startJointsControlLoop() override;
    void startTaskControlLoop() override;

    void softStop() override;
    void hardStop() override;

    Signals getSignals() override;
    std::set<crf::Code> getStatus() override;
    std::shared_ptr<RobotConfiguration> getConfiguration() override;

 protected:
    /**
     * @brief Method to change the control type to Joint control
     *
     */
    void switchToJointControl();

    /**
     * @brief Method to change the control type to Task control
     *
     */
    void switchToTaskControl();

    /**
     * @brief Cheks if the control loop is running
     *
     * @return true if it's running
     * @return false if it's not running
     */
    bool isControlLoopActive();

    /**
     * @brief Starts the control loop if it's not started yet
     *
     */
    void startControlLoop();

    /**
     * @brief Stops the control loop if it's started
     *
     */
    void stopControlLoop();

    /**
     * @brief Function that implements the joint control to run in the loop
     *
     * @details This function must be overwritten
     *
     * @return JointSignals The output to the robot as control signals
     */
    virtual Signals jointControl() = 0;

    /**
     * @brief Function that implements the task control to run in the loop
     *
     * @details This function must be overwritten
     *
     * @return JointSignals The output to the robot as control signals
     */
    virtual Signals taskControl() = 0;

    /**
     * @brief Receives a reference in task and scales it down if smooth stop is active
     *
     * @param referenceTask The reference
     * @return TaskSignals The scaled down reference
     */
    TaskSignals smoothStopScaling(const TaskSignals& referenceTask);

    /**
     * @brief Receives a reference in task and scales it down if smooth stop is active
     *
     * @param referenceJoint The reference
     * @return JointSignals The scaled down reference
     */
    JointSignals smoothStopScaling(const JointSignals& referenceJoint);

    /**
     * @brief Mehtod to update the time the last command was sent
     *
     */
    void updateLastCommandTime();

    /**
     * @brief Method to dictate the behaviour after a command timeout. By default it just prints
     * a logger message
     *
     * @details This class can be overwritten if a behaviour wants to be set
     *
     */
    virtual void commandTimeout();

    std::chrono::microseconds loopTime_;
    double trajectoryTimeCounter_;

    std::atomic<bool> smoothStop_;
    JointSignals robotJointSignals_;
    TaskSignals robotTaskSignals_;
    Signals lastDesiredSignals_;
    std::chrono::high_resolution_clock::time_point lastCommandTime_;

    std::set<crf::Code> codes_;

 private:
    std::shared_ptr<IRobot> robot_;
    std::shared_ptr<IController> controller_;
    std::chrono::milliseconds commandTimeInterval_;

    std::shared_ptr<RobotConfiguration> robotConfig_;
    std::atomic<bool> controlLoopStopped_;
    std::thread controlLoopThread_;

    std::unique_ptr<Sinusoid> sinusoid_;

    bool strictRealTime_;
    double softStopRange_;

    std::atomic<bool> jointControl_;
    std::atomic<bool> commandSet_;
    std::atomic<uint64_t> smoothStopCounter_;

    std::mutex robotMtx_;
    std::mutex softStopMtx_;
    std::condition_variable statusAccess_;
    std::condition_variable stoppedLoop_;

    crf::utility::logger::EventLogger logger_;

    struct SmoothStopSigmas {
        double sigma0thDer;
        double sigma1stDer;
    };

    SmoothStopSigmas calculateSigma();

    void controlLoop();
    JointSignals getRobotJointValues();
    TaskSignals calculateRobotTaskValues(const JointSignals& jointValues);
    crf::expected<bool> sendSignalToRobot(const Signals& robotInput);
};

}  // namespace crf::control::motioncontroller
