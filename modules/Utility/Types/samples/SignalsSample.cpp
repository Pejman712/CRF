/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Signals.hpp"

using crf::utility::types::VectorXd;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;
using crf::utility::types::JointSignals;

using crf::utility::types::TaskPose;
using crf::utility::types::Vector6d;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::TaskSignals;

using crf::utility::types::Signals;

int main() {
    /**
     * Sample fields for JointSignals
    */
    crf::expected<JointPositions> jointPositions;
    crf::expected<JointVelocities> jointVelocities;
    crf::expected<JointAccelerations> jointAccelerations;
    crf::expected<JointForceTorques> jointForceTorques;

    /**
     * JointSignals is a struct for combining joints data.
    */
    JointSignals jointSignals;
    jointSignals.positions = jointPositions;
    jointSignals.velocities = jointVelocities;
    jointSignals.accelerations = jointAccelerations;
    jointSignals.forceTorques = jointForceTorques;

    jointPositions = jointSignals.positions;
    jointVelocities = jointSignals.velocities;
    jointAccelerations = jointSignals.accelerations;
    jointForceTorques = jointSignals.forceTorques;

    /**
     * Sample fields for TaskSignals
    */
    crf::expected<TaskPose> taskPose;
    crf::expected<TaskVelocity> taskVelocity;
    crf::expected<TaskAcceleration> taskAcceleration;
    crf::expected<TaskForceTorque> taskForceTorque;

    /**
     * TaskSignals is a struct for combining task data.
    */
    TaskSignals taskSignals;
    taskSignals.pose = taskPose;
    taskSignals.velocity = taskVelocity;
    taskSignals.acceleration = taskAcceleration;
    taskSignals.forceTorque = taskForceTorque;

    taskPose = taskSignals.pose;
    taskVelocity = taskSignals.velocity;
    taskAcceleration = taskSignals.acceleration;
    taskForceTorque = taskSignals.forceTorque;

    /**
     * Signals is a struct for combining JointSignals and TaskSignals.
    */
    Signals signals;
    signals.joints = jointSignals;
    signals.task = taskSignals;

    jointSignals = signals.joints;
    taskSignals = signals.task;

    return 0;
}
