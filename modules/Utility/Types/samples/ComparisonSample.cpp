/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Comparison.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;

using crf::utility::types::areAlmostEqual;
using crf::utility::types::isBetween;
using crf::utility::types::isLesser;
using crf::utility::types::isGreater;

int main() {
    /**
     * Sample variables of some of the types that can be compared
    */
    JointPositions jointPositions1(
        {-2.55417, -std::numeric_limits<double>::infinity(), 4.3433, -8.16, 52.16, 2.0, -4.8});
    JointPositions jointPositions2({15.4, 23.2, 512.6, -3.16, 84.44, 80.455743523234, -3.0});
    JointPositions jointPositions3(
        {21.0, 23.2000001, 516.0, std::numeric_limits<double>::infinity(), 882.24, 81.2, -2.0});

    TaskPose taskPose1(
        {-2.537896, 3.451273, -23.45126754},
        Eigen::Quaterniond{
            0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799});
    TaskPose taskPose2(
        {-14.53, 22.47, 2.3},
        Eigen::AngleAxisd(
            2.4832094549611980,
            Eigen::Vector3d({0.5816806901502302, 0.8134121496309343, 0.0028722010957822})));

    TaskVelocity taskVelocity1(
        {-2.55417, -std::numeric_limits<double>::infinity(), 4.3433, -8.16, 52.16, 2.0});
    TaskVelocity taskVelocity2({15.4, 23.2, 512.6, -3.16, 84.44, 80.455743523234});
    TaskVelocity taskVelocity3(
        {21.0, 23.2000001, 516.0, std::numeric_limits<double>::infinity(), 882.24, 81.2});

    /**
     * In following examples JointVelocities, JointAccelerations and JointForceTorques
     * behave the same as JointPositions.
     * TaskAcceleration, TaskForceTorque behaves the same as TaskVelocity.
    */

    /**
     * Are almost equal
    */
    /**
     * Accuracy is defaulted to 1e-12. In all the types it can be passed as follows:
    */
    areAlmostEqual(jointPositions1, jointPositions2, 1e-7);  // false

    areAlmostEqual(jointPositions1, jointPositions1);  // true
    areAlmostEqual(jointPositions1, jointPositions2);  // false

    areAlmostEqual(taskPose1, taskPose1);  // true
    areAlmostEqual(taskPose1, taskPose2);  // false

    areAlmostEqual(taskVelocity1, taskVelocity1);  // true
    areAlmostEqual(taskVelocity1, taskVelocity2);  // false

    /**
     * Is between
    */
    isBetween(jointPositions1, jointPositions3, jointPositions2);  // true
    isBetween(jointPositions1, jointPositions2, jointPositions3);  // false

    isBetween(taskVelocity1, taskVelocity3, taskVelocity2);  // true
    isBetween(taskVelocity1, taskVelocity2, taskVelocity3);  // false

    /**
     * Is lesser
    */
    isLesser(jointPositions1, jointPositions2);  // true
    isLesser(jointPositions2, jointPositions1);  // false

    isLesser(taskVelocity1, taskVelocity2);  // true
    isLesser(taskVelocity2, taskVelocity1);  // false

    /**
     * Is greater
    */
    isGreater(jointPositions2, jointPositions1);  // true
    isGreater(jointPositions1, jointPositions2);  // false

    isGreater(taskVelocity2, taskVelocity1);  // true
    isGreater(taskVelocity1, taskVelocity2);  // false

    return 0;
}
