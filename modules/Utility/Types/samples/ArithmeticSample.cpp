/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Arithmetic.hpp"

using crf::utility::types::TaskPose;

int main() {
    /**
     * Sample taskPoses and angular velocities
    */
    TaskPose taskPose1(
        {-2.537896, 3.451273, -23.45126754},
        Eigen::Quaterniond{
            0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799});
    TaskPose taskPose2(
        {-14.53, 22.47, 2.3},
        Eigen::AngleAxisd(
            2.4832094549611980,
            Eigen::Vector3d({0.5816806901502302, 0.8134121496309343, 0.0028722010957822})));
    Eigen::Vector3d angularVelocity1({-1.505583160450048, 0.855195318314842, 0.951045633351785});

    /**
     * Multiplication.
    */
    TaskPose product = multiply(taskPose1, taskPose2);

    /**
     * Inversion.
    */
    TaskPose inverse = invert(taskPose1);

    return 0;
}
