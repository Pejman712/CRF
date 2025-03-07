/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
*/

#include "TrajectoryGenerator/PreplannedTaskTrajectory/PreplannedTaskTrajectory.hpp"

void printTaskSignals(double time, crf::utility::types::TaskSignals signal) {
    std::cout << time << " ||| "
        << signal.pose.value().getPosition()[0] << " | "
        << signal.pose.value().getPosition()[1] << " | "
        << signal.pose.value().getPosition()[2] << " | "
        << signal.pose.value().getCardanXYZ()[0] << " | "
        << signal.pose.value().getCardanXYZ()[1] << " | "
        << signal.pose.value().getCardanXYZ()[2] << " ||| "
        << signal.velocity.value()[0] << " | " << signal.velocity.value()[1] << " | "
        << signal.velocity.value()[2] << " | " << signal.velocity.value()[3] << " | "
        << signal.velocity.value()[4] << " | " << signal.velocity.value()[5] << " ||| "
        << signal.acceleration.value()[0] << " | " << signal.acceleration.value()[1] << " | "
        << signal.acceleration.value()[2] << " | " << signal.acceleration.value()[3] << " | "
        << signal.acceleration.value()[4] << " | " << signal.acceleration.value()[5]
        << std::endl;
}

int main(int argc, char *argv[]) {
    // Path to the config file
    std::string filePath = __FILE__;
    filePath = filePath.substr(0, filePath.find("modules"));
    filePath += "modules/Actuators/Robot/config/SampleRobots/";
    filePath += "TaskTrajectoryWithPoseCardanVelocityAndAcceleration.csv";

    std::cout << "filePath: " << filePath << std::endl << std::endl;

    double cycleTime = 0.01;
    crf::control::trajectorygenerator::PreplannedTaskTrajectory traj(filePath, cycleTime);

    std::cout << "time ||| PoseX | PoseY | PoseZ | PoseCardanX | PoseCardanY | PoseCardanZ ||| "
        "VelX | VelY | VelZ | VelAngularX | VelAngularY | VelAngularZ ||| "
        "AccX | AccY | AccZ | AccAngularX | AccAngularY | AccAngularZ" << std::endl;

    // The trajectory starts
    crf::utility::types::TaskSignals result;
    double time = 0.0;
    while (traj.isTrajectoryRunning()) {
        result = traj.getTrajectoryPoint(time);
        printTaskSignals(time, result);
        time += cycleTime;
    }

    // Once the trajectory finished, the results will be constant
    for (uint64_t i = 0; i < 3; i++) {
        if (!traj.isTrajectoryRunning()) {
            result = traj.getTrajectoryPoint(time);
            printTaskSignals(time, result);
            time += cycleTime;
        }
    }

    // Once the trajectory is reseted, it can start again
    std::cout << std::endl << std::endl
        << "time ||| PoseX | PoseY | PoseZ | PoseCardanX | PoseCardanY | PoseCardanZ ||| "
        "VelX | VelY | VelZ | VelAngularX | VelAngularY | VelAngularZ ||| "
        "AccX | AccY | AccZ | AccAngularX | AccAngularY | AccAngularZ" << std::endl;
    traj.reset();
    time = 0.0;
    while (traj.isTrajectoryRunning()) {
        result = traj.getTrajectoryPoint(time);
        printTaskSignals(time, result);
        time += cycleTime;
    }
    std::cout << std::endl;

    return 0;
}
