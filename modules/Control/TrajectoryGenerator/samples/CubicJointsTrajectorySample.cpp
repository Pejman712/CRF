/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@crf.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/

#include "TrajectoryGenerator/CubicJointsTrajectory/CubicJointsTrajectory.hpp"

int main(int argc, char *argv[]) {
    // Three dimensions, four points
    std::vector<crf::utility::types::JointPositions> points = {
        crf::utility::types::JointPositions({0, 0, 0}),
        crf::utility::types::JointPositions({1, 1, 1}),
        crf::utility::types::JointPositions({0, 0, 0}),
        crf::utility::types::JointPositions({2, 2, 2})};

    // Max vel and Acc for all the dimensions
    crf::utility::types::JointVelocities maxVel({1, 1, 1});
    crf::utility::types::JointAccelerations maxAcc({1, 1, 1});

    crf::control::trajectorygenerator::CubicJointsTrajectory traj(maxVel, maxAcc);

    traj.append(points);

    std::cout << "1st joint pos at time 0 " <<
    traj.getTrajectoryPoint(0).positions.value()[0] << std::endl;
    std::cout << "1st joint pos at time 1 " <<
    traj.getTrajectoryPoint(1).positions.value()[0] << std::endl;
    std::cout << "1st joint pos at time 2 " <<
    traj.getTrajectoryPoint(2).positions.value()[0] << std::endl;

    return 0;
}
