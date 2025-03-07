/* © Copyright CERN 2024.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@crf.ch
 *
 * Author: Chelsea Davidson CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
*/
#include "TrajectoryGenerator/CubicTaskTrajectory/CubicTaskTrajectory.hpp"

using crf::control::trajectorygenerator::CubicTaskTrajectory;

int main(int argc, char** argv) {
    // Create a Cubic Task Trajectory object
    TaskVelocity maxVelocity({0.1, 0.1, 0.1, 0.5, 0.5, 0.5});
    TaskAcceleration maxAcceleration({0.1, 0.1, 0.1, 0.5, 0.5, 0.5});

    CubicTaskTrajectory taskTrajectory(maxVelocity, maxAcceleration);

    // Set initial pose
    TaskPose initialPose = TaskPose(
        Eigen::Vector3d(0, 0, 0),
        Eigen::Quaternion<double>(1.0, 0, 0, 0));

    taskTrajectory.setInitialPose(initialPose);

    // Append a path
    std::vector<TaskPose> path1 = {
        TaskPose(Eigen::Vector3d(0, 0, 0.5), Eigen::Quaternion<double>(1.0, 0, 0, 0)),
        TaskPose(Eigen::Vector3d(0.1, 0, 0.5), Eigen::Quaternion<double>(0, 0, 0, 1.0))};

    taskTrajectory.append(path1);

    // Evaluate the trajectory
    TaskSignals initialPoint = taskTrajectory.getTrajectoryPoint(0.0);

    std::cout << "Position at time 30s:\n"<<
        taskTrajectory.getTrajectoryPoint(30.0).pose.value().getPosition() << std::endl;
    std::cout << "\nOrientation at time 30s:\n"<<
        taskTrajectory.getTrajectoryPoint(30.0).pose.value().getOrientation() << std::endl;
    std::cout << "\nVelocity at time 30s:\n"<<
        taskTrajectory.getTrajectoryPoint(30.0).velocity.value() << std::endl;
    std::cout << "\nAcceleration at time 30s:\n"<<
        taskTrajectory.getTrajectoryPoint(30.0).acceleration.value() << std::endl;

    // Updating the profile velocity and acceleration
    taskTrajectory.setProfileVelocity(TaskVelocity({0.2, 0.2, 0.2, 0.6, 0.6, 0.6}));
    taskTrajectory.setProfileAcceleration(TaskAcceleration({0.3, 0.3, 0.3, 0.4, 0.4, 0.4}));

    // Append a single point
    std::vector<TaskPose> path2 = {
        TaskPose(Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Quaternion<double>(0, 0, 1.0, 0))};

    taskTrajectory.append(path2);

    // Seeing if there are still points left to evaluate in the trajectory
    // - Trajectory running should be 1 since we just appended a point
    std::cout << "\nTrajectory running: " << taskTrajectory.isTrajectoryRunning() << std::endl;

    // Clearing memory of previous trajectories
    taskTrajectory.clearMemory();

    // Resetting the trajectory
    taskTrajectory.reset();
    taskTrajectory.setInitialPose(initialPose);  // Must set the initial pose after a reset

    // Can get trajectory point when only initial pose has been set
    std::cout << "\nPosition at time 0s after reset:\n" <<
        taskTrajectory.getTrajectoryPoint(0.0).pose.value().getPosition() << std::endl;
    std::cout << "\nOrientation at time 0s after reset:\n" <<
        taskTrajectory.getTrajectoryPoint(0.0).pose.value().getOrientation() << std::endl;
    std::cout << "\nVelocity at time 0s after reset:\n" <<
        taskTrajectory.getTrajectoryPoint(0.0).velocity.value() << std::endl;
    std::cout << "\nAcceleration at time 0s after reset:\n" <<
        taskTrajectory.getTrajectoryPoint(0.0).acceleration.value() << std::endl;

    return 0;
}
