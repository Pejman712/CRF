/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <vector>
#include <memory>
#include <iostream>

#include "MotionController/PathFollower/PathFollower.hpp"
#include "Robot/UniversalRobot/UniversalRobot.hpp"
#include "UniversalRobotRTDE/UniversalRobotRTDEInterface.hpp"
#include "TrajectoryGenerator/PointToPointJointsTrajectory/PointToPointJointsTrajectory.hpp"
#include "Controller/PositionCtrlVelocityFF/PositionCtrlVelocityFF.hpp"
#include "InverseKinematics/OptCLIK/OptCLIK.hpp"

int main(int argc, char** argv) {
    std::ifstream robotConfigFilePath(argv[1]);

    std::shared_ptr<crf::actuators::robot::UniversalRobot> robot =
        std::make_shared<crf::actuators::robot::UniversalRobot>(
            std::make_shared<crf::communication::universalrobotrtde::UniversalRobotRTDEInterface>(),
            crf::actuators::robot::UniversalRobotConfiguration(
                nlohmann::json::parse(robotConfigFilePath)));

    if (!robot->initialize()) {
        std::puts("Can't initialize!!");
        return -1;
    }

    std::shared_ptr<crf::control::inversekinematics::OptCLIK> inverseKinematics =
        std::make_shared<crf::control::inversekinematics::OptCLIK>(
            JointPositions({0, 0, 0, 0, 0, 0}),
            robot->getConfiguration()->getRobotControllerLoopTime(),
            robot->getConfiguration(),
            std::vector<double>(robot->getConfiguration()->getJointSpaceDoF(), 1));

    std::vector<double> Kp = {1, 1, 1, 1, 1, 1};
    std::vector<double> Ki = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    std::vector<double> Kd = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    double Ts = 2.0;

    std::shared_ptr<crf::control::controller::PositionCtrlVelocityFF> controller =
        std::make_shared<crf::control::controller::PositionCtrlVelocityFF>(
            Kp, Ki, Kd, Ts, inverseKinematics);

    auto robotConfig = robot->getConfiguration();

    std::shared_ptr<crf::control::trajectorygenerator::PointToPointJointsTrajectory> trajGenerator =
        std::make_shared<crf::control::trajectorygenerator::PointToPointJointsTrajectory>(
            robotConfig->getProfileParameters().jointVelocities,
            robotConfig->getProfileParameters().jointAccelerations);

    std::shared_ptr<crf::control::motioncontroller::PathFollower> motion =
        std::make_shared<crf::control::motioncontroller::PathFollower>(
            robot, controller, trajGenerator, nullptr);

    sleep(1);

    auto path = {
        crf::utility::types::JointPositions({0, -1.57, 0, 0, 0, 0}),
        crf::utility::types::JointPositions({0, 0, -1.57, 0, 0, 0}),
        crf::utility::types::JointPositions({0, -1.57, 0, 0, 0, 0}),
        crf::utility::types::JointPositions({0, 0, -1.57, 0, 0, 0})
    };

    motion->appendPath(path);

    motion->appendPath(path);

    sleep(7);

    motion->softStop();
}
