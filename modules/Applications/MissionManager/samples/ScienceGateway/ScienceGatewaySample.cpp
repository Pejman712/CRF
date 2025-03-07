/* © Copyright CERN 2023.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include <signal.h>
#include <csignal>
#include <fstream>

#include "Robot/KinovaGen3/KinovaGen3.hpp"
#include "Robot/KinovaGen3/KinovaGen3Configuration.hpp"
#include "KortexAPI/KortexMovementAPIInterface.hpp"

#include "MotionController/PathFollower/PathFollower.hpp"
#include "TrajectoryGenerator/PointToPointJointsTrajectory/PointToPointJointsTrajectory.hpp"
#include "TrajectoryGenerator/CubicJointsTrajectory/CubicJointsTrajectory.hpp"
#include "Controller/PositionCtrlVelocityFF/PositionCtrlVelocityFF.hpp"

#include "MissionUtility/DeployableRobot/DeployableRobot.hpp"
#include "MissionManager/ScienceGateway/ScienceGateway.hpp"
#include "GraphPlot/MotionControllerPlotter/MotionControllerPlotter.hpp"

int main(int argc, char *argv[]) {
    // Robot Arm: KinovaGen3
    std::ifstream kinovagen3ConfigFilePath(argv[1]);
    std::shared_ptr<crf::communication::kortexapi::KortexMovementAPIInterface> kinovainterface =
        std::make_shared<crf::communication::kortexapi::KortexMovementAPIInterface>();
    std::shared_ptr<crf::actuators::robot::KinovaGen3> kinovag3 =
        std::make_shared<crf::actuators::robot::KinovaGen3>(kinovainterface,
            crf::actuators::robot::KinovaGen3Configuration(
                nlohmann::json::parse(kinovagen3ConfigFilePath)));

    // Motion Controller: Pathcontroller
    std::vector<double> Kp = {1, 1, 1, 1.25, 1.3, 0.25};
    std::vector<double> Ki = {0.2, 0.2, 0.2, 0.15, 0.1, 0.1};
    std::vector<double> Kd = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    double Ts = 0.07;

    std::shared_ptr<crf::control::controller::PositionCtrlVelocityFF> controller =
        std::make_shared<crf::control::controller::PositionCtrlVelocityFF>(Kp, Ki, Kd, Ts, nullptr);

    auto robotConfig = kinovag3->getConfiguration();

    std::shared_ptr<crf::control::trajectorygenerator::PointToPointJointsTrajectory> trajGenerator =
        std::make_shared<crf::control::trajectorygenerator::PointToPointJointsTrajectory>(
            robotConfig->getProfileParameters().jointVelocities,
            robotConfig->getProfileParameters().jointAccelerations);

    std::shared_ptr<crf::control::motioncontroller::PathFollower> motioncontroller =
        std::make_shared<crf::control::motioncontroller::PathFollower>(
            kinovag3, controller, trajGenerator, nullptr);

    // Plotter
    crf::utility::graphplot::MotionControllerPlotter plot(motioncontroller,
        std::chrono::milliseconds(100), true);

    // Deployable Robotarm
    std::ifstream deployableConfigFilePath(argv[2]);
    std::shared_ptr<crf::utility::missionutility::DeployableRobot> deployablerobot =
        std::make_shared<crf::utility::missionutility::DeployableRobot>(
        motioncontroller, nlohmann::json::parse(deployableConfigFilePath));

    // Sciencegateway Mission
    std::ifstream scigatewayConfigFilePath(argv[3]);
    std::shared_ptr<
        crf::applications::missionmanager::sciencegateway::ScienceGateway> scigateway =
    std::make_shared<crf::applications::missionmanager::sciencegateway::ScienceGateway>(
        deployablerobot, nlohmann::json::parse(scigatewayConfigFilePath));

    // choose the mission, three mission in total:
    //   Easy: Deploy, Look around, Retract
    //   Medium: Deploy, Look around, Retract
    //   Hard: Deploy, Look around, Retract
    nlohmann::json missionJson;
    // easy mode
    missionJson["MissionSelect"] =
        crf::applications::missionmanager::sciencegateway::MissionSelect::Easy;
    scigateway->start();
    plot.start();
    scigateway->setStatus(missionJson);
    sleep(2);
    scigateway->goHome();
    plot.stop();
    scigateway->stop();
    sleep(5);

    // medium mode
    missionJson["MissionSelect"] =
        crf::applications::missionmanager::sciencegateway::MissionSelect::Medium;
    scigateway->start();
    scigateway->setStatus(missionJson);
    scigateway->next();
    sleep(2);
    scigateway->goHome();
    scigateway->stop();
    sleep(5);

    // hard mode
    missionJson["MissionSelect"] =
        crf::applications::missionmanager::sciencegateway::MissionSelect::Hard;
    scigateway->start();
    scigateway->setStatus(missionJson);
    scigateway->next();
    sleep(2);
    scigateway->goHome();
    scigateway->stop();

    scigateway->start();
    scigateway->stop();
    scigateway->start();
    scigateway->stop();
    scigateway->start();
    scigateway->stop();
    // while(true){}
}
