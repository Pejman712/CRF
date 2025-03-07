/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>

#include "RobotArmControllersDeprecated/RobotArmSlaveController.hpp"
#include "RobotArmKinematics/RobotArmKDLKinematics/RobotArmKDLKinematics.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "ClosedLoopController/PIDController.hpp"
#include "SchunkArm/SchunkArm.hpp"
#include "CANSocket/CANSocket.hpp"
#include "Types/Types.hpp"
#include "FTSensorCalibrator/FTSensorCalibrator.hpp"
#include <FTSensor/FTM115/FTM115.hpp>

using crf::robots::robotarmkinematics::RobotArmKDLKinematics;
using crf::robots::robotarm::RobotArmConfiguration;
using crf::algorithms::closedloopcontroller::PIDController;
using crf::applications::robotarmcontroller::IRobotArmController;
using crf::applications::robotarmcontroller::RobotArmSlaveController;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::areAlmostEqual;
using crf::sensors::ftsensor::IFTSensor;
using crf::sensors::ftsensor::FTM115;
using crf::applications::ftsensorcalibrator::FTSensorCalibrator;

// This program calibrates the Schunk Force Torque sensor,
// To initialize the Schunk CAN : ./../scripts/setcan.sh 0 1000000
// To initialize the FTSensor CAN: ./../scripts/setcan.sh 1 250000
int main(int argc, char* argv[]) {
    if (argc != 5) {
        std::cout << "Too few arguments" << std::endl;
        std::cout << "[1] Schunk Arm CAN interface " << std::endl;
        std::cout << "[2] FTSensor CAN interface " << std::endl;
        std::cout << "[3] SchunkArm CONFIG_FILENAME " << std::endl;
        std::cout << "[4] Calibration CONFIG_FILENAME " << std::endl;
        return -1;
    }

    // Initialize the Schunk Arm
    std::ifstream robotData(argv[3]);
    nlohmann::json configSchunk = nlohmann::json::parse(robotData);
    auto can_socket = std::make_shared<CANSocket>(argv[1]);
    auto arm = std::make_shared<crf::robots::schunkarm::SchunkArm>(
            can_socket, configSchunk);
    arm->initialize();
    auto robotArmConfiguration = std::make_shared<RobotArmConfiguration>();
    robotArmConfiguration->parse(configSchunk);
    auto kinematics = std::make_shared<RobotArmKDLKinematics>(robotArmConfiguration);
    auto closed_loop_controller = std::make_shared<PIDController>(
            std::vector<float>(6, .9),
            std::vector<float>(6, 0),
            std::vector<float>(6, 0));
    std::shared_ptr<IRobotArmController> ctrl = std::make_shared<RobotArmSlaveController>(
            arm, kinematics, closed_loop_controller);
    ctrl->initialize();


    // Initialize the force torque sensor
    auto can_socket2 = std::make_shared<CANSocket>(argv[2]);
    std::shared_ptr<IFTSensor> ftsensor = std::make_shared<FTM115>(can_socket2);
    ftsensor->initialize();

    // Calibrate the force torque sensor
    FTSensorCalibrator calib(ctrl, ftsensor, argv[4]);
    calib.initialize();
    calib.calibrate();

    // Move the robot into normal pick up position
    std::this_thread::sleep_for(std::chrono::seconds(5));
    TaskPose cp1({-0.5, 0, 0.5, 0, -M_PI / 2, 0});
    ctrl->setTaskPose(cp1);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    calib.validateCalibration();

    calib.deinitialize();
    ctrl->deinitialize();
    arm->deinitialize();

    return 0;
}
