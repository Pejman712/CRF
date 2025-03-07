/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <csignal>
#include <iostream>
#include <memory>
#include <vector>

#include <boost/program_options.hpp>

#include "CANopenDrivers/CiA402/CoEDrivers/ELMOGoldCoEDriver/ELMOGoldCoEDriver.hpp"

#include "KinovaJacoAPI/KinovaJacoAPIInterface.hpp"

#include "Robot/CiA402Robot/CiA402Robot.hpp"
#include "Robot/CombinedRobot/CombinedRobot.hpp"
#include "Robot/KinovaJaco2/KinovaJaco2.hpp"
#include "TrajectoryGenerator/CubicTaskTrajectory/CubicTaskTrajectory.hpp"
#include "TrajectoryGenerator/PointToPointJointsTrajectory/PointToPointJointsTrajectory.hpp"

#include "Controller/PositionCtrlVelocityFF/PositionCtrlVelocityFF.hpp"
#include "ForwardKinematics/MathExprForwardKinematics/MathExprForwardKinematics.hpp"
#include "InverseKinematics/DesiredJointPositions/DesiredJointPositions.hpp"
#include "InverseKinematics/JointLimits/JointLimits.hpp"
#include "InverseKinematics/OptCLIK/OptCLIK.hpp"
#include "MotionController/PathFollower/PathFollower.hpp"

using crf::utility::types::TaskPose;
using crf::math::rotation::CardanXYZ;

namespace po = boost::program_options;

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    std::cout << "Caught signal Ctrl-Z: " << signal << std::endl;
    gSignalStatus = signal;
}
}  // unnamed namespace

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("configuration", po::value<std::string>(), "Configuration file path for the arm")
        ("ethercat_port", po::value<std::string>(), "EtherCAT Port")
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp) [Required if port is set]")  // NOLINT
        ("port", po::value<unsigned int>(), "Network port [1-65535] [Required if protocol is set]");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    } catch (std::exception&) {
        std::cout << "Bad command line values" << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }

    if (!vm.count("configuration")) {
        std::cout << "Missing configuration file" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("protocol") || !vm.count("port") || !vm.count("ethercat_port")) {
        std::cout << "Missing params" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    // Manage configs
    std::string configFile = vm["configuration"].as<std::string>();
    nlohmann::json config = nlohmann::json::parse(std::ifstream(configFile));
    crf::actuators::robot::CombinedRobotConfiguration configCombined(config);

    if (configCombined.getNumberOfRobots() != 2) {
        std::puts("Incorrect number of robots in config file");
        return -1;
    }

    std::vector<nlohmann::json> configs = configCombined.getRobotConfigFiles();

    crf::actuators::robot::CiA402RobotConfiguration ciaConfig(configs[0]);

    std::vector<nlohmann::json> motorJSON = ciaConfig.getMotorConfigFiles();

    nlohmann::json masterConfig = motorJSON;

    // Create CiA402 Robot
    std::string ifname = vm["ethercat_port"].as<std::string>();

    std::shared_ptr<crf::devices::canopendrivers::CoEMaster> master =
        std::make_shared<crf::devices::canopendrivers::CoEMaster>(
            ifname, 4, std::chrono::microseconds(250), masterConfig, 4096);

    if (!master->initialize()) {
        std::puts("Master initialization failed");
        return -1;
    }

    std::shared_ptr<crf::devices::canopendrivers::ELMOGoldCoEDriver> driver2 =
        std::make_shared<crf::devices::canopendrivers::ELMOGoldCoEDriver>(master, 1, motorJSON[0]);

    std::shared_ptr<crf::devices::canopendrivers::ELMOGoldCoEDriver> driver1 =
        std::make_shared<crf::devices::canopendrivers::ELMOGoldCoEDriver>(master, 2, motorJSON[1]);

    std::shared_ptr<crf::devices::canopendrivers::ELMOGoldCoEDriver> driver3 =
        std::make_shared<crf::devices::canopendrivers::ELMOGoldCoEDriver>(master, 3, motorJSON[2]);

    std::shared_ptr<crf::devices::canopendrivers::ELMOGoldCoEDriver> driver4 =
        std::make_shared<crf::devices::canopendrivers::ELMOGoldCoEDriver>(master, 4, motorJSON[3]);

    std::vector<std::shared_ptr<crf::devices::canopendrivers::ICiA402Driver>> vectorDrivers = {
        driver1, driver2, driver3, driver4};

    std::shared_ptr<crf::actuators::robot::IRobot> cia402Robot =
        std::make_shared<crf::actuators::robot::CiA402Robot>(vectorDrivers, ciaConfig);

    // Create Kinova Jaco 2 Robot
    std::shared_ptr<crf::actuators::robot::KinovaJaco2> kinovaRobot =
        std::make_shared<crf::actuators::robot::KinovaJaco2>(
            std::make_shared<crf::communication::kinovajacoapi::KinovaJacoAPIInterface>(),
            crf::actuators::robot::KinovaJaco2Configuration(configs[1]));

    // Create Combined robot
    std::vector<std::shared_ptr<crf::actuators::robot::IRobot>> vectorRobots = {
        cia402Robot, kinovaRobot};

    std::shared_ptr<crf::actuators::robot::CombinedRobot> robot =
        std::make_shared<crf::actuators::robot::CombinedRobot>(vectorRobots, configCombined);

    auto robotConfig = robot->getConfiguration();

    // Create Joint Space Trajectory Generator
    std::shared_ptr<crf::control::trajectorygenerator::PointToPointJointsTrajectory>
        jointTrajGenerator =
            std::make_shared<crf::control::trajectorygenerator::PointToPointJointsTrajectory>(
                robotConfig->getProfileParameters().jointVelocities,
                robotConfig->getProfileParameters().jointAccelerations);

    // Create Task Space Trajectory Generator
    std::shared_ptr<crf::control::trajectorygenerator::CubicTaskTrajectory> taskTrajGenerator =
        std::make_shared<crf::control::trajectorygenerator::CubicTaskTrajectory>(
            robotConfig->getProfileParameters().taskVelocity,
            robotConfig->getProfileParameters().taskAcceleration);

    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>>
        vecObjFun;
    crf::utility::types::JointPositions homeJointsPosition({0, 0, 0, 0, 0, 0, 0, 0, 0, 0});

    std::shared_ptr<crf::control::inversekinematics::OptCLIK> inverseKinematics =
        std::make_shared<crf::control::inversekinematics::OptCLIK>(
            homeJointsPosition,
            static_cast<std::chrono::microseconds>(
                robot->getConfiguration()->getRobotControllerLoopTime()),
            robot->getConfiguration(),
            std::vector<double>({1e8, 1e8, 1e8, 1e8, 1, 1, 1, 1, 1, 1}),
            vecObjFun,
            TaskPose({0.0001, 0.0001, 0.0001}, crf::math::rotation::CardanXYZ({0, 0, 0})),
            200,
            0.01);

    std::vector<double> Kp = {1, 1, 1, 1, 1, 1};
    std::vector<double> Ki = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    std::vector<double> Kd = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    double Ts = 2.0;

    std::shared_ptr<crf::control::controller::PositionCtrlVelocityFF> controller =
        std::make_shared<crf::control::controller::PositionCtrlVelocityFF>(
            Kp, Ki, Kd, Ts, inverseKinematics);

    // Motion controller
    std::shared_ptr<crf::control::motioncontroller::PathFollower> motion =
        std::make_shared<crf::control::motioncontroller::PathFollower>(
            robot,
            controller,
            jointTrajGenerator,
            taskTrajGenerator);

    if (!motion->initialize()) {
        std::puts("Could not initialize controller");
        return -1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));

    motion->appendPath(
        {TaskPose({0, 0, -0.2}, CardanXYZ({0, 0, 0}))},
        crf::control::motioncontroller::TrajectoryExecutionMethod::TaskSpace,
        crf::control::motioncontroller::PointReferenceFrame::TCP);

    std::this_thread::sleep_for(std::chrono::seconds(500));
}
