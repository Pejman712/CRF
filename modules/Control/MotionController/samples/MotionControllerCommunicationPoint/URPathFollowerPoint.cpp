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
#include <csignal>

#include <boost/program_options.hpp>

#include "Robot/UniversalRobot/UniversalRobot.hpp"
#include "UniversalRobotRTDE/UniversalRobotRTDEInterface.hpp"
#include "TrajectoryGenerator/CubicTaskTrajectory/CubicTaskTrajectory.hpp"
#include "TrajectoryGenerator/PointToPointJointsTrajectory/PointToPointJointsTrajectory.hpp"
#include "Controller/PositionCtrlVelocityFF/PositionCtrlVelocityFF.hpp"

#include "InverseKinematics/DesiredJointPositions/DesiredJointPositions.hpp"
#include "InverseKinematics/JointLimits/JointLimits.hpp"
#include "InverseKinematics/OptCLIK/OptCLIK.hpp"
#include "MotionController/PathFollower/PathFollower.hpp"

#include "Sockets/TCP/TCPServer.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "MotionController/MotionControllerCommunicationPoint/MotionControllerCommunicationPoint.hpp"
#include "MotionController/MotionControllerCommunicationPoint/MotionControllerCommunicationPointFactory.hpp"
#include "MotionController/MotionControllerCommunicationPoint/MotionControllerManager.hpp"

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

    if (!vm.count("protocol") || !vm.count("port")) {
        std::cout << "Missing params" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    // Create UR
    std::ifstream robotConfigFilePath(vm["configuration"].as<std::string>());

    auto robot = std::make_shared<crf::actuators::robot::UniversalRobot>(
        std::make_shared<crf::communication::universalrobotrtde::UniversalRobotRTDEInterface>(),
        crf::actuators::robot::UniversalRobotConfiguration(
            nlohmann::json::parse(robotConfigFilePath)));

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

    if (!robot->initialize()) {
        std::puts("Failed to initialize the robot");
        return -1;
    }

    crf::utility::types::JointPositions homeJointsPosition = robot->getJointPositions().value();

    if (!robot->deinitialize()) {
        std::puts("Failed to deinitialize the robot");
        return -1;
    }

    std::shared_ptr<crf::control::inversekinematics::OptCLIK> inverseKinematics =
        std::make_shared<crf::control::inversekinematics::OptCLIK>(
            homeJointsPosition,
            static_cast<std::chrono::microseconds>(
                robot->getConfiguration()->getRobotControllerLoopTime()),
            robot->getConfiguration(),
            std::vector<double>({1, 1, 1, 1, 1, 1}),
            vecObjFun,
            TaskPose(
                {0.0001, 0.0001, 0.0001},
                crf::math::rotation::CardanXYZ({0.0001, 0.0001, 0.0001})),
            500,  // has to be less than 2/Ts
            0.01);

    std::vector<double> Kp = {1, 1, 1, 1, 1, 1};
    std::vector<double> Ki = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    std::vector<double> Kd = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    double Ts = 0.002;

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

    // Motion Communication Point
    std::shared_ptr<crf::control::motioncontroller::MotionControllerManager> manager(
        new crf::control::motioncontroller::MotionControllerManager(motion));

    std::shared_ptr<crf::control::motioncontroller::MotionControllerCommunicationPointFactory> communicationPointFactory( // NOLINT
        new crf::control::motioncontroller::MotionControllerCommunicationPointFactory(
            manager));

    std::unique_ptr<crf::communication::communicationpointserver::CommunicationPointServer>
        networkServer;

    std::string net_protocol = vm["protocol"].as<std::string>();
    int net_port = vm["port"].as<unsigned int>();

    if ((net_protocol != "tcp") || (net_port < 1) || (net_port > 65535)) {
        std::cout << "Wrong network parameters" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::shared_ptr<crf::communication::sockets::ISocketServer> server;
    if (net_protocol == "tcp") {
        server = std::make_shared<crf::communication::sockets::TCPServer>(net_port);
    }
    networkServer.reset(
        new crf::communication::communicationpointserver::CommunicationPointServer(server,
        communicationPointFactory));

    if (!networkServer->initialize()) {
        std::cout << "Failed to initialize network server, check logger for details" << std::endl;
        return -1;
    }

    std::cout << "Started network server on port " << net_port << std::endl;
    std::cout << "Communication point started correctly" << std::endl;
    std::cout << "Use Ctrl-Z for a smooth ending, or Ctrl-C to abort the execution" << std::endl;
    std::signal(SIGTSTP, signal_handler);
    while (gSignalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
