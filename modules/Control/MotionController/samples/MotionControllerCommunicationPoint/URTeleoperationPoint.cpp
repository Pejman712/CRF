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
#include <boost/program_options.hpp>
#include <csignal>

#include "Robot/UniversalRobot/UniversalRobot.hpp"
#include "UniversalRobotRTDE/UniversalRobotRTDEInterface.hpp"
#include "Controller/DirectOpenLoopVelocity/DirectOpenLoopVelocity.hpp"
#include "ForwardKinematics/MathExprForwardKinematics/MathExprForwardKinematics.hpp"
#include "InverseKinematics/OptOLIK/OptOLIK.hpp"
#include "MotionController/Teleoperation/Teleoperation.hpp"
#include "InverseKinematics/JointLimits/JointLimits.hpp"
#include "InverseKinematics/DesiredJointPositions/DesiredJointPositions.hpp"

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

    // Create UR Robot
    std::ifstream robotConfigFilePath(vm["configuration"].as<std::string>());

    auto robot = std::make_shared<crf::actuators::robot::UniversalRobot>(
        std::make_shared<crf::communication::universalrobotrtde::UniversalRobotRTDEInterface>(),
        crf::actuators::robot::UniversalRobotConfiguration(
            nlohmann::json::parse(robotConfigFilePath)));

    // Create Forward Kinematics
    auto forwardKinematics = robot->getConfiguration()->getForwardKinematics();

    // Create Objective Functions
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>>
        vecObjFun;

    // Joint limits
    vecObjFun.push_back(std::make_shared<crf::control::inversekinematics::JointLimits>(
        10.0,  // 10s for the range of the sinusoid
        static_cast<double>(robot->getConfiguration()->getRobotControllerLoopTime().count())/1000,
        1.0,  // Unitary exponential function
        0.1,  // Low proportional gain
        robot->getConfiguration()->getJointLimits().minPosition,
        robot->getConfiguration()->getJointLimits().maxPosition));

    // Desired Joint Position
    vecObjFun.push_back(std::make_shared<crf::control::inversekinematics::DesiredJointPositions>(
        10,  // 10s for the range of the sinusoid
        static_cast<double>(robot->getConfiguration()->getRobotControllerLoopTime().count())/1000,
        1));  // Unitary exponential function

    // Inverse Kinematics
    auto inverseKinematics = std::make_shared<crf::control::inversekinematics::OptOLIK>(
        robot->getConfiguration(),
        std::vector<double>(robot->getConfiguration()->getJointSpaceDoF(), 1),
        vecObjFun,
        0.01);

    // Controller
    auto controller = std::make_shared<crf::control::controller::DirectOpenLoopVelocity>(
        robot->getConfiguration()->getJointSpaceDoF(), inverseKinematics, vecObjFun);

    // Motion controller
    auto motion = std::make_shared<crf::control::motioncontroller::Teleoperation>(
        robot, controller);

    // Motion Communication Point
    auto manager = std::make_shared<crf::control::motioncontroller::MotionControllerManager>(motion);  // NOLINT
    auto communicationPointFactory = std::make_shared<crf::control::motioncontroller::MotionControllerCommunicationPointFactory>(manager);  // NOLINT
    std::unique_ptr<crf::communication::communicationpointserver::CommunicationPointServer> networkServer;  // NOLINT

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
        std::cout << "Failed to initialize network server, check logger for details" << std::endl;  // NOLINT
        return -1;
    }

    std::cout << "Started network server on port " << net_port << std::endl;

    std::signal(SIGTSTP, signal_handler);
    std::cout << "Communication point started correctly\n";
    std::cout << "Use Ctrl-Z for a smooth ending, or Ctrl-C to abort the execution\n";
    while (gSignalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
