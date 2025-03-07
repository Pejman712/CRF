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

#include "CANopenDrivers/CiA402/CoEDrivers/ELMOGoldCoEDriver/ELMOGoldCoEDriver.hpp"
#include "CANopenDrivers/CiA402/CoEDrivers/EverestXCRCoEDriver/EverestXCRCoEDriver.hpp"
#include "Robot/CiA402Robot/CiA402Robot.hpp"

#include "Robot/UniversalRobot/UniversalRobot.hpp"
#include "UniversalRobotRTDE/UniversalRobotRTDEInterface.hpp"

#include "Robot/CombinedRobot/CombinedRobot.hpp"

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
    crf::actuators::robot::UniversalRobotConfiguration urConfig(configs[1]);

    std::vector<nlohmann::json> motorJSON = ciaConfig.getMotorConfigFiles();

    nlohmann::json masterConfig = motorJSON;

    // Create CiA402 Robot
    std::string ifname = vm["ethercat_port"].as<std::string>();

    std::shared_ptr<crf::devices::canopendrivers::CoEMaster> master =
        std::make_shared<crf::devices::canopendrivers::CoEMaster>(
            ifname, 1, std::chrono::milliseconds(1), masterConfig, 4096);

    if (!master->initialize()) {
        std::puts("Master initialization failed");
        return -1;
    }

    std::shared_ptr<crf::devices::canopendrivers::EverestXCRCoEDriver> driver1 =
        std::make_shared<crf::devices::canopendrivers::EverestXCRCoEDriver>(
            master, 1, motorJSON[0]);

    std::shared_ptr<crf::devices::canopendrivers::ELMOGoldCoEDriver> driver2 =
        std::make_shared<crf::devices::canopendrivers::ELMOGoldCoEDriver>(
            master, 2, motorJSON[1]);

    std::shared_ptr<crf::devices::canopendrivers::EverestXCRCoEDriver> driver3 =
        std::make_shared<crf::devices::canopendrivers::EverestXCRCoEDriver>(
            master, 3, motorJSON[2]);

    std::vector<std::shared_ptr<crf::devices::canopendrivers::ICiA402Driver>> vectorDrivers =
        {driver1, driver2, driver3};

    std::shared_ptr<crf::actuators::robot::IRobot> cia402Robot =
        std::make_shared<crf::actuators::robot::CiA402Robot>(
            vectorDrivers, ciaConfig);

    // Create UR Robot
    std::shared_ptr<crf::actuators::robot::UniversalRobot> urRobot =
        std::make_shared<crf::actuators::robot::UniversalRobot>(
            std::make_shared<crf::communication::universalrobotrtde::UniversalRobotRTDEInterface>(),
            urConfig);

    // Create Combined robot

    std::vector<std::shared_ptr<crf::actuators::robot::IRobot>> vectorRobots =
        {cia402Robot, urRobot};

    auto robot = std::make_shared<crf::actuators::robot::CombinedRobot>(
        vectorRobots, configCombined);

    // Get Forward Kinematics
    std::shared_ptr<crf::control::forwardkinematics::IForwardKinematics> forwardKinematics =
        robot->getConfiguration()->getForwardKinematics();

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
    std::shared_ptr<crf::control::inversekinematics::OptOLIK> inverseKinematics =
        std::make_shared<crf::control::inversekinematics::OptOLIK>(
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
