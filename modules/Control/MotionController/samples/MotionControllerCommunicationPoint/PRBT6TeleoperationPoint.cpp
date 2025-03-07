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

#include <nlohmann/json.hpp>
#include <boost/program_options.hpp>

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/loop_driver.hpp>

#include "CANopenDrivers/CANopenMaster/CANopenMaster.hpp"
#include "CANopenDrivers/CiA402/CANDrivers/ERB415CANDriver/ERB415CANDriver.hpp"
#include "Robot/CiA402Robot/CiA402Robot.hpp"
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

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("master_dcf", po::value<std::string>(), "Master DCF path")
        ("can_port", po::value<std::string>(), "CAN port (e.g. can0, can1, ...)")
        ("configuration", po::value<std::string>(), "Configuration file path for the arm")
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp) [Required if port is set]")  // NOLINT
        ("port", po::value<uint64_t>(), "Network port [1-65535] [Required if protocol is set]");

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

    if (!vm.count("configuration") || !vm.count("can_port") || !vm.count("master_dcf")) {
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
    std::string canPort = vm["can_port"].as<std::string>();
    uint64_t port = vm["port"].as<uint64_t>();
    std::string protocol = vm["protocol"].as<std::string>();
    std::string masterDCF = vm["master_dcf"].as<std::string>();

    crf::actuators::robot::CiA402RobotConfiguration configuration(
        nlohmann::json::parse(robotConfigFilePath));

    lely::io::IoGuard io_guard;
    lely::io::Context ctx;
    lely::io::Poll poll(ctx);
    lely::ev::Loop loop(poll.get_poll());
    auto exec = loop.get_executor();
    lely::io::Timer timer(poll, exec, CLOCK_MONOTONIC);
    lely::io::CanController ctrl(canPort.c_str());
    lely::io::CanChannel chan(poll, exec);
    chan.open(ctrl);

    std::shared_ptr<crf::devices::canopendrivers::CANopenMaster> master =
        std::make_shared<crf::devices::canopendrivers::CANopenMaster>(
            timer, chan, masterDCF, "", 1);

    std::vector<std::shared_ptr<crf::devices::canopendrivers::ICiA402Driver>> drivers;

    for (uint64_t i = 0; i < configuration.getNumberOfMotors(); i++) {
        drivers.push_back(std::make_shared<crf::devices::canopendrivers::ERB415CANDriver>(
            master, i + 3, configuration.getMotorConfigFiles()[i]));
    }

    auto robot = std::make_shared<crf::actuators::robot::CiA402Robot>(drivers, configuration);

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

    std::string net_protocol = protocol;
    int net_port = port;

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
    std::cout << "Communication point started correctly\n";
    std::cout << "Use Ctrl-C for a smooth ending, click it twice to abort the execution\n";

    lely::io::SignalSet sigset(poll, exec);

    // Watch for Ctrl+C or process termination.
    sigset.insert(SIGHUP);
    sigset.insert(SIGINT);
    sigset.insert(SIGTERM);

    // Submit a task to be executed when a signal is raised. We don't care which.
    sigset.submit_wait([&](int /*signo*/) {
        sigset.clear();
        master->AsyncDeconfig().submit(exec, [&]() {
            ctx.shutdown();
        });
    });

    master->Reset();
    loop.run();
}
