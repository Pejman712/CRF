/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */
#include <string>
#include <csignal>
#include <fstream>
#include <stdio.h>
#include <memory>
#include <vector>

#include <boost/program_options.hpp>

#include "CANSocket/CANSocket.hpp"
#include "CHARMBot/CHARMBot.hpp"

#include "Sockets/Tcp/TcpServer.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"

#include "StateEstimator/kalman/UnscentedKalmanFilter.hpp"
#include "RobotBaseController/RobotBaseControllerCommunicationPoint/RobotBaseControllerCommunicationPointFactory.hpp"
#include "RobotBaseController/RobotBaseControllerCommunicationPoint/RobotBaseControllerManager.hpp"
#include "RobotPoseEstimators/RobotPoseEstimatorTrackingCam.hpp"
#include "TrackingCamera/IntelT265Grabber.hpp"

#define STANDARD_BAUDRATE 115200

namespace po = boost::program_options;

using crf::robots::robotbase::CHARMBot;
using crf::applications::robotposeestimator::RobotPoseEstimatorTrackingCam;

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    gSignalStatus = signal;
}
}  // unnamed namespace

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("configuration", po::value<std::string>(), "Configuration file path for the CERNBot Base")
        ("can_port", po::value<std::string>(), "Can port name (e.g. can0)")
        ("port", po::value<unsigned int>(), "Network port [1-65535]")
        ("tracking_cam_config", po::value<std::string>(), "Configuration file path for the tracking camera");  // NOLINT

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
    if (!vm.count("port")) {
        std::cout << "Missing port to host the communication point" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    if (!vm.count("can_port")) {
        std::cout << "Missing can port" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    int net_port = vm["port"].as<unsigned int>();
    if ((net_port < 1) || (net_port > 65535)) {
        std::cout << "Wrong network parameters" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    std::shared_ptr<CANSocket> canSocket = std::make_shared<CANSocket>(
        reinterpret_cast<const char*>(vm["can_port"].as<std::string>().c_str()));
    std::ifstream robotData(vm["configuration"].as<std::string>());
    nlohmann::json baseJSON = nlohmann::json::parse(robotData);

    std::shared_ptr<crf::robots::robotbase::CHARMBot> base =
        std::make_shared<crf::robots::robotbase::CHARMBot>(
            canSocket,
            baseJSON);

    if (!base->initialize()) {
        std::cout << "Could not initialize the arm" << std::endl;
        return -1;
    }

    std::shared_ptr<crf::robots::robotbasecontroller::RobotBaseControllerManager> manager(
        new crf::robots::robotbasecontroller::RobotBaseControllerManager(base));

    std::shared_ptr<crf::robots::robotbasecontroller::RobotBaseControllerCommunicationPointFactory> communicationPointFactory( // NOLINT
        new crf::robots::robotbasecontroller::RobotBaseControllerCommunicationPointFactory(
            manager));

    std::unique_ptr<crf::communication::sockets::CommunicationPointServer> networkServer;

    std::shared_ptr<crf::communication::sockets::ISocketServer> server =
        std::make_shared<crf::communication::sockets::TcpServer>(net_port);
    networkServer.reset(new crf::communication::sockets::CommunicationPointServer(
        server,
        communicationPointFactory));
    if (!networkServer->initialize()) {
        std::cout << "Failed to initialize network server, check logger for details" << std::endl;  // NOLINT
        return -1;
    }

    std::cout << "CHARMBot started correctly\n";

    std::shared_ptr<crf::applications::robotposeestimator::IRobotPoseEstimator> localizer_;
    if (!vm.count("tracking_cam_config")) {
        std::cout << "Missing tracking camera config" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::ifstream configCam(vm["tracking_cam_config"].as<std::string>());
    nlohmann::json jConfigCam;
    configCam >> jConfigCam;

    localizer_ =
        std::make_shared<crf::applications::robotposeestimator::RobotPoseEstimatorTrackingCam>
            (std::make_shared<Kalman::UnscentedKalmanFilter<crf::applications::
                robotposeestimator::SystemState>>(1), base,
                std::make_shared<crf::sensors::trackingcamera::IntelT265Grabber>(
        jConfigCam.at("RealSense")));

    if (!localizer_->initialize()) {
        std::cout << "Could not initialize the RobotPositionLocalizer" << std::endl;
        return -1;
    }
    sleep(1);
    if (!localizer_->getPosition()) {
        std::cout << "Could create RobotPositionLocalizer" << std::endl;
        return -1;
    }
    std::signal(SIGTSTP, signal_handler);
    while (gSignalStatus != SIGTSTP) {
        auto pose = localizer_->getPosition().get();
            std::cout << pose(0) << pose(1) << pose(2) <<
                pose(3) << pose(4) << pose(5) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "CHARMBot process clean up\n";
    return 0;
}
