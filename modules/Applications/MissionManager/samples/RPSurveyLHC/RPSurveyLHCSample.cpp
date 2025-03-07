/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO 2022
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

#include <boost/program_options.hpp>
#include <nlohmann/json.hpp>

#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "Sockets/TCP/TCPSocket.hpp"
#include "Sockets/TCP/TCPServer.hpp"

#include "TIM/TIMMockConfiguration.hpp"
#include "TIM/TIMCommunicationPoint/TIMManager.hpp"
#include "TIM/TIMCommunicationPoint/TIMCommunicationPointFactory.hpp"
#include "TIM/TIMClient/TIMClient.hpp"

#include "RPSensor/RPSensorMockConfiguration.hpp"
#include "RPSensor/RPSensorCommunicationPoint/RPSensorManager.hpp"
#include "RPSensor/RPSensorCommunicationPoint/RPSensorCommunicationPointFactory.hpp"
#include "RPSensor/RPSensorClient/RPSensorClient.hpp"

#include "MissionManager/RPSurveyLHC/RPSurveyLHC.hpp"
#include "MissionManager/MissionManagerCommunicationPoint/MissionManagerCommunicationPoint.hpp"
#include "MissionManager/MissionManagerCommunicationPoint/MissionManagerCommunicationPointFactory.hpp"

#include "TIMRPWagon/TIMRPWagonMockConfiguration.hpp"
#include "TIMRPWagon/TIMRPWagonCommunicationPoint/TIMRPWagonManager.hpp"
#include "TIMRPWagon/TIMRPWagonCommunicationPoint/TIMRPWagonCommunicationPointFactory.hpp"
#include "TIMRPWagon/TIMRPWagonClient/TIMRPWagonClient.hpp"

#include "RobotArm/RobotArmMockConfiguration.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerManager.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerCommunicationPointFactory.hpp"
#include "RobotArmController/RobotArmControllerClient/RobotArmControllerClient.hpp"

#include "MissionUtility/DeployableRobotArm/DeployableRobotArm.hpp"
#include "MissionUtility/DeployableTIMRPWagonArm/DeployableTIMRPWagonArm.hpp"

using crf::communication::sockets::TCPServer;
using crf::communication::communicationpointserver::CommunicationPointServer;
using crf::communication::sockets::TCPSocket;
using crf::communication::datapacketsocket::PacketSocket;

using crf::actuators::tim::TIMMockConfiguration;
using crf::actuators::tim::TIMManager;
using crf::actuators::tim::TIMCommunicationPointFactory;
using crf::actuators::tim::TIMClient;

using crf::actuators::timrpwagon::TIMRPWagonMockConfiguration;
using crf::actuators::timrpwagon::TIMRPWagonManager;
using crf::actuators::timrpwagon::TIMRPWagonCommunicationPointFactory;
using crf::actuators::timrpwagon::TIMRPWagonClient;

using crf::actuators::robotarm::RobotArmMockConfiguration;
using crf::control::robotarmcontroller::RobotArmControllerManager;
using crf::control::robotarmcontroller::RobotArmControllerCommunicationPointFactory;
using crf::control::robotarmcontroller::RobotArmControllerClient;

using crf::sensors::rpsensor::RPSensorMockConfiguration;
using crf::sensors::rpsensor::RPSensorManager;
using crf::sensors::rpsensor::RPSensorCommunicationPointFactory;
using crf::sensors::rpsensor::RPSensorClient;

using crf::utility::missionutility::DeployableTIMRPWagonArm;
using crf::utility::missionutility::DeployableRobotArm;
using crf::utility::missionutility::IDeployableDevice;

namespace {
volatile std::sig_atomic_t signalStatus;
void signalHandler(int signal) {
    std::cout << ": Stop signal [SIGTSTP] received" << std::endl;
    signalStatus = signal;
}
}  // unnamed namespace

namespace po = boost::program_options;

int main(int argc, char *argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("parametersPath", po::value<std::string>(), "Parameters for the mission [config/BLMCernBot/missionParameters.json].");  // NOLINT

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    } catch (std::exception&) {
        std::cout << "Bad command line values." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }
    if (!vm.count("parametersPath")) {
        std::cout << "Missing parameters." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    // Extract Communication data
    std::ifstream parametersFile(vm["parametersPath"].as<std::string>());
    nlohmann::json config = nlohmann::json::parse(parametersFile);
    unsigned int priority = config["Communications"]["Priority"].get<unsigned int>();
    std::chrono::milliseconds serverReplyTimeout(20000);
    float streamerFrequency = 0;  // Status received when requested

    unsigned int missionPort = 5000;

    unsigned int timPort = config["Communications"]["TIM"]["Port"].get<unsigned int>();
    std::string timHost = config["Communications"]["TIM"]["Host"].get<std::string>();

    unsigned int rpsensorPort = config["Communications"]["RPSensor"]["Port"].get<unsigned int>();
    std::string rpsensorHost = config["Communications"]["RPSensor"]["Host"].get<std::string>();

    unsigned int robotArmPort;
    std::string robotArmHost;
    bool hasTIMRPWagon = false;
    if (config["HardwareConfiguration"] == "TIMRPWagon") {
        robotArmPort = config["Communications"]["TIMRPWagon"]["Port"].get<unsigned int>();
        robotArmHost = config["Communications"]["TIMRPWagon"]["Host"].get<std::string>();
        hasTIMRPWagon = true;
    } else if (config["HardwareConfiguration"] == "RobotArmWagon") {
        robotArmPort = config["Communications"]["TIMArm"]["Port"].get<unsigned int>();
        robotArmHost = config["Communications"]["TIMArm"]["Host"].get<std::string>();
    } else {
        std::cout << "The chosen Hardware Configuration is not valid for a comm point" << std::endl;
        return -1;
    }

    //////////////////////////
    // Communication Points //
    //////////////////////////

    // Launch TIM Point
    crf::actuators::tim::TIMMockConfiguration timMock;
    timMock.TIMAddObstacle(crf::actuators::tim::LHCObstacle(1,
        crf::actuators::tim::LHCObstacleType::NotDefined, 100, 120, 0.5, true));
    timMock.TIMAddObstacle(crf::actuators::tim::LHCObstacle(2,
        crf::actuators::tim::LHCObstacleType::NotDefined, 130, 140, 0.5, true));
    timMock.TIMAddObstacle(crf::actuators::tim::LHCObstacle(3,
        crf::actuators::tim::LHCObstacleType::NotDefined, 200, 210, 0.5, false));
    timMock.TIMAddObstacle(crf::actuators::tim::LHCObstacle(4,
        crf::actuators::tim::LHCObstacleType::NotDefined, 220, 230, 0.5, true));
    timMock.TIMAddObstacle(crf::actuators::tim::LHCObstacle(5,
        crf::actuators::tim::LHCObstacleType::NotDefined, 300, 310, 0.5, true));
    timMock.TIMAddObstacle(crf::actuators::tim::LHCObstacle(6,
        crf::actuators::tim::LHCObstacleType::NotDefined, 320, 330, 0.5, false));
    timMock.configureTIMMock();
    std::shared_ptr<TIMManager> managerTim = std::make_shared<TIMManager>(timMock.getMock());
    std::shared_ptr<TCPServer> serverSocketTim = std::make_shared<TCPServer>(timPort);
    std::shared_ptr<TIMCommunicationPointFactory> factoryTim =
        std::make_shared<TIMCommunicationPointFactory>(managerTim);
    std::unique_ptr<CommunicationPointServer> serverTIM =
        std::make_unique<CommunicationPointServer>(serverSocketTim, factoryTim);
    if (!serverTIM->initialize()) {
        std::puts("Failed to initialize network server for TIM");
        return -1;
    }

    // Launch RobotArm Point
    std::unique_ptr<CommunicationPointServer> serverArm;
    if (hasTIMRPWagon) {
        // RP Wagon Arm Communication Point
        crf::actuators::timrpwagon::TIMRPWagonMockConfiguration rpArmMock;
        rpArmMock.configureTIMRPWagonMock();
        std::shared_ptr<TIMRPWagonManager> managerRPArm =
            std::make_shared<TIMRPWagonManager>(rpArmMock.getMock());
        std::shared_ptr<TCPServer> serverSocketArm = std::make_shared<TCPServer>(robotArmPort);
        std::shared_ptr<TIMRPWagonCommunicationPointFactory> factoryArm =
            std::make_shared<TIMRPWagonCommunicationPointFactory>(managerRPArm);
        serverArm = std::make_unique<CommunicationPointServer>(serverSocketArm, factoryArm);
        if (!serverArm->initialize()) {
            std::puts("Failed to initialize network server for TIM RP Wagon");
            return -1;
        }
    } else {
        // TIM Arm Controller Communication Point
        crf::actuators::robotarm::RobotArmMockConfiguration timArmMock(9);
        timArmMock.configureRobotArmMock();
        std::shared_ptr<TCPServer> serverSocketArm = std::make_shared<TCPServer>(robotArmPort);
        std::shared_ptr<RobotArmControllerManager> managerTIMArm =
            std::make_shared<RobotArmControllerManager>(timArmMock.getMock());
        std::shared_ptr<RobotArmControllerCommunicationPointFactory> factoryArm =
            std::make_shared<RobotArmControllerCommunicationPointFactory>(managerTIMArm);
        serverArm = std::make_unique<CommunicationPointServer>(serverSocketArm, factoryArm);
        if (!serverArm->initialize()) {
            std::puts("Failed to initialize network server for TIM Arm Wagon");
            return -1;
        }
    }

    // Launch RPSensor Point
    std::shared_ptr<crf::sensors::rpsensor::RPSensorMockConfiguration> rpSensorMock =
        std::make_shared<crf::sensors::rpsensor::RPSensorMockConfiguration>();
    rpSensorMock->configureMock();
    std::shared_ptr<RPSensorManager> managerRPSensor =
        std::make_shared<RPSensorManager>(rpSensorMock);
    std::shared_ptr<TCPServer> serverSocketRPSensor = std::make_shared<TCPServer>(rpsensorPort);
    std::shared_ptr<RPSensorCommunicationPointFactory> factoryRPSensor =
        std::make_shared<RPSensorCommunicationPointFactory>(managerRPSensor);
    std::unique_ptr<CommunicationPointServer> serverRPSensor =
        std::make_unique<CommunicationPointServer>(serverSocketRPSensor, factoryRPSensor);
    if (!serverRPSensor->initialize()) {
        std::puts("Failed to initialize network server for RP Sensor");
        return -1;
    }

    /////////////
    // Clients //
    /////////////

    // Deployable Device Client
    std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable;
    if (hasTIMRPWagon) {
        auto clientSocketArm = std::make_shared<crf::communication::sockets::TCPSocket>(
            robotArmHost, robotArmPort);
        auto socketArm = std::make_shared<crf::communication::datapacketsocket::PacketSocket>(
            clientSocketArm);
        auto timrpwagon = std::make_shared<crf::actuators::timrpwagon::TIMRPWagonClient>(
            socketArm, serverReplyTimeout, streamerFrequency, priority);
        deployable = std::make_shared<crf::utility::missionutility::DeployableTIMRPWagonArm>(
            timrpwagon);
    } else {
        auto clientSocketArm = std::make_shared<crf::communication::sockets::TCPSocket>(
            robotArmHost, robotArmPort);
        auto socketArm = std::make_shared<crf::communication::datapacketsocket::PacketSocket>(
            clientSocketArm);
        auto contr = std::make_shared<crf::control::robotarmcontroller::RobotArmControllerClient>(
            socketArm, serverReplyTimeout, streamerFrequency, priority);
        std::string direc = __FILE__;
        direc = direc.substr(0, direc.find("cpproboticframework"));
        std::ifstream traj(direc + config["Movement"]["RobotArmTrajectories"].get<std::string>());
        deployable = std::make_shared<crf::utility::missionutility::DeployableRobotArm>(
            contr, nlohmann::json::parse(traj));
    }

    // TIM Client
    auto clientSocketTIM = std::make_shared<crf::communication::sockets::TCPSocket>(
        timHost, timPort);
    auto socketTIM = std::make_shared<crf::communication::datapacketsocket::PacketSocket>(
        clientSocketTIM);
    auto tim = std::make_shared<crf::actuators::tim::TIMClient>(
        socketTIM, serverReplyTimeout, streamerFrequency, priority);

    // RP Sensor Client
    auto clientSocketRPSensor = std::make_shared<crf::communication::sockets::TCPSocket>(
        rpsensorHost, rpsensorPort);
    auto socketRPSensor = std::make_shared<crf::communication::datapacketsocket::PacketSocket>(
        clientSocketRPSensor);
    auto rpSensor = std::make_shared<crf::sensors::rpsensor::RPSensorClient>(
        socketRPSensor, serverReplyTimeout, streamerFrequency, priority);

    /////////////
    // Mission //
    /////////////

    // Mission Communication Point
    auto mission = std::make_shared<crf::applications::missionmanager::rpsurveylhc::RPSurveyLHC>(
        tim, deployable, rpSensor, config);

    std::cout << "All objects created, starting Mission Manager communication point" << std::endl;

    auto comPointFactory = std::make_shared<crf::applications::missionmanager::MissionManagerCommunicationPointFactory>(  // NOLINT
        mission);
    auto socketServerMission = std::make_shared<crf::communication::sockets::TCPServer>(
        missionPort);
    auto commPointServer = std::make_unique<crf::communication::communicationpointserver::CommunicationPointServer>(  // NOLINT
        socketServerMission, comPointFactory);

    if (!commPointServer->initialize()) {
        std::cout << "Failed to initialize network server, check logger for details" << std::endl;
        return -1;
    }
    std::cout << "Mission Manager communication point started - port " << missionPort << std::endl;

    std::signal(SIGTSTP, signalHandler);
    while (signalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[Ctrl-Z] Detected" << std::endl;
    return 0;
}
