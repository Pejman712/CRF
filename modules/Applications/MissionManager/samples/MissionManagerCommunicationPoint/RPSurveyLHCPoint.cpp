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

#include "RobotArmController/RobotArmControllerClient/RobotArmControllerClient.hpp"
#include "TIMRPWagon/TIMRPWagonClient/TIMRPWagonClient.hpp"
#include "TIM/TIMClient/TIMClient.hpp"
#include "RPSensor/RPSensorClient/RPSensorClient.hpp"

#include "MissionManager/RPSurveyLHC/RPSurveyLHC.hpp"
#include "MissionManager/MissionManagerCommunicationPoint/MissionManagerCommunicationPoint.hpp"
#include "MissionManager/MissionManagerCommunicationPoint/MissionManagerCommunicationPointFactory.hpp"

#include "MissionUtility/DeployableRobotArm/DeployableRobotArm.hpp"
#include "MissionUtility/DeployableTIMRPWagonArm/DeployableTIMRPWagonArm.hpp"

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
        ("missionPort", po::value<unsigned int>(), "Port for the Mission to be launched [1-65535].")  // NOLINT
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
    if (!vm.count("missionPort") || !vm.count("parametersPath")) {
        std::cout << "Missing parameters." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    unsigned int missionPort = vm["missionPort"].as<unsigned int>();
    std::ifstream parametersFile(vm["parametersPath"].as<std::string>());
    nlohmann::json config = nlohmann::json::parse(parametersFile);
    unsigned int priority = config["Communications"]["Priority"].get<unsigned int>();
    std::chrono::milliseconds serverReplyTimeout(20000);
    float streamerFrequency = 0;  // Status received when requested

    // Deployable Device Client
    std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable;
    if (config["HardwareConfiguration"] == "RobotArmWagon") {
        auto clientSocketArm = std::make_shared<crf::communication::sockets::TCPSocket>(
            config["Communications"]["TIMArm"]["Host"].get<std::string>(),
            config["Communications"]["TIMArm"]["Port"].get<unsigned int>());
        auto socketArm = std::make_shared<crf::communication::datapacketsocket::PacketSocket>(
            clientSocketArm);
        auto contr = std::make_shared<crf::control::robotarmcontroller::RobotArmControllerClient>(
            socketArm, serverReplyTimeout, streamerFrequency, priority);
        std::string direc = __FILE__;
        direc = direc.substr(0, direc.find("cpproboticframework"));
        std::ifstream traj(direc + config["Movement"]["RobotArmTrajectories"].get<std::string>());
        deployable = std::make_shared<crf::utility::missionutility::DeployableRobotArm>(
            contr, nlohmann::json::parse(traj));
    } else if (config["HardwareConfiguration"] == "TIMRPWagon") {
        auto clientSocketArm = std::make_shared<crf::communication::sockets::TCPSocket>(
            config["Communications"]["TIMRPWagon"]["Host"].get<std::string>(),
            config["Communications"]["TIMRPWagon"]["Port"].get<unsigned int>());
        auto socketArm = std::make_shared<crf::communication::datapacketsocket::PacketSocket>(
            clientSocketArm);
        auto timrpwagon = std::make_shared<crf::actuators::timrpwagon::TIMRPWagonClient>(
            socketArm, serverReplyTimeout, streamerFrequency, priority);
        deployable = std::make_shared<crf::utility::missionutility::DeployableTIMRPWagonArm>(
            timrpwagon);
    } else {
        std::cout << "The chosen Hardware Configuration is not valid" << std::endl;
        return -1;
    }

    // TIM Client
    auto clientSocketTIM = std::make_shared<crf::communication::sockets::TCPSocket>(
        config["Communications"]["TIM"]["Host"].get<std::string>(),
        config["Communications"]["TIM"]["Port"].get<unsigned int>());
    auto socketTIM = std::make_shared<crf::communication::datapacketsocket::PacketSocket>(
        clientSocketTIM);
    auto tim = std::make_shared<crf::actuators::tim::TIMClient>(
        socketTIM, serverReplyTimeout, streamerFrequency, priority);

    // RP Sensor Client
    auto clientSocketRPSensor = std::make_shared<crf::communication::sockets::TCPSocket>(
        config["Communications"]["RPSensor"]["Host"].get<std::string>(),
        config["Communications"]["RPSensor"]["Port"].get<unsigned int>());
    auto socketRPSensor = std::make_shared<crf::communication::datapacketsocket::PacketSocket>(
        clientSocketRPSensor);
    auto rpSensor = std::make_shared<crf::sensors::rpsensor::RPSensorClient>(
        socketRPSensor, serverReplyTimeout, streamerFrequency, priority);

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
