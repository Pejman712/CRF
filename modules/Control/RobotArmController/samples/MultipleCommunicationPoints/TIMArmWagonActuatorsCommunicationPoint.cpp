/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
*/

#include <csignal>
#include <iostream>
#include <memory>
#include <string>

#include <boost/program_options.hpp>

#include "EtherCATDevices/TIMRobotArmWagonMotors/TIMRobotArmWagonMotors.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "Sockets/TCP/TCPServer.hpp"

// Stabilizer
#include "MechanicalStabilizer/TIMStabilizer/TIMStabilizer.hpp"
#include "MechanicalStabilizer/MechanicalStabilizerCommunicationPoint/MechanicalStabilizerManager.hpp"
#include "MechanicalStabilizer/MechanicalStabilizerCommunicationPoint/MechanicalStabilizerCommunicationPointFactory.hpp"

// Shielding
#include "Shielding/RadioactiveSourceShielding/RadioactiveSourceShielding.hpp"
#include "Shielding/ShieldingCommunicationPoint/ShieldingManager.hpp"
#include "Shielding/ShieldingCommunicationPoint/ShieldingCommunicationPointFactory.hpp"

// TIM Arm
#include "TIMArm/TIMArm.hpp"
#include "KinovaArm/KinovaApiInterface.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerCommunicationPointFactory.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerManager.hpp"

namespace po = boost::program_options;

using crf::control::robotarmcontroller::RobotArmControllerManager;
using crf::control::robotarmcontroller::RobotArmControllerCommunicationPointFactory;
using crf::actuators::mechanicalstabilizer::MechanicalStabilizerManager;
using crf::actuators::mechanicalstabilizer::MechanicalStabilizerCommunicationPointFactory;
using crf::actuators::shielding::ShieldingManager;
using crf::actuators::shielding::ShieldingCommunicationPointFactory;

using crf::communication::communicationpointserver::CommunicationPointServer;

namespace {
volatile std::sig_atomic_t signalStatus;
void signalHandler(int signal) {
    std::cout << ": Stop signal [SIGTSTP] received" << std::endl;
    signalStatus = signal;
}
}  // unnamed namespace

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("ethercat_port", po::value<std::string>(), "EtherCAT port name (e.g. enp5s0).")
        ("port_stabilizer", po::value<unsigned int>(), "Network port for stabilizer [1-65535].")
        ("port_shielding", po::value<unsigned int>(), "Network port for shielding [1-65535].")
        ("port_arm", po::value<unsigned int>(), "Network port for TIM Arm [1-65535].")
        ("configuration", po::value<std::string>(), "Configuration file path for the arm");

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

    if (!vm.count("ethercat_port")) {
        std::cout << "Missing EtherCAT port." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("port_stabilizer") || !vm.count("port_shielding") || !vm.count("port_arm")) {
        std::cout << "Missing at least one port." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    int networkPortStabilizer = vm["port_stabilizer"].as<unsigned int>();
    if ((networkPortStabilizer < 1) || (networkPortStabilizer > 65535)) {
        std::cout << "Wrong network parameters for the stabilizer." << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    int networkPortShielding = vm["port_shielding"].as<unsigned int>();
    if ((networkPortShielding < 1) || (networkPortShielding > 65535)) {
        std::cout << "Wrong network parameters for the shielding." << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    int networkPortArm = vm["port_arm"].as<unsigned int>();
    if ((networkPortArm < 1) || (networkPortArm > 65535)) {
        std::cout << "Wrong network parameters for the arm." << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }


    // Ethercat Motors
    auto motors = std::make_shared<crf::devices::ethercatdevices::TIMRobotArmWagonMotors>(
        vm["ethercat_port"].as<std::string>());
    if (!motors->initialize()) {
        std::cout << "Cannot initialize BLM Wagon motors" << std::endl;
        return -1;
    }


    // TIM Arm
    std::ifstream robotData(vm["configuration"].as<std::string>());
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    auto arm = std::make_shared<crf::actuators::timarm::TIMArm>(robotJSON, motors,
        std::make_shared<crf::actuators::kinovaarm::KinovaApiInterface>());
    if (!arm->initialize()) {
        std::cout << "Could not initialize the arm" << std::endl;
        return -1;
    }

    std::shared_ptr<RobotArmControllerManager> managerArm(new RobotArmControllerManager(arm));
    std::shared_ptr<RobotArmControllerCommunicationPointFactory> communicationPointFactoryArm(
        new RobotArmControllerCommunicationPointFactory(managerArm));


    // Stabilizer
    auto motorStabilizer = motors->getStabilizer();
    if (!motorStabilizer) {
        std::cout << "Cannot retrieve Harmonic Drive 4." << std::endl;
        return -1;
    }
    std::shared_ptr<crf::actuators::mechanicalstabilizer::TIMStabilizer> stabilizer(
        new crf::actuators::mechanicalstabilizer::TIMStabilizer(motorStabilizer.value()));
    if (!stabilizer->initialize()) {
        std::cout << "Failed to initialize the stabilizer." << std::endl;
        return -1;
    }
    stabilizer->deinitialize();

    std::shared_ptr<MechanicalStabilizerManager> managerStabilizer(new MechanicalStabilizerManager(
        stabilizer));
    std::shared_ptr<MechanicalStabilizerCommunicationPointFactory> communicationPointFactoryStabilizer(  // NOLINT
        new MechanicalStabilizerCommunicationPointFactory(managerStabilizer));


    // Shielding
    auto motorShielding = motors->getShielding();
    if (!motorShielding) {
        std::cout << "Cannot retrieve shielding motor." << std::endl;
        return -1;
    }
    std::shared_ptr<crf::actuators::shielding::RadioactiveSourceShielding> shielding(
        new crf::actuators::shielding::RadioactiveSourceShielding(motorShielding.value()));
    if (!shielding->initialize()) {
        std::cout << "Failed to initialize the shielding." << std::endl;
        return -1;
    }
    shielding->deinitialize();

    std::shared_ptr<ShieldingManager> managerShielding(new ShieldingManager(shielding));
    std::shared_ptr<ShieldingCommunicationPointFactory> communicationPointFactoryShielding(
        new ShieldingCommunicationPointFactory(managerShielding));


    // Communication points
    std::unique_ptr<CommunicationPointServer> networkServerStabilizer;
    std::unique_ptr<CommunicationPointServer> networkServerShielding;
    std::unique_ptr<CommunicationPointServer> networkServerArm;

    std::shared_ptr<crf::communication::sockets::ISocketServer> serverStabilizer;
    std::shared_ptr<crf::communication::sockets::ISocketServer> serverShielding;
    std::shared_ptr<crf::communication::sockets::ISocketServer> serverArm;

    serverStabilizer = std::make_shared<crf::communication::sockets::TCPServer>(
        networkPortStabilizer);
    serverShielding = std::make_shared<crf::communication::sockets::TCPServer>(
        networkPortShielding);
    serverArm = std::make_shared<crf::communication::sockets::TCPServer>(
        networkPortArm);

    networkServerStabilizer.reset(new CommunicationPointServer(
        serverStabilizer, communicationPointFactoryStabilizer));
    if (!networkServerStabilizer->initialize()) {
        std::cout << "Failed to initialize network server stabilizer" << std::endl;
        return -1;
    }
    std::cout << "Stabilizer communication point on " << networkPortStabilizer <<
        " started. Type [CTRL+Z] to close it deinitialize the stabilizer" << std::endl;

    networkServerShielding.reset(new CommunicationPointServer(serverShielding,
        communicationPointFactoryShielding));
    if (!networkServerShielding->initialize()) {
        std::cout << "Failed to initialize network server shielding" << std::endl;
        return -1;
    }
    std::cout << "Shielding communication point on " << networkPortShielding <<
        " started. Type [CTRL+Z] to close it deinitialize the shielding" << std::endl;

    networkServerArm.reset(new CommunicationPointServer(serverArm,
        communicationPointFactoryArm));
    if (!networkServerArm->initialize()) {
        std::cout << "Failed to initialize network server arm" << std::endl;
        return -1;
    }
    std::cout << "TIM Arm communication point on " << networkPortArm <<
        " started. Type [CTRL+Z] to close it deinitialize the TIM Arm" << std::endl;


    std::signal(SIGTSTP, signalHandler);
    while (signalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "Received closing command" << std::endl;
    return 0;
}
