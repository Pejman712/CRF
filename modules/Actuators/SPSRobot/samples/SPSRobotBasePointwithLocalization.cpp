/* © Copyright CERN 2020.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 2020
           Alejandro Diaz Rosales CERN EN/SMM/MRO 2020
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

#include "IPC/FIFOZMQ.hpp"
#include "IPC/MMAPZMQ.hpp"
#include "IPC/NetworkIPC.hpp"
#include "NetworkServer/TcpServer.hpp"

#include "SPSRobot/SPSRobot.hpp"
#include "IPC/FIFO.hpp"
#include "IPC/MMAP.hpp"
#include "RobotBaseCommunicationPoint/RobotBaseCommunicationPoint.hpp"

#include "StateEstimator/kalman/UnscentedKalmanFilter.hpp"
#include "RobotPoseEstimators/RobotPoseEstimator.hpp"
#include "IMU/VMU931.hpp"

namespace po = boost::program_options;

using cern::communication::ipc::NetworkIPC;
using cern::communication::ipc::FIFOZMQ;
using cern::communication::ipc::MMAPZMQ;
using cern::communication::networkserver::INetworkServer;
using cern::communication::networkserver::TcpServer;
using cern::robots::robotbase::SPSRobot;
using cern::communication::robotbasecommunicationpoint::RobotBaseCommunicationPoint;

using cern::applications::robotposeestimator::RobotPoseEstimator;

#define STANDARD_BAUDRATE 115200

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
        ("ethercat_port", po::value<std::string>(), "EtherCAT port name (e.g. eno1)")  // NOLINT
        ("configuration", po::value<std::string>(), "Configuration file path for the SPSRobot")
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp) [Required if port is set]")  // NOLINT
        ("port", po::value<unsigned int>(), "Network port [1-65535] [Required if protocol is set]")
        ("input_fifo", po::value<std::string>(), "Input FIFO [Required if output mmap is set]")
        ("output_mmap", po::value<std::string>(), "Output MMAP [Required if input fifo is set]");

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

    if (!vm.count("ethercat_port")) {
        std::cout << "Missing can port" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("configuration")) {
        std::cout << "Missing configuration file" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::signal(SIGTSTP, signal_handler);
    printf("Starting SPSRobot\n");

    std::ifstream configFile(vm["configuration"].as<std::string>());  // NOLINT
    if ((configFile.rdstate() & std::ifstream::failbit) != 0) {
        std::cout << "Provided configuration file does not exists" << std::endl;
        return -1;
    }
    std::shared_ptr<SPSRobot> bot = std::make_shared<SPSRobot>(
        reinterpret_cast<const char*>(vm["ethercat_port"].as<std::string>().c_str()),
        nlohmann::json::parse(configFile));
    if (!bot->initialize()) {
        std::cout << "Could not initialize the SPSRobot" << std::endl;
        return -1;
    }

    RobotBaseCommunicationPoint communicationPoint(bot);
    std::shared_ptr<NetworkIPC> networkIPC = nullptr;
    if (vm.count("protocol") && vm.count("port")) {
        std::string net_protocol = vm["protocol"].as<std::string>();
        int net_port = vm["port"].as<unsigned int>();

        if ((net_protocol != "tcp") || (net_port < 1) || (net_port > 65535)) {
            std::cout << "Wrong network parameters" << std::endl << std::endl;
            std::cout << desc << std::endl;
            return -1;
        }

        std::shared_ptr<INetworkServer> server;
        if (net_protocol == "tcp") {
            server = std::make_shared<TcpServer>(net_port);
        }
        networkIPC = std::make_shared<NetworkIPC>(server);
        if (!communicationPoint.addIpcPair({networkIPC, networkIPC})) {
            std::cout << "Could not add IPC pair" << std::endl;
        }
    }

    if (vm.count("input_fifo") && (vm.count("output_mmap"))) {
        std::string input_fifo_filename = vm["input_fifo"].as<std::string>();
        std::string output_mmap_filename = vm["output_mmap"].as<std::string>();

        auto fifo = FIFOZMQ::CreateReaderPtrNonBlock(input_fifo_filename,
            std::chrono::milliseconds(2000));
        auto mmap = MMAPZMQ::CreateWriterPtr(output_mmap_filename);

        if (!communicationPoint.addIpcPair({fifo, mmap})) {
            std::cout << "Could not add IPC pair" << std::endl;
        }
    }

    if (!communicationPoint.initialize()) {
        std::cout << "Could not initialize the RobotBaseCommunicationPoint" << std::endl;
        return -1;
    }

    if (networkIPC != nullptr) {
        std::cout << "Waiting for client communication..." << std::endl;
        networkIPC->open();
    }

    std::cout << "SPS Robot started correctly\n";
    while (gSignalStatus != SIGKILL) {
        usleep(5000);
    }

    // --------------------------------------------------------------
    // Start robotPoseEstimator
    // --------------------------------------------------------------
    std::vector<std::string> available = cern::sensors::imu::VMU931::getConnectedDevicesList();
    std::shared_ptr<cern::sensors::imu::IIMU> imuSensor = nullptr;
    if (available.size() > 0) {
        auto serial = std::make_shared<cern::communication::serialcommunication::
            SerialCommunication>(available[0], STANDARD_BAUDRATE);
            imuSensor = std::make_shared<cern::sensors::imu::VMU931>(serial);
    }

    auto localizer_ = std::make_shared<cern::applications::robotposeestimator::RobotPoseEstimator>
        (std::make_shared<Kalman::UnscentedKalmanFilter<cern::applications::
        robotposeestimator::SystemState>>(1), bot, imuSensor);

    if (!localizer_->initialize()) {
        std::cout << "Could not initialize the RobotPositionLocalizer" << std::endl;
        return -1;
    }
    sleep(1);
    if (!localizer_->getPosition()) {
        std::cout << "Could create RobotPositionLocalizer" << std::endl;
        return -1;
    }

    // --------------------------------------------------------------
    // Declarations
    // --------------------------------------------------------------
    nlohmann::json jdataFile;
    auto start = std::chrono::system_clock::now();

    // File names
    time_t now = time(0);
    char* dt = ctime(&now); //NOLINT
    std::string saveName = "dataset_";
    saveName.append(dt);
    saveName.pop_back();
    saveName.append("/");
    for (size_t i = 0; i < saveName.size(); i++) {
        if ((saveName.at(i) == ' ') || (saveName.at(i) == ':')) {
            saveName.at(i) = '_';
            if (i > 0) {
                if (saveName.at(i - 1) == '_') {
                    saveName.erase(i, 1);
                }
            }
        }
    }

    std::string command{"mkdir "};
    command += saveName;
    char *strCommand = new char[command.size() + 1];
    strcpy(strCommand, command.c_str());  // NOLINT
    command += "pc/";
    char *strCommand2 = new char[command.size() + 1];
    strcpy(strCommand2, command.c_str());  // NOLINT

    std::cout << "Creating dataset folder..." << std::endl;
    if ((std::system(strCommand) != 0) || (std::system(strCommand2) != 0)) {
        std::cout << "Error creating dataset folder" << std::endl;
        return -1;
    }
    std::cout << "Creating dataset folder... Done" << std::endl;
    delete strCommand;

    // --------------------------------------------------------------
    // Dataset generation
    // --------------------------------------------------------------
    std::cout << "5 seconds to start" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    unsigned int id_vertex{0};

    boost::optional<cern::utility::types::CartesianPosition> boostRobotPosition = boost::none;
    cern::utility::types::CartesianPosition robotPosition;

    boostRobotPosition = localizer_->getPosition();
    if (!boostRobotPosition) {
        std::cout << "Unable to read from Robot Pose Estimator" << std::endl;
        return 0;
    }
    robotPosition = boostRobotPosition.get();

    std::cout << "Recording information..." << std::endl;
    std::chrono::duration<float, std::milli> time_{0.0};
    while (gSignalStatus != SIGINT) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        time_ = std::chrono::system_clock::now() - start;

        // ----------------------------------------------------------
        // 0 - Position estimation
        // ----------------------------------------------------------
        // Call robot pose estimator
        boostRobotPosition = localizer_->getPosition();
        if (!boostRobotPosition) {
            std::cout << "Unable to read from Robot Pose Estimator" << std::endl;
            return 0;
        }
        robotPosition = boostRobotPosition.get();

        // Save robot pose estimator
        jdataFile["poses"][std::to_string(id_vertex)]["time"] = static_cast<float>(time_.count());
        jdataFile["poses"][std::to_string(id_vertex)]["pose"] = {robotPosition(0), robotPosition(1),
            robotPosition(2), robotPosition(3), robotPosition(4), robotPosition(5)};

        id_vertex++;
    }

    // Save json file
    std::string infoFileName{saveName};
    infoFileName += "info.json";
    jdataFile["numPoints"] = id_vertex;

    try {
        std::ofstream outFile;
        outFile.open(infoFileName, std::ofstream::out);
        outFile << std::setw(4) << jdataFile << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Unable to create the new account because: " << e.what() << std::endl;
        return false;
    }


    std::cout << "SPS Robot process clean up\n";
    communicationPoint.deinitialize();
    bot->deinitialize();
    return 0;
}
