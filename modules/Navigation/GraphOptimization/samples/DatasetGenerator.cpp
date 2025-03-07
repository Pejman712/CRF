/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

// CRF libraries
#include "Laser/VelodyneHDL/VelodyneHDLLaserPCL.hpp"

#include "IPC/FIFOZMQ.hpp"
#include "IPC/MMAPZMQ.hpp"
#include "IPC/NetworkIPC.hpp"
#include "NetworkServer/TcpServer.hpp"
#include "Types/Types.hpp"

#include "CANSocket/CANSocket.hpp"
#include "CHARMBot/CHARMBot.hpp"
#include "IMU/VMU931.hpp"
#include "IPC/FIFO.hpp"
#include "IPC/MMAP.hpp"
#include "StateEstimator/kalman/UnscentedKalmanFilter.hpp"
#include "RobotBaseCommunicationPoint/RobotBaseCommunicationPoint.hpp"
#include "RobotPoseEstimators/RobotPoseEstimator.hpp"

// PCL libraries
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>

// Generig libraries
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <nlohmann/json.hpp>
#include <boost/optional.hpp>
#include <string>
#include <memory>
#include <math.h>
#include <csignal>
#include <stdio.h>
#include <chrono>
#include <cstring>

#define STANDARD_BAUDRATE 115200

using crf::communication::ipc::NetworkIPC;
using crf::communication::ipc::FIFOZMQ;
using crf::communication::ipc::MMAPZMQ;
using crf::communication::networkserver::INetworkServer;
using crf::communication::networkserver::TcpServer;
using crf::robots::robotbase::CHARMBot;
using crf::applications::robotposeestimator::RobotPoseEstimator;
using crf::communication::robotbasecommunicationpoint::RobotBaseCommunicationPoint;

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signalHandler(int signal) {
    gSignalStatus = signal;
}
}  // unnamed namespace

bool isFileExist(const std::string filename) {
    /// Open the file
    FILE* f = fopen(filename.c_str(), "rb");

    /// in the event of a failure, return false
    if (!f)
        return false;

    fclose(f);

    return true;
}

int main(int argc, char** argv) {
    // --------------------------------------------------------------
    // Input management
    // --------------------------------------------------------------
    if (argc < 2) {
        std::cout << "Few arguments provide:" <<std::endl;
        std::cout << "  [1] Slam configuration file" << std::endl;
        std::cout << "  Help for Slam configuration file. Needed items:" << std::endl;
        std::cout << "   - canPort: Can port name (e.g. can0)" << std::endl;
        std::cout << "   - charmbotConfigFile: Configuration file path for the robot (CHARMBot)" << std::endl; // NOLINT
        std::cout << "   - protocol: Protocol type (available: tcp) [Required if port is set]. ";
        std::cout << "protocol = \"none\" if not required" << std::endl;
        std::cout << "   - port: Network port [1-65535] [Required if protocol is set]. ";
        std::cout << "port = 0 if not required" << std::endl;
        std::cout << "   - inputFifo: Input FIFO [Required if output mmap is set]. ";
        std::cout << "inputFifo = \"none\" if not required" << std::endl;
        std::cout << "   - outputMmap: Output MMAP [Required if input fifo is set]. ";
        std::cout << "outputMmap = \"none\" if not required" << std::endl;
        std::cout << "   - sensorConfigFile: Lidar or RGBd Camera config file" << std::endl;
        return -1;
    }

    std::signal(SIGINT, signalHandler);

    std::ifstream config(argv[1]);
    nlohmann::json jConfig;
    config >> jConfig;

    std::string canPort_ = jConfig.at("canPort").get<std::string>();
    std::string charmbotConfigFile_ = jConfig.at("charmbotConfigFile").get<std::string>();
    std::string protocol_ = jConfig.at("protocol").get<std::string>();
    unsigned int port_ = jConfig.at("port").get<unsigned int>();
    std::string inputFifo_ = jConfig.at("inputFifo").get<std::string>();
    std::string outputMmap_ = jConfig.at("outputMmap").get<std::string>();
    std::string sensorConfigFile_ = jConfig.at("sensorConfigFile").get<std::string>();
    std::string resultName_ = jConfig.at("resultName").get<std::string>();

    // --------------------------------------------------------------
    // Start CHARMBot
    // --------------------------------------------------------------
    printf("Starting CHARMBot\n");
    std::shared_ptr<CANSocket> socket = std::make_shared<CANSocket>(
        reinterpret_cast<const char*>(canPort_.c_str()));

    std::ifstream configFile(charmbotConfigFile_);
    if ((configFile.rdstate() & std::ifstream::failbit) != 0) {
        std::cout << "Provided configuration file does not exists" << std::endl;
        return -1;
    }

    std::shared_ptr<CHARMBot> bot = std::make_shared<CHARMBot>(
        socket, nlohmann::json::parse(configFile));
    if (!bot->initialize()) {
        std::cout << "Could not initialize the CHARMBot" << std::endl;
        return -1;
    }
    RobotBaseCommunicationPoint communicationPoint(bot);
    std::shared_ptr<NetworkIPC> networkIPC = nullptr;
    if ((protocol_ != "none") && (port_ != 0)) {
        std::string net_protocol = protocol_;
        unsigned int net_port = port_;

        if ((net_protocol != "tcp") || (net_port < 1) || (net_port > 65535)) {
            std::cout << "Wrong network parameters" << std::endl << std::endl;
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

    if ((inputFifo_ != "none") && (outputMmap_ != "none")) {
        std::string input_fifo_filename = inputFifo_;
        std::string output_mmap_filename = outputMmap_;

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

    std::cout << "CHARMBot started correctly\n";
    networkIPC->open();

    // --------------------------------------------------------------
    // Start robotPoseEstimator
    // --------------------------------------------------------------
    std::vector< std::string > available = crf::sensors::imu::VMU931::getConnectedDevicesList();
    std::shared_ptr<crf::sensors::imu::IIMU> imuSensor = nullptr;
    if (available.size() > 0) {
        auto serial = std::make_shared<crf::communication::serialcommunication::
            SerialCommunication>(available[0], STANDARD_BAUDRATE);
            imuSensor = std::make_shared<crf::sensors::imu::VMU931>(serial);
    }

    auto localizer_ = std::make_shared<crf::applications::robotposeestimator::RobotPoseEstimator>
        (std::make_shared<Kalman::UnscentedKalmanFilter<crf::applications::
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
    // Open LIDAR
    // --------------------------------------------------------------
    std::cout << "Opening velodyne... " << std::endl;

    std::ifstream laserData(sensorConfigFile_);
    nlohmann::json laserJSON = nlohmann::json::parse(laserData);
    crf::sensors::laser::VelodyneHDLLaserPCL velo(laserJSON);

    if (!velo.initialize()) {
        std::cout << std::endl << "Unable to initialize velodyne";
        return -1;
    }
    std::cout << "Opening velodyne... Done" << std::endl;

    // --------------------------------------------------------------
    // Declarations
    // --------------------------------------------------------------
    nlohmann::json jdataFile;
    auto start = std::chrono::system_clock::now();

    // File names
    time_t now = time(0);
    char* dt = ctime(&now); //NOLINT
    std::string saveName = "../modules/Applications/GraphOptimization/results/";
    saveName.append("dataset_");
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

    boost::optional<crf::utility::types::TaskPose> boostRobotPosition = boost::none;
    crf::utility::types::TaskPose robotPosition;
    crf::utility::types::TaskPose previousRobotPosition;

    boostRobotPosition = localizer_->getPosition();
    if (!boostRobotPosition) {
        std::cout << "Unable to read from Robot Pose Estimator" << std::endl;
        return 0;
    }
    robotPosition = boostRobotPosition.get();

    std::cout << "Recording information..." << std::endl;
    std::chrono::duration<float, std::milli> time_{0.0};
    while (gSignalStatus != SIGINT) {
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

        // ----------------------------------------------------------
        // Test if distance is higher than a threshold to pass to the following step
        // ----------------------------------------------------------
        // To do in the graphSlam

        // ----------------------------------------------------------
        // From task position to Eigen matrix
        // ----------------------------------------------------------
        // To do in the graphSlam

        // ----------------------------------------------------------
        // Asign previous position
        // ----------------------------------------------------------
        // To do in the graphSlam

        // ----------------------------------------------------------
        // 1 - Coordinate system transformation
        // ----------------------------------------------------------
        // To do in the graphSlam

        // ----------------------------------------------------------
        // 2 - Point cloud adquisition
        // ----------------------------------------------------------

        // Start the chrono
        std::cout << "2 - PointCloud adquisition... " << std::endl;
        // Generate point cloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud
            (new pcl::PointCloud<pcl::PointXYZRGBA>());
        inPointCloud = velo.getPointCloud();
        std::cout << "2 - PointCloud adquisition... Done" << std::endl;

        // ----------------------------------------------------------
        // 3 - Comparation with the previous point cloud
        // ----------------------------------------------------------
        // To do in the graphSlam

        // ----------------------------------------------------------
        // 4 - Add the new vertex to the graph
        // 5 - Calculation of the K NN
        // 6 - PC comparation with the K NN (less with the previous one)
        // 7 - Add the edges to the graph
        // 8 - Pose estimation of the new pose
        // 9 - Add the new pose to the kdtree
        // ----------------------------------------------------------
        // Add the vertex to the graph
        // To do in the graphSlam

        // Save Point Cloud
        std::string pointCloudName{saveName};
        pointCloudName += "pc/pc";
        pointCloudName += std::to_string(id_vertex);
        pcl::io::savePCDFileASCII(pointCloudName, *inPointCloud);

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

    // ----------------------------------------------------------
    // Save results and show them
    // ----------------------------------------------------------
    // To do in the graphSlam

    return 0;
}
