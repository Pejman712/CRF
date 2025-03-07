/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

// Mapper and RGBD libraries
//#include "RGBDVisionUtility/PCLUtils.hpp"
#include "Mapper3d/Mapper3d.hpp"

#include "Laser/VelodyneHDL/VelodyneHDLLaserPCL.hpp"

// GraphSLAM libraries
#include "GraphOptimization/GraphOptimization.hpp"
#include "GraphOptimization/GraphOptimizationConfiguration.hpp"

// Robot libraries
//#include "IPC/FIFOZMQ.hpp"
//#include "IPC/MMAPZMQ.hpp"
//#include "IPC/NetworkIPC.hpp"
//#include "NetworkServer/TcpServer.hpp"

//#include "CANSocket/CANSocket.hpp"
//#include "CHARMBot/CHARMBot.hpp"
//#include "IMU/VMU931.hpp"
//#include "IPC/FIFO.hpp"
//#include "IPC/MMAP.hpp"
//#include "StateEstimator/kalman/UnscentedKalmanFilter.hpp"
//#include "RobotBaseCommunicationPoint/RobotBaseCommunicationPoint.hpp"
//#include "RobotPoseEstimators/RobotPoseEstimator.hpp"

// PCL libraries
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
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

#define STANDARD_BAUDRATE 115200

using crf::communication::ipc::NetworkIPC;
using crf::communication::ipc::FIFOZMQ;
using crf::communication::ipc::MMAPZMQ;
using crf::communication::networkserver::INetworkServer;
using crf::communication::networkserver::TcpServer;
using crf::robots::robotbase::CHARMBot;
using crf::applications::robotposeestimator::RobotPoseEstimator;
using crf::communication::robotbasecommunicationpoint::RobotBaseCommunicationPoint;

using crf::applications::graphoptimization::GraphOptimizationConfiguration;
using crf::applications::graphoptimization::SlamParameters;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {  // NOLINT
    viewer.setBackgroundColor(0.9, 0.9, 1.0);
}

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    gSignalStatus = signal;
}
}  // unnamed namespace

int main(int argc, char** argv) {
    // --------------------------------------------------------------
    // Input management
    // --------------------------------------------------------------
    if (argc < 2) {
        std::cout << "Few arguments provide:" <<std::endl;
        std::cout << "  [1] Slam configuration file" << std::endl;
        std::cout << "  Help for Slam configuration file:" << std::endl;
        std::cout << "   - mapperConfigFile: Path to configuration Json file of the environment" << std::endl; // NOLINT
        std::cout << "   - graphConfigFile: Path to configuration Json file of the graph" << std::endl; // NOLINT
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
        std::cout << "   - resultName: Result files name" << std::endl;
        std::cout << "   - maxIterations: Number of iterations to work" << std::endl;
        return -1;
    }

    std::ifstream config(argv[1]);
    nlohmann::json jConfig;
    config >> jConfig;

    std::string mapperConfigFile_ = jConfig.at("mapperConfigFile").get<std::string>();
    std::string graphConfigFile_ = jConfig.at("graphConfigFile").get<std::string>();
    std::string canPort_ = jConfig.at("canPort").get<std::string>();
    std::string charmbotConfigFile_ = jConfig.at("charmbotConfigFile").get<std::string>();
    std::string protocol_ = jConfig.at("protocol").get<std::string>();
    unsigned int port_ = jConfig.at("port").get<unsigned int>();
    std::string inputFifo_ = jConfig.at("inputFifo").get<std::string>();
    std::string outputMmap_ = jConfig.at("outputMmap").get<std::string>();
    std::string sensorConfigFile_ = jConfig.at("sensorConfigFile").get<std::string>();
    std::string resultName_ = jConfig.at("resultName").get<std::string>();
    unsigned int maxIterations_ = jConfig.at("maxIterations").get<unsigned int>();

    // --------------------------------------------------------------
    // Start CHARMBot
    // --------------------------------------------------------------
    /*
    std::signal(SIGTSTP, signal_handler);
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
    // Declare and init the graph
    crf::applications::graphoptimization::GraphOptimization graph(mapperConfigFile_);
    if (!graph.parse(graphConfigFile_)) {
        std::cout << "Bad input configFile" << std::endl;
        return 0;
    }

    // Take configuration data
    boost::optional <SlamParameters> boostSlamData = graph.getSlamData();
    boost::optional <Eigen::Matrix4f> boostTranformationData = graph.getTransformationData();
    boost::optional <Eigen::Matrix4f> boostInitialPose = graph.getInitialPositionData();
    boost::optional <std::vector <std::vector <float>>> boostDoorsPosition =
        graph.getDoorsPosition();

    SlamParameters slamData = boostSlamData.get();
    Eigen::Matrix4f tranformationData = boostTranformationData.get();
    Eigen::Matrix4f initialPose = boostInitialPose.get();
    std::vector <std::vector <float>> doorsPosition = boostDoorsPosition.get();

    for (unsigned int i = 0; i < doorsPosition.size(); i++) {
        std::cout << "Door " << i << ": (" << doorsPosition[i][0] << ", " <<  doorsPosition[i][1]
            << ", " << doorsPosition[i][2] << ")" << std::endl;
    }

    double threshold = slamData.threshold;
    bool visualEnable = slamData.visualization;

    // Time declaration
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};

    // Motion matrix
    Eigen::Matrix4f SupposedMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previousMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f RealMotion = Eigen::Matrix4f::Identity();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr previousPointCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>());
    */
    // --------------------------------------------------------------
    // Graph generation
    // --------------------------------------------------------------
    unsigned int id_vertex {0};

    boost::optional<crf::utility::types::TaskPose> boostRobotPosition = boost::none;
    crf::utility::types::TaskPose robotPosition;
    crf::utility::types::TaskPose previousRobotPosition;

    boostRobotPosition = localizer_->getPosition();
    if (!boostRobotPosition) {
        std::cout << "Unable to read from Robot Pose Estimator" << std::endl;
        return 0;
    }
    robotPosition = boostRobotPosition.get();
    previousRobotPosition = robotPosition;

    std::cout << "Generating graph..." << std::endl;
    while ((id_vertex < maxIterations_) && (gSignalStatus != SIGKILL)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // ----------------------------------------------------------
        // 0 - Position estimation
        // ----------------------------------------------------------
        double gtX{0.0}, gtY{0.0}, gtZ{0.0};
        if (id_vertex == maxIterations_ - 1) {
            std::cout << "DON'T MOVE THE ROBOT !!";
            std::cout << "Last position X: ";
            std::cin >> gtX;
            std::cout << "Last position Y: ";
            std::cin >> gtY;
            std::cout << "Last position Z: ";
            std::cin >> gtZ;
        }

        // Call robot pose estimator
        boostRobotPosition = localizer_->getPosition();
        if (!boostRobotPosition) {
            std::cout << "Unable to read from Robot Pose Estimator" << std::endl;
            return 0;
        }
        robotPosition = boostRobotPosition.get();
        // std::cout << "Pose estimated: " << robotPosition << std::endl;

        // Distance to the previous point
        double distance = sqrt(pow((robotPosition(0) - previousRobotPosition(0)), 2)
            + pow((robotPosition(1) - previousRobotPosition(1)), 2)
            + pow((robotPosition(2) - previousRobotPosition(2)), 2));

        if ((distance > abs(threshold)) || (id_vertex == 0)) {
            Eigen::Vector3f odometryPosition;
            odometryPosition << robotPosition(0), robotPosition(1), robotPosition(2);

            Eigen::AngleAxisd rollAngle(robotPosition(3), Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(robotPosition(4), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(robotPosition(5), Eigen::Vector3d::UnitZ());
            Eigen::Quaternion<double> rotVector = yawAngle * pitchAngle * rollAngle;
            Eigen::Matrix3d rotationMatrix = rotVector.matrix();

            SupposedMotion <<
                rotationMatrix(0, 0), rotationMatrix(0, 1), rotationMatrix(0, 2), robotPosition(0),
                rotationMatrix(1, 0), rotationMatrix(1, 1), rotationMatrix(1, 2), robotPosition(1),
                rotationMatrix(2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2), robotPosition(2),
                0, 0, 0, 1;

            std::cout << "ODOMETRY (Vector): " << robotPosition << std::endl;
            std::cout << "ODOMETRY (Matrix): " << SupposedMotion << std::endl;
            previousRobotPosition = robotPosition;

            // ----------------------------------------------------------
            // 1 - Coordinate system transformation
            // ----------------------------------------------------------

            SupposedMotion = initialPose * SupposedMotion * tranformationData;

            // ----------------------------------------------------------
            // 2 - Point cloud adquisition
            // ----------------------------------------------------------

            // Start the chrono
            std::cout << "2 - Point adquisition... ";
            start = std::chrono::system_clock::now();

            // Generate point cloud
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud
                (new pcl::PointCloud<pcl::PointXYZRGBA>());
            inPointCloud = velo.getPointCloud();

            std::cout << "Done" << std::endl;
            end = std::chrono::system_clock::now();
            elapsedMilliseconds =
                std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
            std::cout << "2 - Point adquisition execution time: " <<
                elapsedMilliseconds << std::endl;

            // ----------------------------------------------------------
            // 3 - Comparation with the previous point cloud
            // ----------------------------------------------------------

            // Start the chrono
            std::cout << "3 - Comparation with the previous point cloud... " << std::endl;
            start = std::chrono::system_clock::now();

            // Update map and take the matrix of motion
            boost::optional<Eigen::Matrix4f> boostRealMotion;
            previousMotion = RealMotion;

            if (id_vertex > 0) {
                boostRealMotion = graph.comparePointClouds(SupposedMotion, RealMotion,
                    inPointCloud, previousPointCloud);

                if (!boostRealMotion) {
                    std::cout << "Unable to update the map" <<std::endl;
                    return -1;
                }
                RealMotion = boostRealMotion.get();
                SupposedMotion = RealMotion;
                std::cout << "Real motion: " << RealMotion << std::endl;
            }

            previousPointCloud = inPointCloud;

            std::cout << "3 - Comparation with the previous point cloud... Done" << std::endl;
            end = std::chrono::system_clock::now();
            elapsedMilliseconds =
                std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
            std::cout << "3 - Comparation with the previous point cloud execution time: "
                << elapsedMilliseconds << std::endl;

            // ----------------------------------------------------------
            // 4 - Add the new vertex to the graph
            // 5 - Calculation of the K NN
            // 6 - PC comparation with the K NN (less with the previous one)
            // 7 - Add the edges to the graph
            // 8 - Pose estimation of the new pose
            // 9 - Add the new pose to the kdtree
            // ----------------------------------------------------------

            // Add the vertex to the graph
            Eigen::Vector3f fixPose;
            if (id_vertex == 0) {
                fixPose << initialPose(0, 3), initialPose(1, 3), initialPose(2, 3);
                if (!graph.addVertex(RealMotion, previousMotion, fixPose, odometryPosition,
                    boost::none)) {
                    std::cout << "Unable to create the vertex" << std::endl;
                    return -1;
                }
            } else if (id_vertex == maxIterations_ - 1) {
                fixPose << gtX, gtY, gtZ;
                if (!graph.addVertex(RealMotion, previousMotion, fixPose, odometryPosition,
                    boost::none)) {
                    std::cout << "Unable to create the vertex" << std::endl;
                    return -1;
                }
            } else {
                if (!graph.addVertex(RealMotion, previousMotion, boost::none, odometryPosition,
                    boost::none)) {
                    std::cout << "Unable to create the vertex" << std::endl;
                    return -1;
                }
            }

            id_vertex++;
        }
    }

    /*std::cout << "CHARMBot process clean up\n";
    communicationPoint.deinitialize();
    bot->deinitialize();*/

    std::cout << "Generating graph... Done" << std::endl;

    // Save the graph
    if (!graph.saveGraph(resultName_)) {
        std::cout << "There was a problem saving the graph" << std::endl;
        return -1;
    }

    // Show the results
    if (!graph.saveResults(resultName_)) {
        std::cout << "There was a problem showing the graph" << std::endl;
        return -1;
    }

    // Obtain data from graph
    boost::optional<std::vector <pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>>
        boostNodePointCloud = graph.getNodePointclouds();
    if (!boostNodePointCloud) {
        std::cout << "Unable to read the node point cloud" << std::endl;
        return -1;
    }
    std::vector <pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> nodePointCloud =
        boostNodePointCloud.get();

    boost::optional <std::vector <Eigen::Matrix4f, Eigen::aligned_allocator <Eigen::Matrix4f>>>
        boostNodePosition = graph.getNodePositions();
    if (!boostNodePosition) {
        std::cout << "Unable to read the node point cloud" << std::endl;
        return -1;
    }
    std::vector <Eigen::Matrix4f, Eigen::aligned_allocator <Eigen::Matrix4f>> nodePosition =
        boostNodePosition.get();

    // Visualization after optimize the graph
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>());
    for (size_t i = 0; i < nodePosition.size(); i++) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud(
            new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr subsampledCloud(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        pcl::transformPointCloud(*nodePointCloud[i], *transformedCloud, nodePosition[i]);
        pcl::copyPointCloud(*transformedCloud, *auxCloud);
        crf::utility::rgbdvisionutility::PCLUtils::subsample(auxCloud, subsampledCloud, 0.01f);

        pcl::copyPointCloud(*subsampledCloud, *cloud_);
    }

    // Save PC
    pcl::io::savePCDFileASCII(resultName_, *cloud_);

    if (visualEnable) {
        pcl::visualization::CloudViewer viewer("Viewer");
        viewer.showCloud(cloud_);

        viewer.runOnVisualizationThreadOnce(viewerOneOff);

        // Next iteration
        char next {};
        std::cin >> next;
    }

    return 0;
}
