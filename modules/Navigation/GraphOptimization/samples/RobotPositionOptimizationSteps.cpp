/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

// Mapper and RGBD libraries
#include "RGBDCamera/RealSense2Grabber.hpp"
#include "RGBDCamera/IRGBDCamera.hpp"
#include "RGBDCamera/RGBDUtils.hpp"
#include "RGBDVisionUtility/PCLUtils.hpp"
#include "Mapper3d/Mapper3d.hpp"

// GraphSLAM libraries
#include "GraphOptimization/GraphOptimization.hpp"

// Robot libraries
#include "IPC/FIFOZMQ.hpp"
#include "IPC/MMAPZMQ.hpp"
#include "IPC/NetworkIPC.hpp"
#include "NetworkServer/TcpServer.hpp"

#include "CANSocket/CANSocket.hpp"
#include "CHARMBot/CHARMBot.hpp"
#include "IMU/VMU931.hpp"
#include "IPC/FIFO.hpp"
#include "IPC/MMAP.hpp"
#include "StateEstimator/kalman/UnscentedKalmanFilter.hpp"
#include "RobotBaseCommunicationPoint/RobotBaseCommunicationPoint.hpp"
#include "RobotPoseEstimators/RobotPoseEstimator.hpp"

// PCL libraries
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>

// Generig libraries
#include <Eigen/LU>
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>
#include <nlohmann/json.hpp>
#include <boost/optional.hpp>
#include <string>
#include <memory>
#include <math.h>
#include <boost/program_options.hpp>
#include <csignal>
#include <experimental/optional>

#define STANDARD_BAUDRATE 115200

namespace po = boost::program_options;

using std::experimental::nullopt;

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
void signal_handler(int signal) {
    gSignalStatus = signal;
}
}  // unnamed namespace

std::mutex mutex_;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_(
    new pcl::PointCloud<pcl::PointXYZRGBA>());
bool stopViz_ = false;
bool updateMap_ = false;

void Visualization() {
    pcl::visualization::PCLVisualizer environmentViewer("ENVIRONMENT");
    environmentViewer.setBackgroundColor(0.0, 0.0, 0.0);

    int id = 0;
    std::string enviromentName;

    while (!stopViz_) {
        enviromentName = "";
        enviromentName = std::to_string(id);
        mutex_.lock();

        if (cloud_->size() > 0) {
            environmentViewer.removePointCloud(enviromentName);
            environmentViewer.addPointCloud(cloud_, enviromentName);
            environmentViewer.spinOnce();
            if (updateMap_) {
                id++;
                updateMap_ = false;
            }
        }
        mutex_.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    environmentViewer.close();
}

int main(int argc, char **argv) {
    // --------------------------------------------------------------
    // Input management
    // --------------------------------------------------------------
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("rgbdConfigFile", po::value<std::string>(), "[1] Path to RGBD Camera config file")
        ("mapperConfigFile", po::value<std::string>(),
            "[2] Path to configuration Json file of the environment)")
        ("graphConfigFile", po::value<std::string>(),
            "[3] Path to configuration Json file of the graph")
        ("can_port", po::value<std::string>(), "[4] Can port name (e.g. can0)") // NOLINT
        ("configuration", po::value<std::string>(),
            "[5] Configuration file path for the robot (CHARMBot)")
        ("protocol", po::value<std::string>(),
            "[6] Protocol type (available: tcp) [Required if port is set]") // NOLINT
        ("port", po::value<unsigned int>(),
            "[7] Network port [1-65535] [Required if protocol is set]")
        ("input_fifo", po::value<std::string>(),
            "[8] Input FIFO [Required if output mmap is set]")
        ("output_mmap", po::value<std::string>(),
            "[9] Output MMAP [Required if input fifo is set]")
        ("resultName", po::value<std::string>(), "[10] Result files name")
        ("maxIterations", po::value<unsigned int>(), "[11] Number of images");

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

    if (!vm.count("rgbdConfigFile")) {
        std::cout << "Missing camera configFile" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("mapperConfigFile")) {
        std::cout << "Missing mapper configFile" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("graphConfigFile")) {
        std::cout << "Missing graphSLAM configFile" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("can_port")) {
        std::cout << "Missing can port" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("configuration")) {
        std::cout << "Missing configuration file" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("resultName")) {
        std::cout << "Missing the name of the result files" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("maxIterations")) {
        std::cout << "Missing iteration number" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    // --------------------------------------------------------------
    // Start CHARMBot
    // --------------------------------------------------------------
    std::signal(SIGTSTP, signal_handler);
    printf("Starting CHARMBot\n");
    std::shared_ptr<CANSocket> socket = std::make_shared<CANSocket>(
        reinterpret_cast<const char*>(vm["can_port"].as<std::string>().c_str()));

    std::ifstream configFile(vm["configuration"].as<std::string>());
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

    std::cout << "CHARMBot started correctly\n";
    networkIPC->open();

    // --------------------------------------------------------------
    // Start robotPoseEstimator
    // --------------------------------------------------------------
    std::vector< std::string > available = crf::sensors::imu::VMU931::getConnectedDevicesList();
    std::shared_ptr<crf::applications::robotposeestimator::IRobotPoseEstimator> localizer_;
    std::shared_ptr<crf::sensors::imu::IIMU> imuSensor = nullptr;
    if (available.size() > 0) {
        auto serial = std::make_shared<crf::communication::serialcommunication::
            SerialCommunication>(available[0], STANDARD_BAUDRATE);
            imuSensor = std::make_shared<crf::sensors::imu::VMU931>(serial);
    }
    localizer_ =
        std::make_shared<crf::applications::robotposeestimator::RobotPoseEstimator>
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
    // Open camera
    // --------------------------------------------------------------
    std::cout << "Opening camera... " << std::endl;
    std::string configFileCamera(vm["rgbdConfigFile"].as<std::string>());
    std::ifstream configCamera(configFileCamera);
    nlohmann::json jConfigCamera;
    configCamera >> jConfigCamera;
    auto camera = std::make_shared<crf::sensors::rgbdcamera::RealSense2Grabber>
        (jConfigCamera.at("RealSense"));
    std::shared_ptr<crf::sensors::rgbdcamera::IRGBDCamera> camera_(camera);
    if (!camera_->initialize()) {
        std::cout << "There is a problem to initialize the RGBD Camera" << std::endl;
        return -1;
    }

    std::cout << "Opening camera... Done" << std::endl;

    // --------------------------------------------------------------
    // Declarations
    // --------------------------------------------------------------

    // Declare the map
    crf::applications::mapper3d::Mapper3d map(vm["mapperConfigFile"].as<std::string>());

    // Declare and init the graph
    crf::applications::graphoptimization::GraphOptimization graph(
        vm["mapperConfigFile"].as<std::string>());
    if (!graph.parse(vm["graphConfigFile"].as<std::string>())) {
        std::cout << "Bad input configFile" << std::endl;
        return 0;
    }

    // Input data
    // auto imageNumber =  vm["maxIterations"].as<unsigned int>();

    // Read configuration names from .json of the noise
    std::string configFileNoise(vm["graphConfigFile"].as<std::string>());
    std::ifstream configNoise(configFileNoise);
    nlohmann::json jConfigNoise;
    configNoise >> jConfigNoise;
    bool visualEnable = jConfigNoise.at("visualization").get<bool>();
    // double threshold = jConfigNoise.at("threshold").at("value").get<double>();*/

    // Time declaration
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds;

    // Take parameters of the camera
    float scale_ = camera_->getDepthScale2Meters();
    rs2_extrinsics extrinsics_ = camera_->getDepth2ColorExtrinsics();
    rs2_intrinsics depthIntrinsics_ = camera_->getDepthIntrinsics();
    rs2_intrinsics colorIntrinsics_ = camera_->getColorIntrinsics();

    // Motion matrix
    Eigen::Matrix4f SupposedMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previousMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f RealMotion = Eigen::Matrix4f::Identity();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr previousPointCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>());

    // --------------------------------------------------------------
    // Graph generation
    // --------------------------------------------------------------
    unsigned int id_vertex = 0;
    std::vector<double> odometryX;
    std::vector<double> odometryZ;

    crf::utility::types::TaskPose robotPosition;
    crf::utility::types::TaskPose previousRobotPosition;

    std::cout << "Generating graph..." << std::endl;
    while ((id_vertex < vm["maxIterations"].as<unsigned int>()) && (gSignalStatus != SIGKILL)) {
        // ----------------------------------------------------------
        // 0 - Position estimation
        // ----------------------------------------------------------
        char paso;
        std::cin >> paso;

        robotPosition = localizer_->getPosition().get();
        if (id_vertex == 0 )
            previousRobotPosition = robotPosition;

        Eigen::Vector3d odometryPosition;
        odometryPosition << robotPosition(1), robotPosition(2), robotPosition(0);

        Eigen::AngleAxisd rollAngle(robotPosition(4), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(robotPosition(5), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(robotPosition(3), Eigen::Vector3d::UnitZ());
        Eigen::Quaternion<double> rotVector = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix3d rotationMatrix = rotVector.matrix();

        SupposedMotion <<
            rotationMatrix(0, 0), rotationMatrix(0, 1), rotationMatrix(0, 2), robotPosition(1),
            rotationMatrix(1, 0), rotationMatrix(1, 1), rotationMatrix(1, 2), robotPosition(2),
            rotationMatrix(2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2), robotPosition(0),
            0, 0, 0, 1;

        std::cout << "ODOMETRY (Vector): " << robotPosition << std::endl;
        std::cout << "ODOMETRY (Matrix): " << std::endl << SupposedMotion << std::endl;
        previousRobotPosition = robotPosition;
        odometryX.push_back(robotPosition(1));
        odometryZ.push_back(robotPosition(0));

        // ----------------------------------------------------------
        // 1 - Point adquisition
        // ----------------------------------------------------------

        // Start the chrono
        std::cout << "1 - Point adquisition... ";
        start = std::chrono::system_clock::now();

        // Take the images
        cv::Mat color = camera_->getColorFrame();
        cv::Mat depth = camera_->getDepthFrame();

        // Generate point cloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud(
            new pcl::PointCloud<pcl::PointXYZRGBA>());
        *inPointCloud = crf::sensors::rgbdcamera::RGBDUtils::getPointCloud(color, depth,
            scale_, extrinsics_, depthIntrinsics_, colorIntrinsics_, false);

        std::cout << "Done" << std::endl;
        end = std::chrono::system_clock::now();
        elapsedMilliseconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
        std::cout << "1 - Point adquisition execution time: " << elapsedMilliseconds << std::endl;

        // ----------------------------------------------------------
        // 2 - Comparation with the previous point cloud
        // ----------------------------------------------------------

        // Start the chrono
        std::cout << "2 - Comparation with the previous point cloud... " << std::endl;
        start = std::chrono::system_clock::now();

        // Update map and take the matrix of motion
        boost::optional<Eigen::Matrix4f> boostRealMotion;
        previousMotion = RealMotion;

        if (id_vertex != 0) {
            boostRealMotion = map.comparePointClouds(inPointCloud, previousPointCloud,
                SupposedMotion, RealMotion);

            if (!boostRealMotion) {
                std::cout << "Unable to update the map" <<std::endl;
                return -1;
            }

            RealMotion = boostRealMotion.get();
            // SupposedMotion = RealMotion;
            std::cout << "Real motion: " << RealMotion << std::endl;
        }

        std::cout << "2 - Comparation with the previous point cloud... Done" << std::endl;
        end = std::chrono::system_clock::now();
        elapsedMilliseconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
        std::cout << "2 - Comparation with the previous point cloud execution time: "
            << elapsedMilliseconds << std::endl;

        // ----------------------------------------------------------
        // 3 - Add the new vertex to the graph
        // 4 - Calculation of the K NN
        // 5 - PC comparation with the K NN (less with the previous one)
        // 6 - Add the edges to the graph
        // 7 - Pose estimation of the new pose
        // 8 - Add the new pose to the kdtree
        // ----------------------------------------------------------

        Eigen::Vector3f fixPose;
        bool fix = false;
        if (id_vertex < 1) {
            fixPose << RealMotion(0, 3), RealMotion(1, 3), RealMotion(2, 3);
            fix = true;
        } else if (id_vertex == vm["maxIterations"].as<unsigned int>()-1) {
            double gtX, gtY, gtZ;
            std::cout << "Last position X: ";
            std::cin >> gtX;
            std::cout << "Last position Y: ";
            std::cin >> gtY;
            std::cout << "Last position Z: ";
            std::cin >> gtZ;
            fixPose << gtX, gtY, gtZ;
            fix = true;
        } else {
            fixPose << 0, 0, 0;
        }

        // Add the vertex to the graph
        if (!graph.addVertex(RealMotion, previousMotion, fix, fixPose, inPointCloud,
            odometryPosition, odometryPosition, nullopt, nullopt, nullopt)) {
            std::cout << "Unable to create the vertex" << std::endl;
            return -1;
        }
        previousPointCloud = inPointCloud;
        SupposedMotion = RealMotion;
        id_vertex++;
    }

    std::cout << "Generating graph... Done" << std::endl;

    // Save the graph
    if (!graph.saveGraph(vm["resultName"].as<std::string>())) {
        std::cout << "There was a problem saving the graph" << std::endl;
        return -1;
    }

    // Show the results
    if (!graph.saveResults(vm["resultName"].as<std::string>())) {
        std::cout << "There was a problem showing the graph" << std::endl;
        return -1;
    }

    // Obtain data from graph
    boost::optional<std::vector <pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>> boostNodePointCloud =
        graph.getNodePointclouds();
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

    // Write output file
    ofstream fs("results.txt");
    double realPosX, realPosZ;
    for (size_t i = 0; i < odometryX.size(); i++) {
        Eigen::Matrix4f auxrealPos = nodePosition[i];
        std::cout << auxrealPos << std::endl << std::endl;
        realPosX = auxrealPos(0, 3);
        realPosZ = auxrealPos(2, 3);
        fs << odometryX[i] << " " << odometryZ[i] << " " << realPosX << " " << realPosZ
            << std::endl;
    }

    if (visualEnable) {
        // Open thread for the visualization
        std::thread viz(&Visualization);

        // Visualization after optimize the graph
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
            mutex_.lock();
            updateMap_ = true;
            pcl::copyPointCloud(*subsampledCloud, *cloud_);
            mutex_.unlock();
        }

        // Next iteration
        char next;
        std::cin >> next;
        // --------------------------------------------------------------
        // Finishing work
        // --------------------------------------------------------------

        stopViz_ = true;
        viz.join();
    }
    return 0;
}
