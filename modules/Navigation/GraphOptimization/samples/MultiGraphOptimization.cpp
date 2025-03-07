/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

// CRF packages
#include "RGBDCamera/RealSense2Grabber.hpp"
#include "RGBDCamera/IRGBDCamera.hpp"
#include "RGBDCamera/RGBDUtils.hpp"
#include "RGBDVisionUtility/PCLUtils.hpp"
#include "Mapper3d/Mapper3d.hpp"

#include "GraphOptimization/GraphOptimization.hpp"
#include "GraphOptimization/GraphOptimizationConfiguration.hpp"

// PCL packages
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>

// Generig libraries
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>
#include <boost/optional.hpp>
#include <string>
#include <memory>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

using crf::applications::graphoptimization::GraphOptimizationConfiguration;
using crf::applications::graphoptimization::Simulations;
using crf::applications::graphoptimization::SlamParameters;

std::mutex mutex_;

int main(int argc, char **argv) {
    // --------------------------------------------------------------
    // Input management
    // --------------------------------------------------------------
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("mapperConfigFile", po::value<std::string>(),
            "[1] Path to configuration Json file of the environment)")
        ("graphConfigFile", po::value<std::string>(),
            "[2] Path to configuration Json file of the graph")
        ("cameraConfigFile", po::value<std::string>(), "[3] RGBDCamera config file")
        ("resultName", po::value<std::string>(), "[4] Result files name")
        ("maxIterations", po::value<int>(), "[5] Number of images");

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

    if (!vm.count("cameraConfigFile")) {
        std::cout << "Missing lidar configuration file" << std::endl << std::endl;
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
    // Open camera
    // --------------------------------------------------------------
    std::cout << "Opening camera... " << std::endl;
    std::string configFileCamera(vm["cameraConfigFile"].as<std::string>());
    std::ifstream configCamera(configFileCamera);
    nlohmann::json jConfigCamera;
    configCamera >> jConfigCamera;
    auto camera = std::make_shared<crf::sensors::rgbdcamera::RealSense2Grabber>
        (jConfigCamera.at("RealSense"));
    std::shared_ptr<crf::sensors::rgbdcamera::IRGBDCamera> camera_(camera);
    if (!camera_->initialize()) {
        std::cout << "There is a problem to initialize the RGBD Camera" << std::endl;
        return false;
    }

    std::cout << "Opening camera... Done" << std::endl;

    // --------------------------------------------------------------
    // Declarations
    // --------------------------------------------------------------

    // Declare and init the graph
    crf::applications::graphoptimization::GraphOptimization graph(
        vm["mapperConfigFile"].as<std::string>());
    if (!graph.parse(vm["graphConfigFile"].as<std::string>())) {
        std::cout << "Bad input configFile" << std::endl;
        return 0;
    }

    // Input data
    int imageNumber = vm["maxIterations"].as<int>();

    // Take configuration data
    boost::optional <Simulations> boostSimulationsData = graph.getSimulationsData();
    boost::optional <SlamParameters> boostSlamData = graph.getSlamData();
    Simulations simulationsData = boostSimulationsData.get();
    SlamParameters slamData = boostSlamData.get();

    double xNoise = simulationsData.noiseX;
    double yNoise = simulationsData.noiseY;
    double zNoise = simulationsData.noiseZ;
    bool visualEnable = slamData.visualization;

    // Time declaration
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};

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
    int id_vertex = 0;
    double newX {0.0}, newY {0.0}, newZ {0.0};
    double odoX {0.0}, odoY {0.0}, odoZ {0.0};

    std::cout << "Generating graph..." << std::endl;
    while (id_vertex < imageNumber) {
        // New position
        double gtX {0.0}, gtY {0.0}, gtZ {0.0};
        Eigen::Vector3f groundTruthPosition;
        std::cout << "Real position X: ";
        std::cin >> gtX;
        std::cout << "Real position Y: ";
        std::cin >> gtY;
        std::cout << "Real position Z: ";
        std::cin >> gtZ;
        groundTruthPosition << gtX, gtY, gtZ;

        if (id_vertex != 0) {
            newX += gtX - odoX + ((std::rand() % 100)*xNoise/100 - xNoise/2);
            newY += gtY - odoY + ((std::rand() % 100)*yNoise/100 - yNoise/2);
            newZ += gtZ - odoZ + ((std::rand() % 100)*zNoise/100 - zNoise/2);
        }

        std::cout << "ODOMETRY: " << newX << " " << newY << " " << newZ << std::endl;

        Eigen::Vector3f odometryPosition;
        SupposedMotion(0, 3) = newX;
        SupposedMotion(1, 3) = newY;
        SupposedMotion(2, 3) = newZ;
        odometryPosition << newX, newY, newZ;

        odoX = gtX;
        odoY = gtY;
        odoZ = gtZ;

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

        // Add the vertex to the graph
        Eigen::Vector3f fixPose;
        if ((id_vertex == imageNumber-1) || (id_vertex == 0)) {
            fixPose << gtX, gtY, gtZ;
            if (!graph.addVertex(RealMotion, previousMotion, fixPose, odometryPosition,
                groundTruthPosition)) {
                std::cout << "Unable to create the vertex" << std::endl;
                return -1;
            }
        } else {
            if (!graph.addVertex(RealMotion, previousMotion, boost::none, odometryPosition,
                groundTruthPosition)) {
                std::cout << "Unable to create the vertex" << std::endl;
                return -1;
            }
        }

        // Next iteration
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

    if (visualEnable) {
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

        pcl::visualization::CloudViewer viewer("Viewer");
        viewer.showCloud(cloud_);

        // Next iteration
        char next {};
        std::cin >> next;
    }
}
