/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

// CRF packages
#include "RGBDVisionUtility/PCLUtils.hpp"
#include "Mapper3d/Mapper3d.hpp"
#include "Laser/VelodyneHDL/VelodyneHDLLaserPCL.hpp"
#include "GraphOptimization/GraphOptimization.hpp"
#include "GraphOptimization/GraphOptimizationConfiguration.hpp"

// PCL libraries
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/io/pcd_io.h>

// Generig libraries
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <boost/optional.hpp>
#include <string>
#include <memory>
#include <math.h>

#define PI 3.14159265

using crf::applications::graphoptimization::GraphOptimizationConfiguration;
using crf::applications::graphoptimization::Simulations;
using crf::applications::graphoptimization::SlamParameters;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {  // NOLINT
    viewer.setBackgroundColor(0.9, 0.9, 1.0);
}

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
    std::string sensorConfigFile_ = jConfig.at("sensorConfigFile").get<std::string>();
    std::string resultName_ = jConfig.at("resultName").get<std::string>();
    unsigned int maxIterations_ = jConfig.at("maxIterations").get<unsigned int>();

    // --------------------------------------------------------------
    // Open LIDAR
    // --------------------------------------------------------------
    std::cout << "Opening velodyne... " << std::endl;
    std::ifstream laserData(sensorConfigFile_);
    nlohmann::json laserJSON = nlohmann::json::parse(laserData);
    crf::sensors::laser::VelodyneHDLLaserPCL velo(laserJSON);

    if (!velo.initialize()) {
        std::cout << "Unable to initialize velodyne" << std::endl;
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
    boost::optional <Simulations> boostSimulationsData = graph.getSimulationsData();
    boost::optional <SlamParameters> boostSlamData = graph.getSlamData();
    boost::optional <std::vector <std::vector <float>>> boostDoorsPosition =
        graph.getDoorsPosition();
    Simulations simulationsData = boostSimulationsData.get();
    SlamParameters slamData = boostSlamData.get();
    std::vector <std::vector <float>> doorsPosition = boostDoorsPosition.get();
    std::cout << std::endl << "Doors position: " << std::endl;
    for (int i = 0; i < doorsPosition.size(); i++) {
        std::cout << "Door " << i << ": (" << doorsPosition[i][0] << ", " << doorsPosition[i][1] <<
            ", " << doorsPosition[i][2] << ")" << std::endl;
    }

    float xNoise = simulationsData.noiseX;
    float yNoise = simulationsData.noiseY;
    float zNoise = simulationsData.noiseZ;
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

    // --------------------------------------------------------------
    // Graph generation
    // --------------------------------------------------------------
    unsigned int id_vertex {0};
    float newX {0.0}, newY {0.0}, newZ {0.0};
    float odoX {0.0}, odoY {0.0}, odoZ {0.0};

    std::cout << "Generating graph..." << std::endl;
    while (id_vertex < maxIterations_) {
        // New position
        float gtX {0.0}, gtY {0.0}, gtZ {0.0}, ori{0.0};
        Eigen::Vector3f groundTruthPosition;
        std::cout << "Real position X: ";
        std::cin >> gtX;
        std::cout << "Real position Y: ";
        std::cin >> gtY;
        std::cout << "Real position Z: ";
        std::cin >> gtZ;
        std::cout << "Orientation in degrees: ";
        std::cin >> ori;
        groundTruthPosition << gtX, gtY, gtZ;

        if (id_vertex != 0) {
            newX += gtX - odoX + ((std::rand() % 100)*xNoise/100 - xNoise/2);
            newY += gtY - odoY + ((std::rand() % 100)*yNoise/100 - yNoise/2);
            newZ += gtZ - odoZ + ((std::rand() % 100)*zNoise/100 - zNoise/2);
        }

        std::cout << "ODOMETRY: " << newX << " " << newY << " " << newZ << std::endl;

        Eigen::Vector3f odometryPosition;
        SupposedMotion(0, 0) = static_cast<float>(cos(static_cast<double>(ori) * PI / 180.0));
        SupposedMotion(0, 1) = static_cast<float>(-sin(static_cast<double>(ori) * PI / 180.0));
        SupposedMotion(0, 2) = 0;
        SupposedMotion(1, 0) = static_cast<float>(sin(static_cast<double>(ori) * PI / 180.0));
        SupposedMotion(1, 1) = static_cast<float>(cos(static_cast<double>(ori) * PI / 180.0));
        SupposedMotion(1, 2) = 0;
        SupposedMotion(2, 0) = 0;
        SupposedMotion(2, 1) = 0;
        SupposedMotion(2, 2) = 1;
        SupposedMotion(3, 0) = 0;
        SupposedMotion(3, 1) = 0;
        SupposedMotion(3, 2) = 0;
        SupposedMotion(3, 3) = 1;
        SupposedMotion(0, 3) = newX;
        SupposedMotion(1, 3) = newY;
        SupposedMotion(2, 3) = newZ;
        odometryPosition << newX, newY, newZ;
        std::cout << "SupossedMotion: " << std::endl << SupposedMotion << std::endl;

        odoX = gtX;
        odoY = gtY;
        odoZ = gtZ;

        // ----------------------------------------------------------
        // 1 - Point adquisition
        // ----------------------------------------------------------

        // Start the chrono
        std::cout << "1 - Point adquisition... ";
        start = std::chrono::system_clock::now();

        // Generate point cloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud
            (new pcl::PointCloud<pcl::PointXYZRGBA>());
        inPointCloud = velo.getPointCloud();
        // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud = velo.getPointCloud();

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
        if ((id_vertex == maxIterations_ - 1) || (id_vertex == 0)) {
            std::cout << "Plancing fixed vertex " << id_vertex << std::endl;
            if (!graph.addVertex(RealMotion, previousMotion, groundTruthPosition, odometryPosition,
                groundTruthPosition)) {
                std::cout << "Unable to create the vertex" << std::endl;
                return -1;
            }
        } else {
            std::cout << "Plancing freely vertex " << id_vertex << std::endl;
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
    if (!graph.saveGraph(resultName_)) {
        std::cout << "There was a problem saving the graph" << std::endl;
        return -1;
    }

    // Show the results
    if (!graph.saveResults(resultName_)) {
        std::cout << "There was a problem showing the graph" << std::endl;
        return -1;
    }

    if (visualEnable) {
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

        pcl::visualization::CloudViewer viewer("Viewer");
        viewer.showCloud(cloud_);

        viewer.runOnVisualizationThreadOnce(viewerOneOff);

        // Next iteration
        char next {};
        std::cin >> next;
    }
}
