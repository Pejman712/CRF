/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

// Mapper and RGBD libraries
#include "RGBDVisionUtility/PCLUtils.hpp"
#include "Mapper3d/Mapper3d.hpp"
#include "Types/Types.hpp"

// GraphSLAM libraries
#include "GraphOptimization/GraphOptimization.hpp"
#include "GraphOptimization/GraphOptimizationConfiguration.hpp"

// PCL libraries
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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
#include <thread>

using crf::applications::graphoptimization::GraphOptimizationConfiguration;
using crf::applications::graphoptimization::SlamParameters;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {  // NOLINT
    viewer.setBackgroundColor(0.9, 0.9, 1.0);
}

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signalHandler(int signal) {
    gSignalStatus = signal;
}
}  // unnamed namespace

int main(int argc, char** argv) {
    // --------------------------------------------------------------
    // Input management
    // --------------------------------------------------------------
    if (argc < 3) {
        std::cout << "Few arguments provide:" <<std::endl;
        std::cout << "  [1] Slam configuration file" << std::endl;
        std::cout << "  [2] Dataset folder" << std::endl << std::endl;
        std::cout << "  Help for Slam configuration file:" << std::endl;
        std::cout << "   - mapperConfigFile: Path to configuration Json file of the environment" << std::endl; // NOLINT
        std::cout << "   - graphConfigFile: Path to configuration Json file of the graph" << std::endl; // NOLINT
        std::cout << "   - resultName: Result files name" << std::endl;
        return -1;
    }

    std::signal(SIGINT, signalHandler);

    std::ifstream config(argv[1]);
    nlohmann::json jConfig;
    config >> jConfig;

    std::string mapperConfigFile_ = jConfig.at("mapperConfigFile").get<std::string>();
    std::string graphConfigFile_ = jConfig.at("graphConfigFile").get<std::string>();
    std::string resultName_ = jConfig.at("resultName").get<std::string>();

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

    SlamParameters slamData = boostSlamData.get();
    Eigen::Matrix4f tranformationData = boostTranformationData.get();

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

    unsigned int id_vertex {0};

    crf::utility::types::TaskPose robotPosition;
    crf::utility::types::TaskPose previousRobotPosition;

    // --------------------------------------------------------------
    // Read dataset
    // --------------------------------------------------------------
    // Save Point Cloud
    std::string saveName = argv[2];
    std::string pointCloudNameBase{saveName};
    pointCloudNameBase += "pc/pc";

    std::string infoFileName{saveName};
    infoFileName += "info.json";

    // Read data file
    std::ifstream dataFile(infoFileName);
    nlohmann::json jdataFile;
    unsigned int maxIterations_{0};

    if ((dataFile.rdstate() & std::ifstream::failbit) != 0) {
        std::cout << std::endl << "Unable to read the file";
        return -1;
    }
    try {
        dataFile >> jdataFile;
        maxIterations_ = jdataFile.at("numPoints").get<unsigned int>();
        std::vector<float> auxRobotPosition{};
        for (int i = 0; i < 6; i++) {
            auxRobotPosition.push_back(jdataFile.at("poses").at("0").at("pose")[i].get<float>());
        }
        robotPosition = auxRobotPosition;
    } catch (const std::exception& e) {
        std::cout << std::endl << "Failed to parse dataset data because: " << e.what();
        return -1;
    }

    Eigen::Matrix4f initialPose = robotPosition.eigenMatrix();

    // --------------------------------------------------------------
    // Graph generation
    // --------------------------------------------------------------
    std::cout << "Generating graph..." << std::endl;
    float time_{0.0};
    while ((id_vertex < maxIterations_) && (gSignalStatus != SIGINT)) {
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

        // Take information from the dataset
        try {
            // dataFile >> jdataFile;
            time_ = jdataFile.at("poses").at(std::to_string(id_vertex)).at("time").get<float>();
            std::vector<float> auxRobotPosition{};
            for (int i = 0; i < 6; i++) {
                auxRobotPosition.push_back(jdataFile.at("poses").at(std::to_string(id_vertex)).at(
                    "pose").at(i).get<float>());
            }
            robotPosition = auxRobotPosition;
        } catch (const std::exception& e) {
            std::cout << "Failed to get point " << id_vertex << " because: " << e.what() << "\n";
            return -1;
        }

        std::cout << "Pose estimated: " << robotPosition << std::endl;

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
            std::cout << "2 - Reading PointCloud from file... " << std::endl;
            start = std::chrono::system_clock::now();

            // Generate point cloud
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud
                (new pcl::PointCloud<pcl::PointXYZRGBA>());
            std::string pointCloudName{pointCloudNameBase};
            pointCloudName += std::to_string(id_vertex);
            pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pointCloudName, *inPointCloud);

            std::cout << "Reading PointCloud from file... Done" << std::endl;
            end = std::chrono::system_clock::now();
            elapsedMilliseconds =
                std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
            std::cout << "2 - Reading PointCloud from file execution time: " <<
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
        }

        id_vertex++;
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
