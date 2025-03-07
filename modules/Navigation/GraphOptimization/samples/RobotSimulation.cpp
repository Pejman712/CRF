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

// PCL packages
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>

// Generig libraries
#include <Eigen/LU>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <nlohmann/json.hpp>
#include <boost/optional.hpp>
#include <string>
#include <memory>

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
    if (argc < 6) {
        std::cout << "Few arguments provide:" <<std::endl;
        std::cout << " [1] Path to RGBD Camera config file" << std::endl;
        std::cout << " [2] Path to configuration Json file of the environment" << std::endl;
        std::cout << " [3] Path to configuration Json file of the graph" << std::endl;
        std::cout << " [4] Number of images to be taken" << std::endl;
        std::cout << " [5] Result files name" << std::endl;
        return -1;
    }

    // --------------------------------------------------------------
    // Open camera
    // --------------------------------------------------------------
    std::cout << "Opening camera... " << std::endl;
    std::string configFileCamera(argv[1]);
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

    // Declare the map
    crf::applications::mapper3d::Mapper3d map(argv[2]);

    // Declare and init the graph
    crf::applications::graphoptimization::GraphOptimization graph(argv[2]);
    if (!graph.parse(argv[3])) {
        std::cout << "Bad input configFile" << std::endl;
        return 0;
    }

    // Input data
    int imageNumber = atoi(argv[4]);

    // Read configuration names from .json of the noise
    std::string configFileGraph(argv[3]);
    std::ifstream configGraph(configFileGraph);
    nlohmann::json jconfigGraph;
    configGraph >> jconfigGraph;
    double xNoise = jconfigGraph.at("odometrySimulation").at("noise").at("x").get<double>();
    double yNoise = jconfigGraph.at("odometrySimulation").at("noise").at("y").get<double>();
    double zNoise = jconfigGraph.at("odometrySimulation").at("noise").at("z").get<double>();
    double speedX = jconfigGraph.at("odometrySimulation").at("speed").at("x").get<double>();
    double speedY = jconfigGraph.at("odometrySimulation").at("speed").at("y").get<double>();
    double speedZ = jconfigGraph.at("odometrySimulation").at("speed").at("z").get<double>();
    bool visualEnable = jconfigGraph.at("visualization").get<bool>();

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
    int id_vertex = 0;
    double odoX = -speedX, odoY = -speedY, odoZ = -speedZ;
    double gtX, gtY, gtZ;

    std::cout << "Saving point clouds..." << std::endl;
    while (id_vertex < imageNumber) {
        // New odometry position
        odoX += speedX;
        odoY += speedY;
        odoZ += speedZ;

        std::cout << "ODOMETRY: " << odoX << " " << odoY << " " << odoZ << std::endl;

        Eigen::Vector3d odometryPosition;
        SupposedMotion(0, 3) = odoX;
        SupposedMotion(1, 3) = odoY;
        SupposedMotion(2, 3) = odoZ;
        odometryPosition << odoX, odoY, odoZ;

        if (id_vertex == imageNumber-1) {
            std::cout << "Final position. X: ";
            std::cin >> gtX;
            std::cout << "Final position. Y: ";
            std::cin >> gtY;
            std::cout << "Final position. Z: ";
            std::cin >> gtZ;
        }

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

        Eigen::Vector3f fixPose;
        bool fix = false;
        if (id_vertex == 0) {
            fixPose << 0, 0, 0;
            fix = true;
        } else if (id_vertex == imageNumber-1) {
            fixPose << gtX, gtY, gtZ;
            fix = true;
        }

        // Add the vertex to the graph
        if (!graph.addVertex(RealMotion, previousMotion, fix, fixPose, inPointCloud,
            odometryPosition, odometryPosition)) {
            std::cout << "Unable to create the vertex" << std::endl;
            return -1;
        }

        // Next iteration
        id_vertex++;
    }

    std::cout << "Generating graph... Done" << std::endl;

    // Save the graph
    if (!graph.saveGraph(argv[5])) {
        std::cout << "There was a problem saving the graph" << std::endl;
        return -1;
    }

    // Show the results
    if (!graph.saveResults(argv[5])) {
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
}
