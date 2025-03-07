/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include "Mapper3d/HandHeldMapper3d.hpp"
#include "CameraViewer/CameraViewer.hpp"
#include "RGBDCamera/RealSense2Grabber.hpp"
#include "RGBDCamera/IRGBDCamera.hpp"
#include "RGBDCamera/RGBDUtils.hpp"

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <thread>
#include <mutex>
#include <nlohmann/json.hpp>
#include <visp3/vision/vpPose.h>
#include <boost/optional.hpp>
#include <memory>
#include <string>
#include <vector>
#include <sys/time.h>

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
Eigen::Affine3f affinePose_;
bool stopViz_ = false;
std::mutex mutex_;

typedef std::array<float, 6> JointPositions;

void visualization() {
    crf::applications::cameraviewer::CameraViewer viewer("HandHeldScannerOffline");

    while (!stopViz_) {
        mutex_.lock();
        if (cloud_->size() > 0) {
            viewer.viewPointCloud(*cloud_, "RealSense");
            viewer.viewCoordinateSystem(affinePose_, "RealSense");
        }
        mutex_.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Few arguments provide:" <<std::endl;
        std::cout << "  [1] Path to HandHeldMapper3d configuration Json file" << std::endl;
        std::cout << "  [2] Path to RGBD Camera config file" << std::endl;
        std::cout << "  [3] Number of scans to be taken" << std::endl;
        return -1;
    }

    crf::applications::mapper3d::HandHeldMapper3d merger(argv[1]);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud_ = cloud;
    std::thread viz(&visualization);

    // Store pointclouds
    int iterations = 0;
    int maxIterations = atoi(argv[3]);
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> pcVector;

    std::string configFile(argv[2]);
    std::ifstream config(configFile);
    nlohmann::json jConfig;
    config >> jConfig;
    auto camera =
        std::make_shared<crf::sensors::rgbdcamera::RealSense2Grabber>(jConfig.at("RealSense"));
    std::shared_ptr<crf::sensors::rgbdcamera::IRGBDCamera> camera_(camera);
    if (!camera_->initialize()) {
        std::cout << "There is a problem to initialize the RGBD Camera" << std::endl;
        return false;
    }
    rs2_extrinsics extrinsics_ = camera_->getDepth2ColorExtrinsics();
    rs2_intrinsics depthIntrinsics_ = camera_->getDepthIntrinsics();
    rs2_intrinsics colorIntrinsics_ = camera_->getColorIntrinsics();
    float scale_ = camera_->getDepthScale2Meters();

    std::cout << " WAITING 5 SECONDS TO BEGIN SCAN" << std::endl;
    sleep(5);

    while (iterations < maxIterations) {
        // Capture pointcloud
        cv::Mat color = camera_->getColorFrame();
        cv::Mat depth = camera_->getDepthFrame();
        pcl::PointCloud<pcl::PointXYZRGBA> inPointCloud =
            crf::sensors::rgbdcamera::RGBDUtils::getPointCloud(color, depth, scale_, extrinsics_,
            depthIntrinsics_, colorIntrinsics_);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloudPtr(
            new pcl::PointCloud<pcl::PointXYZRGBA>(inPointCloud));
        pcVector.push_back(inPointCloudPtr);
        std::cout << "PointCloud " << iterations << " acquired" <<std::endl;

        iterations++;
    }

    Eigen::Matrix4f mappingCameraPose;
    mappingCameraPose.setIdentity();
    // Merge pointclouds offline
    iterations = 0;
    while (iterations < maxIterations) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud = pcVector[iterations];
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outPointCloud
            (new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outPointCloud2
            (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
            emptyCloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcVector[iterations] = emptyCloud;

        // Update map
        Eigen::Matrix4f finalCameraPose;
        boost::optional<Eigen::Matrix4f> boostFinalCameraPose;
        boostFinalCameraPose = merger.updateMap(inPointCloud, mappingCameraPose);
        if (boostFinalCameraPose == boost::none) {
            std::cout << "Unable to update the map" <<std::endl;
        } else {
            finalCameraPose = boostFinalCameraPose.get();
        }

        outPointCloud2 = merger.getPointCloudMap().get();
        pcl::copyPointCloud(*outPointCloud2, *outPointCloud);
        std::cout << "waiting for lock" << std::endl;
        mutex_.lock();
        cloud_ = outPointCloud;
        affinePose_.matrix() = finalCameraPose;
        mutex_.unlock();
        std::cout << "mutex_ unlock " << std::endl;
        iterations++;
    }

    merger.savePointCloudToDisk("debugPC", false);

    // If the letter 'q' is pressed the program process this frame pointcloud
    char key;
    bool endflag = false;
    std::cout << "KEYBOARD CONTROL"<< std::endl;
    while (!endflag) {
        std::cout << "Press 'q'+ ENTER to exit" << std::endl;
        std::cin >> key;
        if (key == 'q') endflag = true;
    }

    stopViz_ = true;
    viz.join();

    return 0;
}

