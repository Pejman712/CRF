/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales & Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include <mutex>
#include <thread>
#include <memory>
#include <vector>
#include <string>
#include <future>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <nlohmann/json.hpp>
#include <visp3/vision/vpPose.h>
#include <boost/optional.hpp>

#include "EventLogger/EventLogger.hpp"
#include "KinovaArm/KinovaJaco.hpp"
#include "KinovaArm/KinovaApiInterface.hpp"
#include "RobotArm/RobotArmDefaultKinematics.hpp"
#include "ClosedLoopController/PIDController.hpp"
#include "RobotArmControllersDeprecated/RobotArmDefaultController.hpp"
#include "Types/JointVelocities.hpp"
#include "Types/JointPositions.hpp"
#include "Types/TaskTypes/TaskPose.hpp"
#include "RGBDCamera/RealSense2Grabber.hpp"
#include "RGBDCamera/IRGBDCamera.hpp"
#include "RGBDCamera/RGBDUtils.hpp"
#include "Mapper3d/Mapper3d.hpp"

bool stopVisualizer_ = false;
std::mutex mutex_;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
Eigen::Affine3f affinePose_;

void visualization() {
    pcl::visualization::PCLVisualizer mapViewer("Map Viewer");
    mapViewer.setBackgroundColor(0.0, 0.0, 0.0);
    Eigen::Matrix4f identity;
    identity.setIdentity();
    Eigen::Affine3f base;
    base.matrix() = identity;
    mapViewer.addCoordinateSystem(0.25, base , "Base");
    while (!stopVisualizer_) {
        mutex_.lock();
        if (cloud_->size() > 0) {
            mapViewer.addPointCloud(cloud_, "Map");
            mapViewer.addCoordinateSystem(0.25, affinePose_, "RealSense");
            mapViewer.spinOnce();
            mapViewer.removeAllPointClouds();
            mapViewer.removeCoordinateSystem("RealSense");
        }
        mutex_.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char **argv) {
    crf::utility::logger::EventLogger logger("Rs2KinovaHandEye");

    if (argc != 6) {
        logger->error("The number of arguments is not correct");
        return -1;
    }

    // Initialize the Kinova
    std::ifstream robotData(argv[1]);
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    auto arm = std::make_shared<crf::robots::kinovaarm::KinovaJaco>(
        std::make_shared<crf::robots::kinovaarm::KinovaApiInterface>(),
        robotJSON);
    if (!arm->initialize()) {
        logger->error("Failed to initialize the robot arm ");
    }
    auto kinematics = std::make_shared<crf::robots::robotarm::RobotArmDefaultKinematics>(
        arm->getConfiguration());
    size_t jointsNumber = arm->getConfiguration()->getNumberOfJoints();
    auto pid = std::make_shared<crf::algorithms::closedloopcontroller::PIDController>(
        std::vector<float>(jointsNumber, 1.0f),
        std::vector<float>(jointsNumber, 0.3f),
        std::vector<float>(jointsNumber, 1.0f));
    auto controller =
        std::make_shared<crf::applications::robotarmcontroller::RobotArmDefaultController>(
            arm,
            kinematics,
            pid);
    if (!controller->initialize()) {
        logger->error("Failed to initialize the controller ");
    }
    crf::utility::types::JointVelocities maxVelocity({0.05, 0.05, 0.05, 0.05, 0.05, 0.05});
    controller->setJointsMaximumVelocity(maxVelocity);
    logger->info("Kinova initialize");

    // Initialize the camera
    std::string configFileCamera(argv[2]);
    std::ifstream configCamera(configFileCamera);
    nlohmann::json configCameraJSON;
    configCamera >> configCameraJSON;
    auto camera = std::make_shared<crf::sensors::rgbdcamera::RealSense2Grabber>(
        configCameraJSON.at("RealSense"));
    if (!camera->initialize()) {
        logger->info("There is a problem to initialize the RGBD Camera");
        return -1;
    }
    logger->info("Camera initialize");

    // Get camera parameters and HandEye calibration
    float scale = camera->getDepthScale2Meters();
    rs2_extrinsics extrinsicsParam = camera->getDepth2ColorExtrinsics();
    rs2_intrinsics depthIntrinsicsParam = camera->getDepthIntrinsics();
    rs2_intrinsics colorIntrinsicsParam = camera->getColorIntrinsics();
    vpHomogeneousMatrix vispHandEyeTransMatrix;
    Eigen::Matrix4f eigenHandEyeTransMatrix;
    std::ifstream file(argv[3]);
    vispHandEyeTransMatrix.load(file);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            eigenHandEyeTransMatrix(i, j) = vispHandEyeTransMatrix[i][j];
        }
    }

    // Get the path to follow from the JSON file
    std::vector<crf::utility::types::JointPositions> path;
    std::ifstream config(argv[4]);
    nlohmann::json pathConfig;
    config >> pathConfig;
    int pathSize = pathConfig.at("positionsSize").get<int>();
    logger->info("There size of the path is {}", pathSize);
    for (int i = 0; i < pathSize; i++) {
        crf::utility::types::JointPositions position(6);
        for (int j = 0; j < 6; j++) {
            std::string posNumber = "position" + std::to_string(i);
            std::string joint = "joint" + std::to_string(j);
            float jointPositions = pathConfig.at("positions").at(posNumber).at(joint).get<float>();
            position(j) = jointPositions;
        }
        path.push_back(position);
    }
    logger->info("There path was store correctly");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud_ = cloud;
    std::thread visualizer(&visualization);
    crf::applications::mapper3d::Mapper3d mapper(argv[5]);

    // Execute the path
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::future<bool> future = controller->executeJointsTrajectory(path);
    logger->info("Executing path");
    std::future_status status;
    do {
        status = future.wait_for(std::chrono::milliseconds(10));
        if (status == std::future_status::deferred) {
            logger->info("Deferred");
        } else if (status == std::future_status::timeout) {
            logger->info("Timeout");
            // Extract actual robot position
            crf::utility::types::TaskPose robotPose =
                controller->getTaskPose();
            Eigen::Matrix4f robotPoseEigen;
            robotPoseEigen(0, 0) = robotPose.rotationAndTranslationMatrix()[0];
            robotPoseEigen(0, 1) = robotPose.rotationAndTranslationMatrix()[1];
            robotPoseEigen(0, 2) = robotPose.rotationAndTranslationMatrix()[2];
            robotPoseEigen(1, 0) = robotPose.rotationAndTranslationMatrix()[3];
            robotPoseEigen(1, 1) = robotPose.rotationAndTranslationMatrix()[4];
            robotPoseEigen(1, 2) = robotPose.rotationAndTranslationMatrix()[5];
            robotPoseEigen(2, 0) = robotPose.rotationAndTranslationMatrix()[6];
            robotPoseEigen(2, 1) = robotPose.rotationAndTranslationMatrix()[7];
            robotPoseEigen(2, 2) = robotPose.rotationAndTranslationMatrix()[8];
            robotPoseEigen(0, 3) = robotPose.rotationAndTranslationMatrix()[9];
            robotPoseEigen(1, 3) = robotPose.rotationAndTranslationMatrix()[10];
            robotPoseEigen(2, 3) = robotPose.rotationAndTranslationMatrix()[11];
            Eigen::Matrix4f cameraPose = robotPoseEigen*eigenHandEyeTransMatrix;

            // Capture point cloud
            pcl::PointCloud<pcl::PointXYZRGBA> cameraPointCloud =
                crf::sensors::rgbdcamera::RGBDUtils::getPointCloud(
                    camera->getColorFrame(),
                    camera->getDepthFrame(),
                    scale,
                    extrinsicsParam,
                    depthIntrinsicsParam,
                    colorIntrinsicsParam);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cameraPointCloudPtr(
                new pcl::PointCloud<pcl::PointXYZRGBA>(cameraPointCloud));
            // Update the map
            boost::optional<Eigen::Matrix4f> finalCameraPose = mapper.updateMap(
                cameraPointCloudPtr,
                cameraPose);
            if (!finalCameraPose) {
                logger->warn("Unable to update the map");
                continue;
            }
            logger->info("Map updated");
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outPointCloud(
                new pcl::PointCloud<pcl::PointXYZRGBA>());
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr map = mapper.getPointCloudMap().get();
            pcl::copyPointCloud(*map, *outPointCloud);
            mutex_.lock();
            logger->info("Map size: {} points", map->size());
            cloud_ = outPointCloud;
            affinePose_.matrix() = finalCameraPose.get();
            mutex_.unlock();
        } else if (status == std::future_status::ready) {
            logger->info("Ready");
        }
    } while (status != std::future_status::ready);

    char key;
    bool endflag = false;
    logger->info("Keyboard Control");
    while (!endflag) {
        logger->info("Press 'q' + ENTER to exit");
        std::cin >> key;
        if (key == 'q') endflag = true;
    }
    stopVisualizer_ = true;
    visualizer.join();

    return 0;
}
