/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */


#include "Mapper3d/Mapper3d.hpp"
#include "RGBDCamera/RealSense2Grabber.hpp"
#include "RGBDCamera/IRGBDCamera.hpp"
#include "RGBDCamera/RGBDUtils.hpp"
#include "CameraViewer/CameraViewer.hpp"
#include "ClosedLoopController/PIDController.hpp"
#include "RobotArm/RobotArmDefaultKinematics.hpp"
#include "CANSocket/CANSocket.hpp"
#include "SchunkArm/SchunkArm.hpp"
#include "RobotArmControllersDeprecated/RobotArmDefaultController.hpp"

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <fstream>
#include <thread>
#include <mutex>
#include <nlohmann/json.hpp>
#include <visp3/vision/vpPose.h>
#include <boost/optional.hpp>
#include <memory>
#include <string>
#include <vector>
#include <future>

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
Eigen::Affine3f affinePose_;
bool stopViz_ = false;
std::mutex mutex_;

void visualization() {
    crf::applications::cameraviewer::CameraViewer viewer("MapWhileMoving");

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
    if (argc < 7) {
        std::cout << "Few arguments provide:" <<std::endl;
        std::cout << "  [1] Path to hand-eye transformation matrix" << std::endl;
        std::cout << "  [2] Path to trajectory Json file" << std::endl;
        std::cout << "  [3] Can Socket interface name" << std::endl;
        std::cout << "  [4] Path to Schunk Arm configuration file" << std::endl;
        std::cout << "  [5] Path to 3dMapper configuration Json file" << std::endl;
        std::cout << "  [6] Path to RGBD Camera config file" << std::endl;
        return -1;
    }

    // Open camera
    std::string configFileCamera(argv[6]);
    std::ifstream configCamera(configFileCamera);
    nlohmann::json jConfigCamera;
    configCamera >> jConfigCamera;
    auto camera =
    std::make_shared<crf::sensors::rgbdcamera::RealSense2Grabber>(jConfigCamera.at("RealSense"));
    std::shared_ptr<crf::sensors::rgbdcamera::IRGBDCamera> camera_(camera);
    if (!camera_->initialize()) {
        std::cout << "There is a problem to initialize the RGBD Camera" << std::endl;
        return false;
    }
    rs2_extrinsics extrinsics_ = camera_->getDepth2ColorExtrinsics();
    rs2_intrinsics depthIntrinsics_ = camera_->getDepthIntrinsics();
    rs2_intrinsics colorIntrinsics_ = camera_->getColorIntrinsics();
    float scale_ = camera_->getDepthScale2Meters();

    // Read hand-eye transform from .txt
    vpHomogeneousMatrix M;
    Eigen::Matrix4f handeyeTransform;
    std::ifstream f(argv[1]);
    M.load(f);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            handeyeTransform(i, j) = M[i][j];
        }
    }
    std::cout << "Hand-eye transformation matrix: " << std::endl << handeyeTransform << std::endl;

    // Read trajectory positions from .json
    std::vector<crf::utility::types::JointPositions> path;
    std::ifstream config(argv[2]);
    nlohmann::json jConfig;
    config >> jConfig;

    int positionsSize = jConfig.at("positionsSize").get<int>();
    std::cout << "positionsSize: " << positionsSize << std::endl;

    for (int i = 0; i < positionsSize; i++) {
        crf::utility::types::JointPositions pos(6);
        for (int j = 0; j < 6; j++) {
            std::string position = "position" + std::to_string(i);
            std::string joint = "joint" + std::to_string(j);
            float jointPositions = jConfig.at("positions").at(position).at(joint).get<float>();
            pos(j) = jointPositions;
        }
        path.push_back(pos);
    }
    std::cout << "Trajectory stored" << std::endl;

    // Create Can socket
    auto can_socket = std::make_shared<CANSocket>(argv[3]);
    // Create Schunk Arm
    std::ifstream robotData(argv[4]);
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    auto arm = std::make_shared<crf::robots::schunkarm::SchunkArm>(
        can_socket, robotJSON);

    if (!arm->initialize()) {
        std::cout << "Could not initialize the arm" << std::endl;
        return -1;
    }

    // Create arm kinematics
    auto kinematics =
    std::make_shared<crf::robots::robotarm::RobotArmDefaultKinematics>(arm->getConfiguration());
    int jointsCount = arm->getConfiguration()->getNumberOfJoints();

    // Create closed loop controller
    auto closed_loop_controller =
        std::make_shared<crf::algorithms::closedloopcontroller::PIDController>(
        std::vector<float>(jointsCount, 0),
        std::vector<float>(jointsCount, 0),
        std::vector<float>(jointsCount, 0));

    crf::applications::robotarmcontroller::RobotArmDefaultController controller(arm,
        kinematics, closed_loop_controller);

    if (!controller.initialize()) {
        std::cout << "Could not initialize the controller" << std::endl;
        return -1;
    }
    std::cout << "Controller initialized" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud_ = cloud;
    std::thread viz(&visualization);
    crf::applications::mapper3d::Mapper3d merger(argv[5]);

    // Execute robot trajectory
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::future<bool> future = controller.executeJointsTrajectory(path);
    std::future_status status;
    std::cout << "EXECUTING TRAJECTORY" <<std::endl;

    do {
        status = future.wait_for(std::chrono::milliseconds(10));

        if (status == std::future_status::deferred) {
            std::cout << "deferred\n";

        } else if (status == std::future_status::timeout) {
            std::cout << "timeout\n";
            // Capture pose and add hand eye transform
            Eigen::Matrix4f cameraPose;
            Eigen::Matrix4f robot_pose;
            robot_pose.setIdentity();
            robot_pose(0, 0) = controller.getTaskPose().rotationAndTranslationMatrix()[0];
            robot_pose(0, 1) = controller.getTaskPose().rotationAndTranslationMatrix()[1];
            robot_pose(0, 2) = controller.getTaskPose().rotationAndTranslationMatrix()[2];
            robot_pose(1, 0) = controller.getTaskPose().rotationAndTranslationMatrix()[3];
            robot_pose(1, 1) = controller.getTaskPose().rotationAndTranslationMatrix()[4];
            robot_pose(1, 2) = controller.getTaskPose().rotationAndTranslationMatrix()[5];
            robot_pose(2, 0) = controller.getTaskPose().rotationAndTranslationMatrix()[6];
            robot_pose(2, 1) = controller.getTaskPose().rotationAndTranslationMatrix()[7];
            robot_pose(2, 2) = controller.getTaskPose().rotationAndTranslationMatrix()[8];
            robot_pose(0, 3) = controller.getTaskPose().rotationAndTranslationMatrix()[9];
            robot_pose(1, 3) = controller.getTaskPose().rotationAndTranslationMatrix()[10];
            robot_pose(2, 3) = controller.getTaskPose().rotationAndTranslationMatrix()[11];
            cameraPose = robot_pose*handeyeTransform;

            // Capture pointcloud
            cv::Mat color = camera_->getColorFrame();
            cv::Mat depth = camera_->getDepthFrame();
            pcl::PointCloud<pcl::PointXYZRGBA> inPointCloud =
                crf::sensors::rgbdcamera::RGBDUtils::getPointCloud(color, depth, scale_,
                extrinsics_, depthIntrinsics_, colorIntrinsics_);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloudPtr(
                new pcl::PointCloud<pcl::PointXYZRGBA>(inPointCloud));
            // Update map
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outPointCloud(
                new pcl::PointCloud<pcl::PointXYZRGBA>());
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outPointCloud2(
                new pcl::PointCloud<pcl::PointXYZRGBNormal>());
            Eigen::Matrix4f finalCameraPose;
            boost::optional<Eigen::Matrix4f> boostFinalCameraPose;

            // cameraPose.setIdentity();
            boostFinalCameraPose = merger.updateMap(inPointCloudPtr, cameraPose);
            if (boostFinalCameraPose == boost::none) {
                std::cout << "Unable to update the map" <<std::endl;
            } else {
                finalCameraPose = boostFinalCameraPose.get();
                std::cout << "Map updated" <<std::endl;
            }
            outPointCloud2 = merger.getPointCloudMap().get();
            pcl::copyPointCloud(*outPointCloud2, *outPointCloud);
            mutex_.lock();
            std::cout << "outPointCloud size: "
                << outPointCloud->size() << " points" << std::endl;
            cloud_ = outPointCloud;
            affinePose_.matrix() = finalCameraPose;
            mutex_.unlock();

        } else if (status == std::future_status::ready) {
            std::cout << "ready!\n";
        }
    } while (status != std::future_status::ready);

    // merger.savePointCloudToDisk("movinArmPointCloud", false);

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
