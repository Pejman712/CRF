/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */
#include "RGBDVisionUtility/PCLDataTypes.hpp"
#include "ZMQ/Request.hpp"
#include "ObjectDetection/ObjectDetector.hpp"
#include "ObjectPoseEstimator/ObjectPoseEstimator.hpp"
#include "ObjectPoseEstimator/PoseEstimationData.hpp"
#include "Mapper3d/HandHeldMapper3d.hpp"
#include "Mapper3d/Mapper3d.hpp"
#include "Mapper3d/NormalsColorOctree.hpp"
#include "RGBDCamera/RealSense2Grabber.hpp"
#include "RGBDCamera/IRGBDCamera.hpp"
#include "RGBDCamera/RGBDUtils.hpp"
#include "ClosedLoopController/PIDController.hpp"
#include "RobotArm/RobotArmDefaultKinematics.hpp"
#include "CANSocket/CANSocket.hpp"
#include "SchunkArm/SchunkArm.hpp"
#include "RobotArmControllersDeprecated/RobotArmDefaultController.hpp"
#include "Types/Types.hpp"

#include <initializer_list>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/geometry.h>
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

bool stopViz_ = false;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr environmnetPointCloud_;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr modelTransformedPointCloud_;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr blmCloud_;
Eigen::Affine3f clusterffinePose_;
Eigen::Affine3f blmAffinePose_;
crf::applications::objectposeestimator::Object3DBoundingBox blm3dBB_;
std::mutex blmMutex_;


void Visualization() {
        pcl::visualization::PCLVisualizer blmViewer("BLM");
        blmViewer.setBackgroundColor(0.0, 0.0, 0.0);
        pcl::visualization::PCLVisualizer modelViewer("MODEL");
        modelViewer.setBackgroundColor(0.0, 0.0, 0.0);
        pcl::visualization::PCLVisualizer environmentViewer("ENVIRONMENT");
        environmentViewer.setBackgroundColor(0.0, 0.0, 0.0);

        Eigen::Matrix4f identity;
        identity.setIdentity();
        Eigen::Affine3f base;
        base.matrix() = identity;
        blmViewer.addCoordinateSystem(0.1, base , "Base");
        modelViewer.addCoordinateSystem(0.1, base , "Base");
        environmentViewer.addCoordinateSystem(0.1, base , "Base");

    while (!stopViz_) {
        blmMutex_.lock();

        if (blmCloud_->size() > 0) {
            blmViewer.removeAllPointClouds();
            blmViewer.removeCoordinateSystem("BLM");
            blmViewer.addPointCloud(blmCloud_, "BLM");
            blmViewer.addCoordinateSystem(0.3, blmAffinePose_, "BLM");
            blmViewer.spinOnce();
        }

        if (modelTransformedPointCloud_->size() > 0) {
            modelViewer.removeAllPointClouds();
            modelViewer.removeCoordinateSystem("MODEL");
            modelViewer.addPointCloud(modelTransformedPointCloud_, "MODEL");
            modelViewer.addCoordinateSystem(0.3, blmAffinePose_, "MODEL");
            modelViewer.spinOnce();
        }

        if (environmnetPointCloud_->size() > 0) {
            environmentViewer.removeAllPointClouds();
            environmentViewer.removeCoordinateSystem("ENVIRONMENT");
            environmentViewer.addPointCloud(environmnetPointCloud_, "ENVIRONMENT");
            environmentViewer.addCoordinateSystem(0.3, blmAffinePose_, "ENVIRONMENT");
            environmentViewer.spinOnce();

            if (blmCloud_->size() > 0) {
                environmentViewer.removeShape("OBB");
                Eigen::Vector3f position(blm3dBB_.positionOBB.x, blm3dBB_.positionOBB.y,
                    blm3dBB_.positionOBB.z);
                Eigen::Quaternionf quat(blm3dBB_.rotationalMatrixOBB);
                environmentViewer.addCube(position, quat,
                    blm3dBB_.maxPointOBB.x - blm3dBB_.minPointOBB.x,
                    blm3dBB_.maxPointOBB.y - blm3dBB_.minPointOBB.y,
                    blm3dBB_.maxPointOBB.z - blm3dBB_.minPointOBB.z, "OBB");
                environmentViewer.setShapeRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
            }
        }

        blmMutex_.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char **argv) {
    if (argc < 10) {
        std::cout << "Few arguments provide:" <<std::endl;
        std::cout << "  [1] Path to hand-eye transformation matrix" << std::endl;
        std::cout << "  [2] Path to trajectory Json file" << std::endl;
        std::cout << "  [3] Can Socket interface name" << std::endl;
        std::cout << "  [4] Path to Schunk Arm configuration file" << std::endl;
        std::cout << "  [5] Path to Blm 3dMapper configuration Json file" << std::endl;
        std::cout << "  [6] Path to RGBD Camera config file" << std::endl;
        std::cout << "  [7] ZMQ Requester socket address" << std::endl;
        std::cout << "  [8] Path to Environment 3dMapper configuration Json file" << std::endl;
        std::cout << "  [9] Path to pose estimator configuration file" << std::endl;
        return -1;
    }

    // Create 2d object detector
    std::string socketAddress(argv[7]);
    auto requester = std::make_shared<crf::communication::zmqcomm::Request>(socketAddress);
    crf::vision::objectDetection::ObjectDetector blm2dDetector(requester,
        std::chrono::milliseconds(1500), 1);
    blm2dDetector.initialize();

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

    // Create objects to visualize the environment and blm pointcloud
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr environmentAuxCloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr environmentCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>());
    environmnetPointCloud_ = environmentCloud;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr blmAuxCloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr blmCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    blmCloud_ = blmCloud;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr modelAuxCloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr modelCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr modelTransformedCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>());
    modelTransformedPointCloud_ = modelTransformedCloud;


    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr clusterPointcloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr clusterTransformedPointcloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    std::thread viz(&Visualization);

    // Create one mapper for the environment and other for the blm
    crf::applications::mapper3d::Mapper3d environmentMerger(argv[8]);
    crf::applications::mapper3d::Mapper3d blmMerger(argv[5]);
    Eigen::Matrix4f cameraPose;
    Eigen::Matrix4f robot_pose;
    boost::optional<Eigen::Matrix4f> boostBlmFinalCameraPose;
    boost::optional<Eigen::Matrix4f> boostFinalCameraPose;
    Eigen::Matrix4f finalCameraPose;
    Eigen::Matrix4f targetFinalPose;
    crf::applications::objectposeestimator::Object3DBoundingBox blm3dBB;

    // BLM Pose Estimator declaration
    crf::applications::objectposeestimator::ObjectPoseEstimator poseEstimator(argv[9]);

    // Execute robot trajectory
    std::future<bool> future = controller.executeJointsTrajectory(path);
    std::future_status status;
    std::cout << "EXECUTING TRAJECTORY" <<std::endl;

    do {
        status = future.wait_for(std::chrono::milliseconds(0));

        if (status == std::future_status::deferred) {
            std::cout << "deferred\n";

        } else if (status == std::future_status::timeout) {
            std::cout << "timeout\n";
            // Capture pose and add hand eye transform
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

            // Capture pointcloud and color frame
            cv::Mat color = camera_->getColorFrame();
            cv::Mat depth = camera_->getDepthFrame();
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud(
                new pcl::PointCloud<pcl::PointXYZRGBA>());
            *inPointCloud = crf::sensors::rgbdcamera::RGBDUtils::getPointCloud(color, depth,
             scale_, extrinsics_, depthIntrinsics_, colorIntrinsics_);

            // Update environment map and extract final camera position from there
            boostFinalCameraPose = environmentMerger.updateMap(inPointCloud, cameraPose);
            if (!boostFinalCameraPose) {
                std::cout << "Unable to update the environment map" <<std::endl;
            } else {
                environmentAuxCloud = environmentMerger.getPointCloudMap().get();

                finalCameraPose = boostFinalCameraPose.get();
                std::cout << "Environment map updated" <<std::endl;

                // Get Detected PointCloud from 2d bounding box
                boost::optional<std::vector<crf::vision::types::DetectedPointCloud>>
                    boostDetectedPointClouds;
                std::vector<crf::vision::types::DetectedPointCloud> detectedPointClouds;
                boostDetectedPointClouds = blm2dDetector.getDetectedPointClouds(color,*inPointCloud); //NOLINT
                // If the inference returns any detected pointcloud
                if (boostDetectedPointClouds) {
                    detectedPointClouds = boostDetectedPointClouds.get();
                    // Check if any of the detectedPointClouds is from a BLM
                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr blmPointcloud;
                    for (int i = 0; i < static_cast<int>(detectedPointClouds.size()); i++) {
                        if (detectedPointClouds[i].classif == "blm") {
                            std::cout << "Pointcloud from " << detectedPointClouds[i].classif <<" detected" << std::endl; //NOLINT
                            *blmPointcloud = detectedPointClouds[i].pointCloud;

                            // Update BLM map
                            boostBlmFinalCameraPose = blmMerger.updateMap(blmPointcloud, finalCameraPose); //NOLINT
                            if (!boostBlmFinalCameraPose) {
                                std::cout << "Unable to update the blm map" <<std::endl;
                            } else {
                                std::cout << "Blm map updated" <<std::endl;
                                blmAuxCloud = blmMerger.getPointCloudMap().get();
                                boost::optional<crf::applications::objectposeestimator::PoseEstimationData> boostTargetPose = poseEstimator.computeTargetPose(blmAuxCloud); //NOLINT
                                crf::applications::objectposeestimator::PoseEstimationData targetPose; //NOLINT
                                if (!boostTargetPose) {
                                    std::cout << "Unable to compute target pose" <<std::endl;
                                } else {
                                    // Pose of the object model in the scanned world
                                    targetPose.pose = poseEstimator.getBestPose();
                                    targetFinalPose <<  targetPose.pose[0], targetPose.pose[1], targetPose.pose[2], targetPose.pose[9], //NOLINT
                                                        targetPose.pose[3], targetPose.pose[4], targetPose.pose[5], targetPose.pose[10], //NOLINT
                                                        targetPose.pose[6], targetPose.pose[7], targetPose.pose[8], targetPose.pose[11], //NOLINT
                                                        0, 0, 0, 1;

                                     // Get object model and transform it
                                    boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> boostModelAuxCloud = poseEstimator.getObjectModel(); //NOLINT
                                    if (boostModelAuxCloud)
                                        *modelAuxCloud = *boostModelAuxCloud.get();
                                    pcl::copyPointCloud(*modelAuxCloud, *modelCloud);
                                    pcl::transformPointCloud(*modelCloud, *modelTransformedCloud, targetFinalPose); //NOLINT
                                    blm3dBB = poseEstimator.getBestObject3DBoundingBox();
                                }
                                pcl::copyPointCloud(*blmAuxCloud, *blmCloud);
                                std::cout << "blmOutPointCloud size: "
                                    << blmCloud->size() << " points" << std::endl;
                            }
                        }
                    }

                } else {
                    std::cout << "ANY DETECTED POINTCLOUD FOUND" << std::endl;
                }
            }

            pcl::copyPointCloud(*environmentAuxCloud, *environmentCloud);
            blmMutex_.lock();
            *blmCloud_ = *blmCloud;
            *environmnetPointCloud_ = *environmentCloud;
            blmAffinePose_.matrix() = targetFinalPose;
            modelTransformedPointCloud_ = modelTransformedCloud;
            blm3dBB_ = blm3dBB;

            blmMutex_.unlock();

        } else if (status == std::future_status::ready) {
            std::cout << "ready!\n";
        }
    } while (status != std::future_status::ready);


    // If the letter 'q' is pressed the program process this frame pointcloud
    char key;
    bool endflag = false;
    std::cout << "KEYBOARD CONTROL"<< std::endl;
    while (!endflag) {
        std::cout << "Press 'q'+ ENTER to exit" << std::endl;
        std::cin >> key;
        if (key == 'q') endflag = true;
    }

    // Extract cluster
    boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> boostClusterPointcloud = poseEstimator.getCurrentObjectCluster(); //NOLINT
    if (boostClusterPointcloud)
        *clusterPointcloud = *boostClusterPointcloud.get();

    Eigen::Matrix4f clusterPose = targetFinalPose.inverse();
    pcl::transformPointCloudWithNormals(*clusterPointcloud, *clusterTransformedPointcloud, clusterPose); //NOLINT

    pcl::io::savePCDFileBinary("BlmModel.pcd", *clusterTransformedPointcloud);


    pcl::io::savePCDFileBinary("Environment.pcd", *environmnetPointCloud_);


    blm2dDetector.deinitialize();

    stopViz_ = true;
    viz.join();



    return 0;
}
