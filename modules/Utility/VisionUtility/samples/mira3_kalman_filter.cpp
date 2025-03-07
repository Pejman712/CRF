 /*© Copyright CERN 2024.  All rights reserved.
 * This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pejman Habibiroudkenar CERN EN/SMM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <string>
#include <memory>
#include <vector>
#include <random>
#include <chrono>
#include <thread>
#include <signal.h>
#include <csignal>
#include <fstream>
#include <stdio.h>
#include <iostream>

//#include <gmock/gmock.h>

#include <Eigen/Dense>

// Kalman Filter includes
#include "KalmanFilter/SPSmodel/SPSSystemModel.hpp"
#include "KalmanFilter/SPSmodel/SPSObservationModel.hpp"
#include "KalmanFilter/KalmanFilter.hpp"
#include "EventLogger/EventLogger.hpp"

//Wheel Encoders
#include "SPSRobot/SPSRobot.hpp"
#include "RobotBase/EtherCATRobotBase.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include <boost/program_options.hpp>
//#include <initializer_list>


//PCL includes
#include <nlohmann/json.hpp>

#include "Laser/UnitreeL1/UnitreeL1.hpp"  
#include "IMU/UnitreeL1IMU/UnitreeL1IMU.hpp"


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>


#include <Eigen/Geometry>  


#include "VisionUtility/PointCloud/Gicp.hpp"  


//Initialize the Robot


using crf::actuators::robotbase::SPSRobot;
using crf::math::kalmanfilter::KalmanFilter;
using crf::math::kalmanfilter::StateSpace;
using crf::math::kalmanfilter::SPSSystemModel;
using crf::math::kalmanfilter::SPSObservationModel;

namespace po = boost::program_options;
bool ctrl_c_pressed = false;
void ctrlc(int) {
    ctrl_c_pressed = true;
}

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("ethercat_port", po::value<std::string>(), "EtherCAT port name (e.g., enp2s0)")
        ("sps_config", po::value<std::string>(), "SPSRobot configuration file path")
        ("device", po::value<std::string>(), "Serial port name (e.g., /dev/ttyUSB0)")
        ("unitree_config", po::value<std::string>(), "UnitreeL1 configuration file");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (const std::exception& e) {
        std::cout << "Error parsing command line arguments: " << e.what() << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }

    // Check for required arguments and handle them appropriately
    if (!vm.count("ethercat_port")) {
        std::cout << "Missing ethercat port argument" << std::endl << desc << std::endl;
        return -1;
    }
    if (!vm.count("sps_config")) {
        std::cout << "Missing SPSRobot configuration file argument" << std::endl << desc << std::endl;
        return -1;
    }
    if (!vm.count("device")) {
        std::cout << "Missing device argument" << std::endl << desc << std::endl;
        return -1;
    }
    if (!vm.count("unitree_config")) {
        std::cout << "Missing UnitreeL1 configuration file argument" << std::endl << desc << std::endl;
        return -1;
    }

    std::string ethercat_port = vm["ethercat_port"].as<std::string>();
    std::string sps_config_path = vm["sps_config"].as<std::string>();
    std::string device = vm["device"].as<std::string>();
    std::string unitree_config_path = vm["unitree_config"].as<std::string>();

    // Load SPSRobot configuration file
    std::ifstream sps_config_file(sps_config_path);
    if (!sps_config_file.is_open()) {
        std::cout << "Unable to open SPSRobot configuration file: " << sps_config_path << std::endl;
        return -1;
    }
    nlohmann::json sps_config;
    sps_config_file >> sps_config;

    // Initialize SPSRobot
    std::shared_ptr<SPSRobot> bot = std::make_shared<SPSRobot>(ethercat_port.c_str(), sps_config);
    if (!bot->initialize()) {
        std::cout << "Could not initialize the SPSRobot" << std::endl;
        return -1;
    }
    std::cout << "SPSRobot initialized successfully" << std::endl;

    // Load UnitreeL1 configuration file
    std::ifstream unitree_config_file(unitree_config_path);
    if (!unitree_config_file.is_open()) {
        std::cout << "Unable to open UnitreeL1 configuration file: " << unitree_config_path << std::endl;
        return -1;
    }
    nlohmann::json unitree_config;
    unitree_config_file >> unitree_config;

    // Initialize UnitreeL1 Lidar
    std::unique_ptr<crf::sensors::laser::UnitreeL1> laser = std::make_unique<crf::sensors::laser::UnitreeL1>(
        crf::sensors::laser::LaserConnectionType::Serial, device, 2000000, unitree_config);

    if (!laser->initialize()) {
        std::cout << "Could not initialize the UnitreeL1 Lidar" << std::endl;
        return -1;
    }
    std::cout << "UnitreeL1 Lidar initialized successfully" << std::endl;

    // Initialize UnitreeL1 IMU
    std::unique_ptr<crf::sensors::imu::UnitreeL1IMU> imu = std::make_unique<crf::sensors::imu::UnitreeL1IMU>(
        crf::sensors::imu::LaserConnectionType::Serial, device, 2000000);

    if (!imu->initialize()) {
        std::cout << "Could not initialize the UnitreeL1 IMU" << std::endl;
        return -1;
    }
    std::cout << "UnitreeL1 IMU initialized successfully" << std::endl;

    // Set up signal handler for Ctrl+C
    signal(SIGINT, ctrlc);

    int comparisonCount = 0;
    int nonConvergenceCount = 0;
    float x_l = 1.0; 
    float y_l = 1.0;
    float w_l = 1.0;

    float yaw = 0.0;

    Eigen::Matrix4f cumulativeTransformation = Eigen::Matrix4f::Identity();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> transformedClouds;

    // Create a pointer to store the previous scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr previousCloud(new pcl::PointCloud<pcl::PointXYZ>());

    Eigen::MatrixXd system_noise = Eigen::MatrixXd::Identity(3, 3) * 0.05;
    Eigen::MatrixXd observation_noise = Eigen::MatrixXd::Identity(3, 3) * 0.02;

    // Initial state estimate
    Eigen::VectorXd initial_mean = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Zero(3, 3);

    // Create control input and measurement vectors
    Eigen::VectorXd control_input = Eigen::VectorXd::Zero(6);  // 6-dimensional control input
    Eigen::VectorXd measurement_input = Eigen::VectorXd::Zero(3);    // 3-dimensional measurement

    // Initialize the System Model
    std::shared_ptr<SPSSystemModel> system_model = std::make_shared<SPSSystemModel>(system_noise);

    // Initialize the Observation Model
    std::shared_ptr<SPSObservationModel> observation_model = std::make_shared<SPSObservationModel>(observation_noise);
    //observation_model->C() = C;
    //observation_model->measurement() = measurement_input; // Set initial measurement

    // Initialize the State Space
    std::shared_ptr<StateSpace> state_space = std::make_shared<StateSpace>(initial_mean, initial_covariance);

    // Create an instance of the Kalman Filter
    KalmanFilter kf;
    float previous_gyro_z =0.0;
    float previous_x_l = 0.0; 
    float previous_y_l = 1.0;
    float previous_w_l = 0.0;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0, "axis", 0);

    while (!ctrl_c_pressed) {
        // Main loop to fetch point cloud data and visualize
        auto start = std::chrono::system_clock::now();
        
        // Read cloud
        //pcl::PointCloud<Point3D>::Ptr cloud = laser->getPointCloud();

        pcl::PointCloud<Point3D>::Ptr cloud = laser->getTargetPointCloud();
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Convert your Point3D points to pcl::PointXYZ points
        for (const auto& point : cloud->points) {
            pcl::PointXYZ xyz_point;
            xyz_point.x = point.x;
            xyz_point.y = point.y;
            xyz_point.z = point.z;
            xyz_cloud->points.push_back(xyz_point);
        }
        // Ensure the point cloud is properly organized before the GICP process
        xyz_cloud->width = xyz_cloud->points.size();
        xyz_cloud->height = 1;  // Unorganized point cloud
        xyz_cloud->is_dense = true;  // Optional

        // Ensure that the previous cloud is also organized correctly
        previousCloud->width = previousCloud->points.size();
        previousCloud->height = 1;  // Unorganized point cloud
        previousCloud->is_dense = true;  // Optional        

        // Skip the comparison if this is the first scan (i.e., no previous cloud available)
        if (previousCloud->empty()) {
            *previousCloud = *xyz_cloud;  // Store the first scan as the previous cloud
            continue;  // Skip to the next iteration
        }

        // Filter the clouds based on the height
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_source(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_target(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0, 5.0);

        pass.setInputCloud(previousCloud);
        pass.filter(*cloud_filtered_source);

        pass.setInputCloud(xyz_cloud);
        pass.filter(*cloud_filtered_target);

        Eigen::Matrix4f transformationMatrix;
        auto startTime = std::chrono::high_resolution_clock::now();

        // Use 'template' keyword to disambiguate the function template
        bool gicpConverged = crf::utility::visionutility::pointcloud::gicp::template gicp<pcl::PointXYZ>(
            cloud_filtered_source, cloud_filtered_target, transformationMatrix);

        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> processingTime = endTime - startTime;

        if (!gicpConverged) {
            std::cerr << "GICP did not converge for the current scan comparison" << std::endl;
            nonConvergenceCount++;
        } else {
            // Process the transformation as before
            Eigen::Matrix3f rotationMatrix = transformationMatrix.block<3,3>(0,0);
            Eigen::Vector3f translationVector = transformationMatrix.block<3,1>(0,3);
            yaw = atan2(rotationMatrix(1,0), rotationMatrix(0,0));

            Eigen::Matrix3f newRotationMatrix;
            newRotationMatrix << cos(yaw), -sin(yaw), 0.0f,
                                sin(yaw),  cos(yaw), 0.0f,
                                    0.0f,      0.0f, 1.0f;

            translationVector[2] = 0.0f;
            Eigen::Matrix4f modifiedTransformationMatrix = Eigen::Matrix4f::Identity();
            modifiedTransformationMatrix.block<3,3>(0,0) = newRotationMatrix;
            modifiedTransformationMatrix.block<3,1>(0,3) = translationVector;
            transformationMatrix = modifiedTransformationMatrix;

            // Accumulate the transformation matrix
            cumulativeTransformation = cumulativeTransformation * transformationMatrix.inverse();
        }

        // Update the previous cloud for the next iteration
        *previousCloud = *xyz_cloud;  // Save the current scan as the previous scan

        // Ensure the point cloud is properly populated before displaying it
        std::cout << "Number of points in the cloud: " << xyz_cloud->points.size() << std::endl;

        comparisonCount++;
        // Transform the target cloud using the cumulative transformation matrix
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedTargetCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*xyz_cloud, *transformedTargetCloud, cumulativeTransformation);

        // Add the transformed cloud to the viewer with a unique identifier
        std::string cloudName = "transformedCloud_" + std::to_string(comparisonCount);

        // IMU DATA
        crf::sensors::imu::IMUSignals imuSignals = imu->getSignal();

        // Print the IMU signals
        std::cout << "IMU Signals:" << std::endl;

        if (imuSignals.quaternion) {
            std::cout << "\tQuaternion: [" 
                    << imuSignals.quaternion.value()[0] << ", " 
                    << imuSignals.quaternion.value()[1] << ", " 
                    << imuSignals.quaternion.value()[2] << ", " 
                    << imuSignals.quaternion.value()[3] << "]" << std::endl;
        } else {
            std::cout << "\tQuaternion data not available" << std::endl;
        }

        if (imuSignals.angularVelocity) {
            std::cout << "\tAngular Velocity: [" 
                    << imuSignals.angularVelocity.value()[0] << ", " 
                    << imuSignals.angularVelocity.value()[1] << ", " 
                    << imuSignals.angularVelocity.value()[2] << "]" << std::endl;
        } else {
            std::cout << "\tAngular Velocity data not available" << std::endl;
        }

        if (imuSignals.linearAcceleration) {
            std::cout << "\tLinear Acceleration: [" 
                    << imuSignals.linearAcceleration.value()[0] << ", " 
                    << imuSignals.linearAcceleration.value()[1] << ", " 
                    << imuSignals.linearAcceleration.value()[2] << "]" << std::endl;
        } else {
            std::cout << "\tLinear Acceleration data not available" << std::endl;
        }

        // Extract control parameters from IMU data if available
        float a_x = 0.0f, a_y = 0.0f, gyro_z = 0.0f;
        if (imuSignals.linearAcceleration) {
            a_x = imuSignals.linearAcceleration.value()[0];
            a_y = imuSignals.linearAcceleration.value()[1];
        }
        if (imuSignals.angularVelocity) {
            gyro_z = imuSignals.angularVelocity.value()[2];
        }
        float delta_gyro_z = gyro_z - previous_gyro_z;
        previous_gyro_z = gyro_z; // Update previous_gyro_z after calculating delta

        // Add cout statements for control parameters from IMU
        std::cout << "Control Parameters from IMU:" << std::endl;
        std::cout << "\ta_x = " << a_x << std::endl;
        std::cout << "\ta_y = " << a_y << std::endl;
        std::cout << "\tdelta_gyro_z = " << delta_gyro_z << std::endl;

        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        boost::optional<crf::utility::types::TaskVelocity> pose = bot->getTaskVelocity();
        std::cout << *pose << std::endl;
        if (pose) {
            if (pose->size() > 5) {  // Ensure there are at least 6 elements
                x_l = (*pose)[0]; 
                y_l = (*pose)[1];
                w_l = (*pose)[5];
                // Use x_l, y_l, w_l as needed

                // Add cout statements for robot velocities
                std::cout << "Robot Velocities:" << std::endl;
                std::cout << "\tx_l = " << x_l << std::endl;
                std::cout << "\ty_l = " << y_l << std::endl;
                std::cout << "\tw_l = " << w_l << std::endl;

            } else {
                std::cerr << "TaskVelocity does not contain enough elements." << std::endl;
                // Handle the error or set default values
                continue;  // Skip this iteration if necessary
            }
        } else {
            std::cerr << "Could not get the velocities from SPSRobot." << std::endl;
            x_l = previous_x_l;
            y_l = previous_y_l;
            w_l = previous_w_l;
            // Handle the error or set default values
            continue;  // Skip this iteration if necessary
        }
        std::cout << "1  = " << std::endl;
        // Use the cumulative transformation matrix to extract x_translation and y_translation
        float x_translation = cumulativeTransformation(0, 3);
        float y_translation = cumulativeTransformation(1, 3);
        float psi = yaw;

        std::cout << "x_translation = " << x_translation << std::endl;
        std::cout << "y_translation  = " << y_translation << std::endl;
        std::cout << "yaw  = " << psi << std::endl;

        Eigen::VectorXd control_input(6);
        control_input << x_l, y_l, w_l, a_x, a_y, delta_gyro_z;
        //system_model->control() = control_input;

        system_model->setControl(control_input);

        Eigen::VectorXd measurement_input(3);
        measurement_input << x_translation, y_translation, psi;
        //observation_model->measurement(measurement_input);
        observation_model->setMeasurement(measurement_input);

        kf.prediction(system_model, state_space);
        kf.correction(observation_model, state_space);

        Eigen::VectorXd meanf = state_space->getMean();
        Eigen::VectorXd covf = state_space->getMean();

        std::cout << "State Mean:\n" << meanf << "\n";
        std::cout << "State Covariance:\n" << covf << "\n\n";

        Eigen::Matrix3f rotationMatrix = transformationMatrix.block<3,3>(0,0);
        Eigen::Vector3f translationVector = transformationMatrix.block<3,1>(0,3);
        float yawf = meanf[2];

        Eigen::Matrix3f newRotationMatrix;
        newRotationMatrix << cos(yawf), -sin(yawf), 0.0f,
                            sin(yawf),  cos(yawf), 0.0f,
                                0.0f,      0.0f, 1.0f;

        translationVector[0] = meanf[0];
        translationVector[1] = meanf[1];
        translationVector[2] = 0.0f;
        Eigen::Matrix4f modifiedTransformationMatrix = Eigen::Matrix4f::Identity();
        modifiedTransformationMatrix.block<3,3>(0,0) = newRotationMatrix;
        modifiedTransformationMatrix.block<3,1>(0,3) = translationVector;
        transformationMatrix = modifiedTransformationMatrix;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*xyz_cloud, *map, transformationMatrix);
        if (!viewer->updatePointCloud(map, "map_cloud")) {
            viewer->addPointCloud(map, "map_cloud");
        }
        viewer->spinOnce(100); // Allow viewer to update

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust as needed
    }

    return 0;
}
