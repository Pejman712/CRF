 /*© Copyright CERN 2024.  All rights reserved.
 * This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pejman Habibiroudkenar & Dadi Hrannar Davidsson CERN EN/SMM/MRO 2024
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

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>  

// Kalman Filter includes
#include "KalmanFilter/SPSmodel/SPSSystemModel.hpp"
#include "KalmanFilter/SPSmodel/SPSObservationModel.hpp"
#include "KalmanFilter/KalmanFilter.hpp"
#include "EventLogger/EventLogger.hpp"

// Wheel Encoders
#include "SPSRobot/SPSRobot.hpp"
#include "RobotBase/EtherCATRobotBase.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include <boost/program_options.hpp>

// PCL includes
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

#include "VisionUtility/PointCloud/Gicp.hpp"  

#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <thread>  // For std::this_thread::sleep_for
#include <chrono>  // For std::chrono::milliseconds
#include <algorithm>  // For sorting sharpness values
#include "VisionUtility/PointCloud/Edge.hpp"
#include "VisionUtility/PointCloud/Communication.hpp"

// g2o includes
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
// For 3D transformations
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

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

pcl::PointCloud<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>::Ptr filterTopSharpness(
    pcl::PointCloud<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>::Ptr sharpness_cloud, float percent) {

    // Ensure percent is between 0 and 100
    if (percent <= 0.0f || percent > 100.0f) {
        throw std::invalid_argument("Percent must be between 0 and 100.");
    }

    // Get the sharpness values
    std::vector<float> sharpness_values;
    for (const auto& point : sharpness_cloud->points) {
        sharpness_values.push_back(point.s);
    }

    // Sort the sharpness values in descending order
    std::sort(sharpness_values.begin(), sharpness_values.end(), std::greater<float>());

    // Get the threshold for the top N% sharpness
    size_t top_percent_index = static_cast<size_t>(sharpness_values.size() * (1.0f - (percent / 100.0f)));
    float sharpness_threshold = sharpness_values[top_percent_index];

    // Filter points that have sharpness values greater than or equal to the threshold
    pcl::PointCloud<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>::Ptr filtered_cloud(
        new pcl::PointCloud<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>());

    for (const auto& point : sharpness_cloud->points) {
        if (point.s >= sharpness_threshold) {
            filtered_cloud->points.push_back(point);
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    return filtered_cloud;
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

    ///////////////////////////////// edge detection declration////////////////////////////

    Eigen::Matrix4f accumilatedTransformationMatrix = Eigen::Matrix4f::Identity();
    unsigned int kNeighbour = 20; 
    unsigned int percent = 90;  // Number of neighbors for edge detection
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumilated_scan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_accumilated_scan(new pcl::PointCloud<pcl::PointXYZ>);

    ////graph optimization////////////////////////////////////
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto linearSolver = g2o::make_unique<LinearSolverType>();
    auto solver_ptr = g2o::make_unique<BlockSolverType>(std::move(linearSolver));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

    optimizer.setAlgorithm(solver);

    int vertexId = 0;  // Unique ID for each vertex

    // Variables to store robot positions and landmark observations
    std::vector<Eigen::Matrix4f> robotPositions; // 4x4 transformation matrices
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scans;



    while (!ctrl_c_pressed) {
        // Main loop to fetch point cloud data and visualize
        auto start = std::chrono::system_clock::now();
        
        // Read cloud
        //pcl::PointCloud<Point3D>::Ptr cloud = laser->getPointCloud();

        std::cout << "Before laser" << std::endl; 
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
        std::vector<Eigen::Matrix4f> robotPositions;  
        transformationMatrix = modifiedTransformationMatrix;
        robotPositions = {modifiedTransformationMatrix};

        robotPositions.push_back(modifiedTransformationMatrix);
        // Store the scan
        scans.push_back(transformedTargetCloud->makeShared());


        // Assume 'scans' is a vector of point clouds and 'robotPositions' is a vector of Eigen::Matrix4f
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scans;  // Populate this with your scans
           // Populate this with your robot positions

        // Accumulate scans every 30 scans
        for (size_t i = 0; i < scans.size(); ++i) {
            if (i % 30 == 0 && i != 0) {
                // Swap current accumulated scan to previous
                *previous_accumilated_scan = *accumilated_scan;

                // Reset the current accumulated scan
                accumilated_scan->clear();
            }

            // Transform the scan based on robotPosition
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scan(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*scans[i], *transformed_scan, robotPositions[i]);

            // Accumulate the transformed scan
            *accumilated_scan += *transformed_scan;
        }

        // Compute edge sharpness
        auto sharpness_cloud = crf::utility::visionutility::pointcloud::edge::computeEigenSharpness<pcl::PointXYZ>(
            accumilated_scan, kNeighbour);

        if (!sharpness_cloud || sharpness_cloud->empty()) {
            std::cerr << "No sharpness data computed!" << std::endl;
            return -1;
        }

        std::cout << "Computed sharpness for " << sharpness_cloud->points.size() << " points." << std::endl;

        // Filter the sharpness to show only the top N%
        auto filtered_sharpness_cloud = filterTopSharpness(sharpness_cloud, percent);

                // For demonstration, we'll add the g2o vertices and edges every N iterations
        int N = 1000; // Define how often you want to run optimization
        if (robotPositions.size() >= N) {
            // Add robot pose vertices
            for (size_t i = 0; i < robotPositions.size(); ++i) {
                Eigen::Matrix4f poseMatrix = robotPositions[i];

                // Convert Eigen::Matrix4f to g2o::SE3Quat
                Eigen::Affine3f affine(poseMatrix);
                Eigen::Quaternionf q(affine.rotation());
                Eigen::Vector3f t = affine.translation();

                // Since the robot operates in 2D, set z, roll, and pitch to zero
                Eigen::Quaternionf q_constrained(Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX()) *
                                                 Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitY()) *
                                                 Eigen::AngleAxisf(affine.rotation().eulerAngles(0,1,2)[2], Eigen::Vector3f::UnitZ()));
                t.z() = 0.0f;

                g2o::SE3Quat pose(Eigen::Quaterniond(q_constrained.cast<double>()), t.cast<double>());

                // Create a new vertex for the robot pose
                g2o::VertexSE3* v_se3 = new g2o::VertexSE3();
                v_se3->setId(vertexId);
                v_se3->setEstimate(pose);

                // Fix the first pose to anchor the graph
                if (i == 0) {
                    v_se3->setFixed(true);
                }

                optimizer.addVertex(v_se3);
                ++vertexId;
            }

            // Add landmark vertices
            std::unordered_map<int, int> landmarkIdMap;
            for (size_t i = 0; i < filtered_sharpness_cloud->points.size(); ++i) {
                const auto& point = filtered_sharpness_cloud->points[i];

                // Create a new vertex for the landmark
                g2o::VertexPointXYZ* v_p = new g2o::VertexPointXYZ();
                v_p->setId(vertexId);
                v_p->setEstimate(Eigen::Vector3d(point.x, point.y, point.z));
                v_p->setMarginalized(true);

                optimizer.addVertex(v_p);
                landmarkIdMap[i] = vertexId;
                ++vertexId;
            }

            // Add edges between poses and landmarks
            Eigen::Matrix3d information = Eigen::Matrix3d::Identity(); // Adjust based on sensor noise
            for (size_t i = 0; i < robotPositions.size(); ++i) {
                g2o::VertexSE3* v_se3 = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(i));

                // For each landmark observed from this pose
                for (const auto& landmarkEntry : landmarkIdMap) {
                    int landmarkIndex = landmarkEntry.first;
                    int landmarkVertexId = landmarkEntry.second;

                    g2o::VertexPointXYZ* v_p = dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(landmarkVertexId));

                    // Compute the measurement: the landmark position in the robot's local frame
                    Eigen::Isometry3d pose = v_se3->estimate();
                    Eigen::Vector3d landmarkPosition = v_p->estimate();
                    Eigen::Vector3d measurement = pose.inverse() * landmarkPosition;

                    // Create an edge between the pose and the landmark
                    g2o::EdgeSE3PointXYZ* edge = new g2o::EdgeSE3PointXYZ();
                    edge->setVertex(0, v_se3);
                    edge->setVertex(1, v_p);
                    edge->setMeasurement(measurement);
                    edge->setInformation(information);

                    optimizer.addEdge(edge);
                }
            }

            // Optionally, add odometry edges between consecutive poses
            Eigen::Matrix<double,6,6> odometryInformation = Eigen::Matrix<double,6,6>::Identity(); 
            /*
            double sigma_x = 0.05; // Standard deviation in x (meters)
            double sigma_y = 0.05; // Standard deviation in y (meters)
            double sigma_z = 1.0;  // Large uncertainty in z

            double sigma_roll = 1.0;   // Large uncertainty in roll
            double sigma_pitch = 1.0;  // Large uncertainty in pitch
            double sigma_yaw = 0.02;   // Standard deviation in yaw (radians)

            // Compute variances
            double sigma_x_squared = sigma_x * sigma_x;
            double sigma_y_squared = sigma_y * sigma_y;
            double sigma_z_squared = sigma_z * sigma_z;
            double sigma_roll_squared = sigma_roll * sigma_roll;
            double sigma_pitch_squared = sigma_pitch * sigma_pitch;
            double sigma_yaw_squared = sigma_yaw * sigma_yaw;

            // Set the information matrix
            odometryInformation(0, 0) = 1.0 / sigma_x_squared;
            odometryInformation(1, 1) = 1.0 / sigma_y_squared;
            odometryInformation(2, 2) = 0.0 / sigma_z_squared;
            odometryInformation(2, 2) = 0.0;
            odometryInformation(3, 3) = 1.0 / sigma_roll_squared;
            odometryInformation(3, 3) = 0.0;
            odometryInformation(4, 4) = 1.0 / sigma_pitch_squared;
            odometryInformation(4, 4) = 0.0; 
            odometryInformation(5, 5) = 1.0 / sigma_yaw_squared;
            
            */
            for (size_t i = 1; i < robotPositions.size(); ++i) {
                g2o::VertexSE3* v_se3_prev = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(i - 1));
                g2o::VertexSE3* v_se3_curr = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(i));

                // Compute the relative pose between consecutive robot positions
                Eigen::Isometry3d pose_prev = v_se3_prev->estimate();
                Eigen::Isometry3d pose_curr = v_se3_curr->estimate();
                Eigen::Isometry3d relative_pose = pose_prev.inverse() * pose_curr;

                // Constrain relative_pose to have zero z translation and zero roll and pitch
                Eigen::AngleAxisd rotation(relative_pose.rotation());
                Eigen::Vector3d eulerAngles = rotation.angle() * rotation.axis();
                eulerAngles[0] = 0.0; // Roll
                eulerAngles[1] = 0.0; // Pitch
                rotation = Eigen::AngleAxisd(eulerAngles.norm(), eulerAngles.normalized());

                relative_pose.linear() = rotation.toRotationMatrix();
                relative_pose.translation().z() = 0.0;

                // Create an edge between the two poses
                g2o::EdgeSE3* edge = new g2o::EdgeSE3();
                edge->setVertex(0, v_se3_prev);
                edge->setVertex(1, v_se3_curr);
                edge->setMeasurement(relative_pose);
                edge->setInformation(odometryInformation);

                optimizer.addEdge(edge);
            }

            // Optimize the graph
            optimizer.initializeOptimization();
            optimizer.optimize(10); // Number of iterations

            // Retrieve optimized robot poses
            for (size_t i = 0; i < robotPositions.size(); ++i) {
                g2o::VertexSE3* v_se3 = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(i));
                Eigen::Isometry3d optimized_pose = v_se3->estimate();
                // Convert to Eigen::Matrix4f if needed
                Eigen::Matrix4f optimizedPoseMatrix = optimized_pose.matrix().cast<float>();

                // Constrain the pose to have zero z, roll, and pitch
                Eigen::Matrix3f rotationMatrix = optimizedPoseMatrix.block<3,3>(0,0);
                Eigen::Vector3f eulerAngles = rotationMatrix.eulerAngles(0, 1, 2);
                eulerAngles[0] = 0.0f; // Roll
                eulerAngles[1] = 0.0f; // Pitch
                Eigen::Matrix3f constrainedRotation = Eigen::AngleAxisf(eulerAngles[2], Eigen::Vector3f::UnitZ()).toRotationMatrix();

                optimizedPoseMatrix.block<3,3>(0,0) = constrainedRotation;
                optimizedPoseMatrix(2,3) = 0.0f; // z translation

                // Update robotPositions with optimized poses
                robotPositions[i] = optimizedPoseMatrix;
            }

            // Retrieve optimized landmark positions
            for (const auto& landmarkEntry : landmarkIdMap) {
                int landmarkVertexId = landmarkEntry.second;
                g2o::VertexPointXYZ* v_p = dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(landmarkVertexId));
                Eigen::Vector3d optimized_landmark_position = v_p->estimate();
                // Update the landmark point in filtered_sharpness_cloud
                size_t index = landmarkEntry.first;
                filtered_sharpness_cloud->points[index].x = optimized_landmark_position.x();
                filtered_sharpness_cloud->points[index].y = optimized_landmark_position.y();
                filtered_sharpness_cloud->points[index].z = optimized_landmark_position.z();
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            // Loop over scans and optimized robot positions
            for (size_t i = 0; i < scans.size(); ++i) {
                // Transform the scan using the optimized robot position
                pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scan(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::transformPointCloud(*scans[i], *transformed_scan, robotPositions[i]);

                // Add the transformed scan to the final cloud
                *final_cloud += *transformed_scan;
            }

            // Save the final point cloud to a PCD file
            pcl::io::savePCDFileASCII("final_cloud.pcd", *final_cloud);
            std::cout << "Final point cloud saved to final_cloud.pcd" << std::endl;
            // Clear optimizer for next iteration if needed
            optimizer.clear();

            // Reset vertexId if necessary
            vertexId = 0;

            // Clear robotPositions and scans if you want to start fresh
            robotPositions.clear();
            scans.clear();
        }

               
    }
    return 0;
}
