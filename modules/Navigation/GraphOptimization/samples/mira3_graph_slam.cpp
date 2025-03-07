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
#include <math.h>

//#include <gmock/gmock.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>  

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

#include "GraphOptimization/GraphOptimization.hpp"
#include "GraphOptimization/GraphOptimizationConfiguration.hpp"



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
#include <pcl/io/hdl_grabber.h>



//edge detection 

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <iostream>
#include <string>
#include <thread>  // For std::this_thread::sleep_for
#include <chrono>  // For std::chrono::milliseconds
#include <algorithm>  // For sorting sharpness values
#include "VisionUtility/PointCloud/Edge.hpp"
#include "VisionUtility/PointCloud/Communication.hpp"


#include <boost/optional.hpp>


#include "VisionUtility/PointCloud/Gicp.hpp"  


//Initialize the Robot


using crf::actuators::robotbase::SPSRobot;
using crf::math::kalmanfilter::KalmanFilter;
using crf::math::kalmanfilter::StateSpace;
using crf::math::kalmanfilter::SPSSystemModel;
using crf::math::kalmanfilter::SPSObservationModel;
using crf::applications::graphoptimization::GraphOptimizationConfiguration;
using crf::applications::graphoptimization::SlamParameters;




unsigned int maxIterations = 10;
unsigned int kNeighbour = 20; 
unsigned int percent = 20;
std::string resultName_ = "results";

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
        ("mapperConfigFile", po::value<std::string>(), "mapperConfigFile_");
        ("graphConfigFile", po::value<std::string>(), "graphConfigFile");

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
    std::string mapperConfigFile = vm["mapperConfigFile"].as<std::string>();
    std::string graphConfigFile = vm["graphConfigFile"].as<std::string>();

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

    ///Graphoptimizaiton declarations /////////////////////
    crf::applications::graphoptimization::GraphOptimization graph(mapperConfigFile);
    if (!graph.parse(graphConfigFile)) {
        std::cout << "Bad input configFile" << std::endl;
        return 0;
    }


    ///////////////////////////////// edge detection declration////////////////////////////

    Eigen::Matrix4f accumilatedTransformationMatrix = Eigen::Matrix4f::Identity();
    unsigned int kNeighbour = 20; 
    unsigned int precent = 90;  // Number of neighbors for edge detection
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumilated_scan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_accumilated_scan(new pcl::PointCloud<pcl::PointXYZ>);

    ////////////////////// Take configuration data//////////////////////////////////////////////
    boost::optional <SlamParameters> boostSlamData = graph.getSlamData();
    boost::optional <Eigen::Matrix4f> boostTranformationData = graph.getTransformationData();
    boost::optional <Eigen::Matrix4f> boostInitialPose = graph.getInitialPositionData();
    boost::optional <std::vector <std::vector <float>>> boostDoorsPosition =
        graph.getDoorsPosition();

    SlamParameters slamData = boostSlamData.get();
    Eigen::Matrix4f tranformationData = boostTranformationData.get();
    Eigen::Matrix4f initialPose = boostInitialPose.get();
    std::vector <std::vector <float>> doorsPosition = boostDoorsPosition.get();

    for (unsigned int i = 0; i < doorsPosition.size(); i++) {
        std::cout << "Door " << i << ": (" << doorsPosition[i][0] << ", " <<  doorsPosition[i][1]
            << ", " << doorsPosition[i][2] << ")" << std::endl;
    }

    double threshold = slamData.threshold;
    bool visualEnable = slamData.visualization;

    // Time declaration
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};

    // Motion matrix
    Eigen::Matrix4f SupposedMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previousMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f RealMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f robotPosition = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previousRobotPosition = Eigen::Matrix4f::Identity();
    unsigned int id_vertex {0};
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
        transformationMatrix = modifiedTransformationMatrix;
        robotPosition = modifiedTransformationMatrix;
        /////Visulization//////////////////
        /*
        pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*xyz_cloud, *map, transformationMatrix);
        if (!viewer->updatePointCloud(map, "map_cloud")) {
            viewer->addPointCloud(map, "map_cloud");
        }
        viewer->spinOnce(100); // Allow viewer to update

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust as needed
        */
///////////////////////////////////////Graph optimization///////////////////////////////////////////////////

        /*boostRobotPosition = localizer_->getPosition();
        if (!boostRobotPosition) {
            std::cout << "Unable to read from Robot Pose Estimator" << std::endl;
            return 0;
        }
        robotPosition = boostRobotPosition.get();
        previousRobotPosition = robotPosition;
        */
        while (id_vertex < maxIterations){
            previousRobotPosition = robotPosition;
            std::cout << "Generating graph..." << std::endl;
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // ----------------------------------------------------------
            // 0 - Position estimation
            // ----------------------------------------------------------
            double gtX{0.0}, gtY{0.0}, gtZ{0.0};
            if (id_vertex == maxIterations - 1) {
                std::cout << "DON'T MOVE THE ROBOT !!";
                std::cout << "Last position X: ";
                std::cin >> gtX;
                std::cout << "Last position Y: ";
                std::cin >> gtY;
                std::cout << "Last position Z: ";
                std::cin >> gtZ;
            }


            /*
            // Call robot pose estimator
            boostRobotPosition = localizer_->getPosition();
            if (!boostRobotPosition) {
                std::cout << "Unable to read from Robot Pose Estimator" << std::endl;
                return 0;
            }
            robotPosition = boostRobotPosition.get();
            // std::cout << "Pose estimated: " << robotPosition << std::endl;
            */
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
                // 3 - Comparation for edge detection
                // ----------------------------------------------------------
                    // Set parameters for sharpness computation

                /*for each 30 scans , tansfer the scan based on robotPosition eigen:matrix 4*4 and create two pcl files previous_accumilated_scan and accumilated scan, apply
                filterTopSharpness and then gicp */
                
                // Assume 'scans' is a vector of point clouds and 'robotPositions' is a vector of Eigen::Matrix4f
                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scans;  // Populate this with your scans
                std::vector<Eigen::Matrix4f> robotPositions;             // Populate this with your robot positions

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

                // Apply GICP
                bool gicpConverged = crf::utility::visionutility::pointcloud::gicp::template gicp<pcl::PointXYZ>(
                    previous_accumilated_scan, accumilated_scan, accumilatedTransformationMatrix);

                if (gicpConverged) {
                    Eigen::Matrix4f RealMotion = accumilatedTransformationMatrix;
                } else {
                    std::cerr << "GICP did not converge!" << std::endl;
                    return -1;
                }
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
                } else if (id_vertex == maxIterations - 1) {
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

                id_vertex++;
                }
        }
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

    // nodePosition to be added for visulization

    return 0;
}
