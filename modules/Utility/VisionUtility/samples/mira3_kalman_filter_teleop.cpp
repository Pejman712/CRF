

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
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <queue>

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
//#include <pcl/point_cloud.h>
//#include <pcl/visualization/cloud_viewer.h>

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

#include <sys/select.h>
//Initialize the Robot


using crf::actuators::robotbase::SPSRobot;
using crf::math::kalmanfilter::KalmanFilter;
using crf::math::kalmanfilter::StateSpace;
using crf::math::kalmanfilter::SPSSystemModel;
using crf::math::kalmanfilter::SPSObservationModel;


// Maximum number of queued inputs
const int MAX_QUEUE_SIZE = 5;

// Buffer to store inputs
std::queue<char> inputQueue;

// Set to store currently pressed keys
std::set<char> pressedKeys;

// Last pressed key to detect changes
char lastKeyPressed = '\0';




// Function to make terminal non-blocking
void setNonBlockingInput(bool enable) {
    static struct termios oldt;
    static int oldf;

    if (enable) {
        struct termios newt;

        // Get the terminal settings for stdin
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;

        // Disable canonical mode and echo
        newt.c_lflag &= ~(ICANON | ECHO);

        // Set the new settings immediately
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        // Get the current file status flags
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);

        // Set the file descriptor to non-blocking
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    } else {
        // Restore the terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        // Restore the file status flags
        fcntl(STDIN_FILENO, F_SETFL, oldf);
    }
}




int getPressedKey() {
    fd_set fds;
    struct timeval tv;
    int ch;

    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    // Set timeout to zero for non-blocking
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    int ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
    if (ret > 0) {
        ch = getchar();
        return ch;
    }
    return -1;
}



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
    float x_l = 0.0; 
    float y_l = 0.0;
    float w_l = 0.0;

    float yaw = 0.0;

    Eigen::Matrix4f cumulativeTransformation = Eigen::Matrix4f::Identity();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> transformedClouds;

    // Create a pointer to store the previous scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr previousCloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Adjusted noise levels
    Eigen::MatrixXd system_noise = Eigen::MatrixXd::Identity(3, 3) * 0.01; // Less noise to wheel odometry
    Eigen::MatrixXd observation_noise = Eigen::MatrixXd::Identity(3, 3) * 0.1; // More noise to LiDAR registration

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

    // Initialize the State Space
    std::shared_ptr<StateSpace> state_space = std::make_shared<StateSpace>(initial_mean, initial_covariance);

    // Create an instance of the Kalman Filter
    KalmanFilter kf;
    float previous_gyro_z =0.0;
    float base_speed = 0.2;


        // Enable non-blocking input
    setNonBlockingInput(true);

    char input;

    std::array<double, 6> velocityArray = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Define accumulatedCloud outside the loop
    pcl::PointCloud<Point3D>::Ptr accumulatedCloud(new pcl::PointCloud<Point3D>);

    // Variables for time tracking and accumulation interval
    auto lastSaveTime = std::chrono::steady_clock::now();
    const std::chrono::milliseconds accumulationInterval(100); // Adjust as needed
    auto previousTime = std::chrono::steady_clock::now();
    static float prev_a_x = 0.0f, prev_a_y = 0.0f, prev_gyro_z = 0.0f;
    float a_x = 0.0f, a_y = 0.0f, gyro_z = 0.0f;
    float alpha = 0.5f; 
    while (!ctrl_c_pressed) {
        // Add a small sleep to prevent high CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Reset velocityArray to zero at the beginning of each loop
        velocityArray = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // Read all available input
        int ch;
        char lastInput = '\0';
        while ((ch = getPressedKey()) != -1) {
            lastInput = static_cast<char>(ch);
        }

        // If we have any input, process the last one
        if (lastInput != '\0') {
            if (lastInput == 'c') {
                std::cout << "Quit Program" << std::endl;
                break;
            }

            // Update velocity based on input
            if (lastInput == 'w') {
                std::cout << "Move Forward" << std::endl;
                velocityArray[0] = base_speed;
                velocityArray[1] = 0.0;
                velocityArray[5] = 0.0;
            }
            else if (lastInput == 's') {
                std::cout << "Move Backward" << std::endl;
                velocityArray[0] = -base_speed;
                velocityArray[1] = 0.0;
                velocityArray[5] = 0.0;
            }
            else if (lastInput == 'a') {
                std::cout << "Move Left" << std::endl;
                velocityArray[0] = 0.0;
                velocityArray[1] = -base_speed;
                velocityArray[5] = 0.0;
            }
            else if (lastInput == 'd') {
                std::cout << "Move Right" << std::endl;
                velocityArray[0] = 0.0;
                velocityArray[1] = base_speed;
                velocityArray[5] = 0.0;
            }
            else if (lastInput == 'e') {
                std::cout << "Turning Right" << std::endl;
                velocityArray[0] = 0.0;
                velocityArray[1] = 0.0;
                velocityArray[5] = base_speed;
            }
            else if (lastInput == 'q') {
                std::cout << "Turning left" << std::endl;
                velocityArray[0] = 0.0;
                velocityArray[1] = 0.0;
                velocityArray[5] = -base_speed;
            }
            else if (lastInput == ' ') {
                std::cout << "Stop" << std::endl;
                // velocityArray is already zero
            }
        }

        // Set the robot velocity
        crf::utility::types::TaskVelocity robotVelocity(velocityArray);
        bot->setTaskVelocity(robotVelocity);

        // Get current time
        auto currentTime = std::chrono::steady_clock::now();

        // Compute dt
        std::chrono::duration<double> elapsedTime = currentTime - previousTime;
        double dt = elapsedTime.count();

        // Update previous time
        previousTime = currentTime;

        // Get robot velocities
        boost::optional<crf::utility::types::TaskVelocity> pose = bot->getTaskVelocity();

        if (pose) {
            if (pose->size() > 5) {
                x_l = (*pose)[0]; 
                y_l = (*pose)[1];
                w_l = (*pose)[5];
            } else {
                x_l = 0.0;
                y_l = 0.0;
                w_l = 0.0;
            }
        } else {
            x_l = 0.0;
            y_l = 0.0;
            w_l = 0.0;
        }

        // Check for movement
        const float movement_threshold = 1e-1; // Adjust as appropriate
        if (std::abs(x_l) < movement_threshold && std::abs(y_l) < movement_threshold && std::abs(w_l) < movement_threshold) {
            // Robot is not moving, skip scan matching and IMU reading
            continue;
        }

        
       
        // Get the point cloud from the Lidar
        pcl::PointCloud<Point3D>::Ptr cloud = laser->getTargetPointCloud();
        
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsedSeconds = end - lastSaveTime;
        std::cout << "Acquisition time: " << elapsedSeconds.count() << " s" << std::endl;


        // Accumulate points from the current cloud into the accumulated cloud
        *accumulatedCloud += *cloud;

 
        if (currentTime - lastSaveTime >= accumulationInterval) {
        // Convert accumulated points to PCL PointXYZ for processing
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : accumulatedCloud->points) {
            pcl::PointXYZ xyz_point;
            xyz_point.x = point.x;
            xyz_point.y = point.y;
            xyz_point.z = point.z;
            xyz_cloud->points.push_back(xyz_point);
        }

        // Ensure the point cloud is properly organized before GICP
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

        // Filter the clouds based on height, if needed (optional)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_source(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_target(new pcl::PointCloud<pcl::PointXYZ>());

        Eigen::Matrix4f transformationMatrix;
        auto startTime = std::chrono::high_resolution_clock::now();

        // Use 'template' keyword to disambiguate the function template
        bool gicpConverged = crf::utility::visionutility::pointcloud::gicp::template gicp<pcl::PointXYZ>(
            previousCloud, xyz_cloud, transformationMatrix);

        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> processingTime = endTime - startTime;

        if (!gicpConverged) {


            // GICP did not converge, use previous tranfomation 

            /*
            // GICP did not converge, using previous transformation matrix (alternative behavior)
            cumulativeTransformation = cumulativeTransformation * transformationMatrix.inverse();
            std::cout << "\rPrevious Transformation Matrix used due to GICP failure." << std::flush;
            */

           
            // GICP did not converge, use wheel odometry
            double dx = x_l * dt;
            double dy = y_l * dt;
            double dtheta = w_l * dt;

            Eigen::Matrix4f odomTransformation = Eigen::Matrix4f::Identity();
            odomTransformation(0, 0) = std::cos(dtheta);
            odomTransformation(0, 1) = -std::sin(dtheta);
            odomTransformation(1, 0) = std::sin(dtheta);
            odomTransformation(1, 1) = std::cos(dtheta);
            odomTransformation(0, 3) = dx;
            odomTransformation(1, 3) = dy;

            cumulativeTransformation = cumulativeTransformation * odomTransformation;
            std::cout << "\rOdometry Translation: x = " << dx
                        << ", y = " << dy
                        << ", yaw = " << dtheta << "          " << std::flush;
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

        comparisonCount++;

        // Transform the target cloud using the cumulative transformation matrix
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedTargetCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*xyz_cloud, *transformedTargetCloud, cumulativeTransformation);


        // Reset the accumulated cloud
        accumulatedCloud->clear();

        // Reset the lastSaveTime
        lastSaveTime = currentTime;
        }

        // IMU DATA
        crf::sensors::imu::IMUSignals imuSignals = imu->getSignal();

        if (imuSignals.linearAcceleration) {
            float raw_a_x = imuSignals.linearAcceleration.value()[0];
            float raw_a_y = imuSignals.linearAcceleration.value()[1];
            // Apply low-pass filter
            a_x = alpha * raw_a_x + (1 - alpha) * prev_a_x;
            a_y = alpha * raw_a_y + (1 - alpha) * prev_a_y;
            prev_a_x = a_x;
            prev_a_y = a_y;
        }
        if (imuSignals.angularVelocity) {
            float raw_gyro_z = imuSignals.angularVelocity.value()[2];
            // Apply low-pass filter
            gyro_z = alpha * raw_gyro_z + (1 - alpha) * prev_gyro_z;
            prev_gyro_z = gyro_z;
        }

        // Use the cumulative transformation matrix to extract x_translation and y_translation
        float x_translation = cumulativeTransformation(0, 3);
        float y_translation = cumulativeTransformation(1, 3);
        float psi = yaw;

        Eigen::VectorXd control_input(6);
        control_input << x_l, y_l, w_l, a_x, a_y, gyro_z;

        system_model->setControl(control_input);

        Eigen::VectorXd measurement_input(3);
        measurement_input << x_translation, y_translation, psi;

        observation_model->setMeasurement(measurement_input);

        kf.prediction(system_model, state_space);
        kf.correction(observation_model, state_space);

        Eigen::VectorXd meanf = state_space->getMean();
        Eigen::VectorXd covf = state_space->getMean();

        std::cout << "\rGICP Translation: x = " << x_translation
            << ", y = " << y_translation
            << ", yaw = " << yaw << "          " << std::flush;
        std::cout << "State Mean:\n" << meanf << std::flush;
        std::cout << "State Covariance:\n" << std::flush;
    }

    setNonBlockingInput(false);
    return 0;
}