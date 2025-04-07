
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
#include <set>

#include <Eigen/Dense>

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

// Unitree Lidar and IMU 
#include "Laser/UnitreeL1/UnitreeL1.hpp"  
#include "IMU/UnitreeL1IMU/UnitreeL1IMU.hpp"

// PCL includes
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

// Eigen includes
#include <Eigen/Geometry>

// Vision Utility includes
#include "VisionUtility/PointCloud/Gicp.hpp"  

#include <sys/select.h>

// g2o headers
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/types_slam2d.h>


// Initialize the IMU
#include "IMU/RealSenseIMU/RealSenseIMU.hpp"


// Initialize the Robot
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

#include <sys/select.h>

namespace po = boost::program_options;
bool ctrl_c_pressed = false;

// Signal handler for Ctrl+C
void ctrlc(int) {
    std::cout << "\n[DEBUG] Ctrl+C pressed. Preparing to exit..." << std::endl;
    ctrl_c_pressed = true;
}

// Function to make terminal non-blocking
void setNonBlockingInput(bool enable) {
    static struct termios oldt;
    static int oldf;
    static bool initialized = false;

    if (enable && !initialized) {
        std::cout << "[DEBUG] Enabling non-blocking input." << std::endl;
        struct termios newt;

        // Get the terminal settings for stdin
        if (tcgetattr(STDIN_FILENO, &oldt) < 0) {
            std::cerr << "[ERROR] tcgetattr failed." << std::endl;
            return;
        }
        newt = oldt;

        // Disable canonical mode and echo
        newt.c_lflag &= ~(ICANON | ECHO);

        // Set the new settings immediately
        if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) < 0) {
            std::cerr << "[ERROR] tcsetattr failed." << std::endl;
            return;
        }

        // Get the current file status flags
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (oldf < 0) {
            std::cerr << "[ERROR] fcntl F_GETFL failed." << std::endl;
            return;
        }

        // Set the file descriptor to non-blocking
        if (fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK) < 0) {
            std::cerr << "[ERROR] fcntl F_SETFL failed." << std::endl;
            return;
        }

        initialized = true;
    } else if (!enable && initialized) {
        std::cout << "[DEBUG] Disabling non-blocking input." << std::endl;
        // Restore the terminal settings
        if (tcsetattr(STDIN_FILENO, TCSANOW, &oldt) < 0) {
            std::cerr << "[ERROR] tcsetattr (restore) failed." << std::endl;
            return;
        }

        // Restore the file status flags
        if (fcntl(STDIN_FILENO, F_SETFL, oldf) < 0) {
            std::cerr << "[ERROR] fcntl F_SETFL (restore) failed." << std::endl;
            return;
        }

        initialized = false;
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
        std::cout << "[DEBUG] Key pressed: " << static_cast<char>(ch) << std::endl;
        return ch;
    }
    return -1;
}

int main(int argc, char* argv[]) {
    std::cout << "[DEBUG] Program started." << std::endl;

    // ------------------------ Command Line Parsing ------------------------
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("ethercat_port", po::value<std::string>(), "EtherCAT port name (e.g., enp2s0)")
        ("sps_config", po::value<std::string>(), "SPSRobot configuration file path")
        ("device", po::value<std::string>(), "Serial port name (e.g., /dev/ttyUSB0)")
        ("unitree_config", po::value<std::string>(), "UnitreeL1 configuration file")
        ("config", po::value<std::string>(), "General configuration file path");

    po::variables_map vm;
    try {
        std::cout << "[DEBUG] Parsing command line arguments." << std::endl;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Error parsing command line arguments: " << e.what() << std::endl;
        std::cerr << desc << std::endl;
        return -1;
    }

    if (vm.count("help")) {
        std::cout << "[DEBUG] Help requested." << std::endl;
        std::cout << desc << std::endl;
        return 0;
    }

    // Check for required arguments and handle them appropriately
    if (!vm.count("ethercat_port")) {
        std::cerr << "[ERROR] Missing ethercat port argument." << std::endl << desc << std::endl;
        return -1;
    }
    if (!vm.count("sps_config")) {
        std::cerr << "[ERROR] Missing SPSRobot configuration file argument." << std::endl << desc << std::endl;
        return -1;
    }
    if (!vm.count("device")) {
        std::cerr << "[ERROR] Missing device argument." << std::endl << desc << std::endl;
        return -1;
    }
    if (!vm.count("unitree_config")) {
        std::cerr << "[ERROR] Missing UnitreeL1 configuration file argument." << std::endl << desc << std::endl;
        return -1;
    }
    if (!vm.count("config")) {
        std::cerr << "[ERROR] Missing configuration file argument." << std::endl << desc << std::endl;
        return -1;
    }

    std::string ethercat_port = vm["ethercat_port"].as<std::string>();
    std::string sps_config_path = vm["sps_config"].as<std::string>();
    std::string device = vm["device"].as<std::string>();
    std::string unitree_config_path = vm["unitree_config"].as<std::string>();
    std::string config_path = vm["config"].as<std::string>();

    std::cout << "[DEBUG] Parsed Arguments:" << std::endl;
    std::cout << "  EtherCAT Port: " << ethercat_port << std::endl;
    std::cout << "  SPS Config Path: " << sps_config_path << std::endl;
    std::cout << "  Device: " << device << std::endl;
    std::cout << "  Unitree Config Path: " << unitree_config_path << std::endl;
    std::cout << "  Config Path: " << config_path << std::endl;

    // ------------------------ Load General Configuration ------------------------
    std::ifstream config_file(config_path);
    if (!config_file.is_open()) {
        std::cerr << "[ERROR] Unable to open configuration file: " << config_path << std::endl;
        return -1;
    }
    nlohmann::json config;
    try {
        config_file >> config;
        std::cout << "[DEBUG] General configuration loaded successfully." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to parse configuration: " << e.what() << std::endl;
        return -1;
    }

    // Extract parameters from configuration
    std::string serialNumber = config["serialNumber"].get<std::string>();
    float low_intensity_bound = config["lowIntensityBound"].get<double>();
    float high_intensity_bound = config["highIntensityBound"].get<double>();
    float base_speed = config["baseSpeed"].get<double>();
    float movement_threshold = config["movementThreshold"].get<double>();
    float alpha = config["alpha"].get<double>();
    int optimize_every = config["optimizeEvery"].get<int>();
    std::chrono::milliseconds accumulationInterval(config["accumulationInterval"].get<int>());

    // Load matrices
    Eigen::MatrixXd system_noise(3, 3);
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 3; ++j)
            system_noise(i, j) = config["systemNoise"][i][j].get<double>();

    Eigen::MatrixXd observation_noise(3, 3);
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 3; ++j)
            observation_noise(i, j) = config["observationNoise"][i][j].get<double>();

    Eigen::VectorXd initial_mean(3);
    for (size_t i = 0; i < 3; ++i)
        initial_mean(i) = config["initialMean"][i].get<double>();

    Eigen::MatrixXd initial_covariance(3, 3);
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 3; ++j)
            initial_covariance(i, j) = config["initialCovariance"][i][j].get<double>();

    Eigen::Vector3d information_matrix_entries;
    for (size_t i = 0; i < 3; ++i)
        information_matrix_entries(i) = config["informationMatrix"][i].get<double>();

    // ------------------------ Initialize SPSRobot ------------------------
    std::cout << "[DEBUG] Initializing SPSRobot." << std::endl;
    std::ifstream sps_config_file(sps_config_path);
    if (!sps_config_file.is_open()) {
        std::cerr << "[ERROR] Unable to open SPSRobot configuration file: " << sps_config_path << std::endl;
        return -1;
    }
    nlohmann::json sps_config;
    try {
        sps_config_file >> sps_config;
        std::cout << "[DEBUG] SPSRobot configuration loaded successfully." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to parse SPSRobot configuration: " << e.what() << std::endl;
        return -1;
    }

    std::shared_ptr<SPSRobot> bot = std::make_shared<SPSRobot>(ethercat_port.c_str(), sps_config);
    if (!bot->initialize()) {
        std::cerr << "[ERROR] Could not initialize the SPSRobot." << std::endl;
        return -1;
    }
    std::cout << "[DEBUG] SPSRobot initialized successfully." << std::endl;

    // ------------------------ Initialize UnitreeL1 Lidar ------------------------
    std::cout << "[DEBUG] Initializing UnitreeL1 Lidar." << std::endl;
    std::ifstream unitree_config_file(unitree_config_path);
    if (!unitree_config_file.is_open()) {
        std::cerr << "[ERROR] Unable to open UnitreeL1 configuration file: " << unitree_config_path << std::endl;
        return -1;
    }
    nlohmann::json unitree_config;
    try {
        unitree_config_file >> unitree_config;
        std::cout << "[DEBUG] UnitreeL1 configuration loaded successfully." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to parse UnitreeL1 configuration: " << e.what() << std::endl;
        return -1;
    }

    std::unique_ptr<crf::sensors::laser::UnitreeL1> laser = std::make_unique<crf::sensors::laser::UnitreeL1>(
        crf::sensors::laser::LaserConnectionType::Serial, device, 2000000, unitree_config);

    if (!laser->initialize()) {
        std::cerr << "[ERROR] Could not initialize the UnitreeL1 Lidar." << std::endl;
        return -1;
    }
    std::cout << "[DEBUG] UnitreeL1 Lidar initialized successfully." << std::endl;

    // ------------------------ Initialize Realsense IMU  ------------------------
    std::unique_ptr<crf::sensors::imu::RealSenseIMU> imu {new crf::sensors::imu::RealSenseIMU(serialNumber)};

    // Initialize the IMU
    if (!imu->initialize()) {
        std::cerr << "Failed to initialize RealSense IMU" << std::endl;
        return -1;
    }

    // Calibrate the IMU to remove bias and set up filters
    if (!imu->calibrate()) {
        std::cerr << "IMU calibration failed" << std::endl;
        return -1;
    }
    std::cout << "RealSense IMU calibration complete" << std::endl;
    std::cout << "RealSense IMU initialized successfully" << std::endl;

    // ------------------------ Set Up Signal Handler ------------------------
    std::cout << "[DEBUG] Setting up signal handler for SIGINT." << std::endl;
    signal(SIGINT, ctrlc);

    // ------------------------ Initialize Kalman Filter ------------------------
    std::cout << "[DEBUG] Initializing Kalman Filter." << std::endl;

    int comparisonCount = 0;
    int nonConvergenceCount = 0;
    float x_l = 0.0; 
    float y_l = 0.0;
    float w_l = 0.0;

    float yaw = 0.0;

    Eigen::Matrix4f cumulativeTransformation = Eigen::Matrix4f::Identity();
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> transformedClouds;

    pcl::PointCloud<pcl::PointXYZI>::Ptr previousCloud(new pcl::PointCloud<pcl::PointXYZI>());

    // Adjusted noise levels from configuration
    // Initial state estimate from configuration

    // Create control input and measurement vectors
    Eigen::VectorXd control_input_vec = Eigen::VectorXd::Zero(6);  // 6-dimensional control input
    Eigen::VectorXd measurement_input_vec = Eigen::VectorXd::Zero(3);    // 3-dimensional measurement

    // Initialize the System Model
    std::shared_ptr<SPSSystemModel> system_model = std::make_shared<SPSSystemModel>(system_noise);
    std::cout << "[DEBUG] SPSSystemModel initialized with system_noise:\n" << system_noise << std::endl;

    // Initialize the Observation Model
    std::shared_ptr<SPSObservationModel> observation_model = std::make_shared<SPSObservationModel>(observation_noise);
    std::cout << "[DEBUG] SPSObservationModel initialized with observation_noise:\n" << observation_noise << std::endl;

    // Initialize the State Space
    std::shared_ptr<StateSpace> state_space = std::make_shared<StateSpace>(initial_mean, initial_covariance);
    std::cout << "[DEBUG] StateSpace initialized with initial_mean:\n" << initial_mean << "\nand initial_covariance:\n" << initial_covariance << std::endl;

    // Create an instance of the Kalman Filter
    KalmanFilter kf;
    float previous_gyro_z = 0.0;

    // ------------------------ Open CSV Files ------------------------
    // Open CSV Files
    std::ofstream gicpOdomFile("gicp_odom_data.csv");
    std::ofstream kalmanFilterFile("kalman_filter_data.csv");
    std::ofstream graphOptimizedPosesFile("graph_optimized_poses.csv");

    // Check if files are open
    if (!gicpOdomFile.is_open()) {
        std::cerr << "[ERROR] Could not open gicp_odom_data.csv for writing." << std::endl;
        return -1;
    }

    if (!kalmanFilterFile.is_open()) {
        std::cerr << "[ERROR] Could not open kalman_filter_data.csv for writing." << std::endl;
        return -1;
    }

    if (!graphOptimizedPosesFile.is_open()) {
        std::cerr << "[ERROR] Could not open graph_optimized_poses.csv for writing." << std::endl;
        return -1;
    }

    // Write headers
    gicpOdomFile << "time,x,y,theta" << std::endl;
    kalmanFilterFile << "time,x,y,yaw" << std::endl;
    graphOptimizedPosesFile << "time,vertex_id,x,y,yaw" << std::endl;

    // ------------------------ Initialize g2o Optimizer ------------------------
    std::cout << "[DEBUG] Initializing g2o optimizer." << std::endl;
    // Initialize g2o optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false); // Set to true for detailed optimization logs
    std::cout << "[DEBUG] g2o optimizer created." << std::endl;

    // Choose the solver
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3, 3> > BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    std::unique_ptr<LinearSolverType> linearSolver (new LinearSolverType());
    std::unique_ptr<BlockSolverType> solver (new BlockSolverType(std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(solver));

    optimizer.setAlgorithm(optimizationAlgorithm);
    std::cout << "[DEBUG] g2o optimization algorithm set." << std::endl;

    // Vertex ID counter
    int vertex_id = 0;

    // Initialize first vertex (fixed)
    g2o::VertexSE2* v0 = new g2o::VertexSE2();
    v0->setId(vertex_id);
    v0->setEstimate(g2o::SE2(0.0, 0.0, 0.0)); // Initial pose
    v0->setFixed(true); // Fix the first pose
    optimizer.addVertex(v0);
    std::cout << "[DEBUG] Added initial fixed vertex with ID " << vertex_id << std::endl;
    vertex_id++;

    // ------------------------ Initialize Variables for Graph Optimization ------------------------
    // Vector to store optimized poses (optional)
    std::vector<g2o::VertexSE2*> graph_vertices;

    // Define accumulatedCloud outside the loop
    pcl::PointCloud<Point3D>::Ptr accumulatedCloud(new pcl::PointCloud<Point3D>());

    // Variables for time tracking and accumulation interval
    auto lastSaveTime = std::chrono::steady_clock::now();
    auto previousTime = std::chrono::steady_clock::now();

    Eigen::Matrix4f odomTransformation = Eigen::Matrix4f::Identity();
    static float prev_a_x = 0.0f, prev_a_y = 0.0f, prev_gyro_z = 0.0f;
    float a_x = 0.0f, a_y = 0.0f, gyro_z = 0.0f;

    // ------------------------ Enable Non-Blocking Input ------------------------
    std::cout << "[DEBUG] Setting terminal to non-blocking mode." << std::endl;
    setNonBlockingInput(true);

    char input;

    std::array<double, 6> velocityArray = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::cout << "[DEBUG] Entering main loop. Press 'c' to quit." << std::endl;

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
            //std::cout << "[DEBUG] Processing input character: " << lastInput << std::endl;
        }

        // If we have any input, process the last one
        if (lastInput != '\0') {
            if (lastInput == 'c') {
                std::cout << "[DEBUG] Quit command received." << std::endl;
                break;
            }

            // Update velocity based on input
            if (lastInput == 'w') {
                //std::cout << "[DEBUG] Command: Move Forward" << std::endl;
                velocityArray[0] = base_speed;
                velocityArray[1] = 0.0;
                velocityArray[5] = 0.0;
            }
            else if (lastInput == 's') {
                //std::cout << "[DEBUG] Command: Move Backward" << std::endl;
                velocityArray[0] = -base_speed;
                velocityArray[1] = 0.0;
                velocityArray[5] = 0.0;
            }
            else if (lastInput == 'a') {
                //std::cout << "[DEBUG] Command: Move Left" << std::endl;
                velocityArray[0] = 0.0;
                velocityArray[1] = -base_speed;
                velocityArray[5] = 0.0;
            }
            else if (lastInput == 'd') {
                //std::cout << "[DEBUG] Command: Move Right" << std::endl;
                velocityArray[0] = 0.0;
                velocityArray[1] = base_speed;
                velocityArray[5] = 0.0;
            }
            else if (lastInput == 'e') {
               // std::cout << "[DEBUG] Command: Turning Right" << std::endl;
                velocityArray[0] = 0.0;
                velocityArray[1] = 0.0;
                velocityArray[5] = base_speed;
            }
            else if (lastInput == 'q') {
                //std::cout << "[DEBUG] Command: Turning Left" << std::endl;
                velocityArray[0] = 0.0;
                velocityArray[1] = 0.0;
                velocityArray[5] = -base_speed;
            }
            else if (lastInput == ' ') {
                //std::cout << "[DEBUG] Command: Stop" << std::endl;
                // velocityArray is already zero
            }
            else {
                std::cout << "[WARNING] Unknown command: " << lastInput << std::endl;
            }
        }

        // Set the robot velocity
        //std::cout << "[DEBUG] Setting robot velocity: [";
        for (size_t i = 0; i < velocityArray.size(); ++i) {
            std::cout << velocityArray[i];
            if (i != velocityArray.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        crf::utility::types::TaskVelocity robotVelocity(velocityArray);
        bot->setTaskVelocity(robotVelocity);

        // Get current time
        auto currentTime = std::chrono::steady_clock::now();
        double timeInSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime.time_since_epoch()).count() / 1000.0;

        // Compute dt
        std::chrono::duration<double> elapsedTime = currentTime - previousTime;
        double dt = elapsedTime.count();
        //std::cout << "[DEBUG] Time elapsed since last loop: " << dt << " seconds." << std::endl;

        // Update previous time
        previousTime = currentTime;

        // Get robot velocities
        boost::optional<crf::utility::types::TaskVelocity> pose = bot->getTaskVelocity();

        if (pose) {
            if (pose->size() > 5) {
                x_l = (*pose)[0]; 
                y_l = (*pose)[1];
                w_l = (*pose)[5];
               // std::cout << "[DEBUG] Retrieved robot velocities: x_l = " << x_l << ", y_l = " << y_l << ", w_l = " << w_l << std::endl;
            } else {
                x_l = 0.0;
                y_l = 0.0;
                w_l = 0.0;
                //std::cout << "[WARNING] Retrieved robot velocity vector is smaller than expected." << std::endl;
            }
        } else {
            x_l = 0.0;
            y_l = 0.0;
            w_l = 0.0;
            //std::cout << "[WARNING] Failed to retrieve robot velocities." << std::endl;
        }

        // Check for movement
        if (std::abs(x_l) < movement_threshold && std::abs(y_l) < movement_threshold && std::abs(w_l) < movement_threshold) {
            //std::cout << "[DEBUG] Robot is stationary. Skipping scan matching and IMU reading." << std::endl;
            continue;
        } else {
            //std::cout << "[DEBUG] Robot is moving. Proceeding with sensor data processing." << std::endl;
        }

        // IMU DATA
            crf::sensors::imu::IMUSignals imuSignals = imu->getSignal();

            if (imuSignals.linearAcceleration) { 
                const std::array<double, 3>& linearAcceleration = imuSignals.linearAcceleration.value();
                a_x = linearAcceleration[2];
                a_y = linearAcceleration[0];
            }

            if (imuSignals.position) {
                double rad2degree = 1;// 180/M_PI;
                const std::array<double, 3>& position = imuSignals.position.value();
                gyro_z = position[0];
                //std::cout << "Position: Roll: ["
                //        << position[0] * rad2degree << ", Pitch: "
                //        << position[1] * rad2degree << ", Yaw: "
                //        << position[2] * rad2degree << "]" << std::endl;
            }
        // Get the point cloud from the Lidar
        pcl::PointCloud<Point3D>::Ptr cloud = laser->getTargetPointCloud();
        if (cloud->empty()) {
            //std::cerr << "[ERROR] Retrieved empty point cloud from Lidar." << std::endl;
            continue;
        }
        //std::cout << "[DEBUG] Retrieved point cloud with " << cloud->points.size() << " points." << std::endl;
        
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsedSeconds = end - lastSaveTime;
        //std::cout << "[DEBUG] Acquisition time: " << elapsedSeconds.count() << " s" << std::endl;
        
        // Accumulate points from the current cloud into the accumulated cloud
        *accumulatedCloud += *cloud;
        //std::cout << "[DEBUG] Accumulated cloud now has " << accumulatedCloud->points.size() << " points." << std::endl;
        
        if (currentTime - lastSaveTime >= accumulationInterval) {
            //std::cout << "[DEBUG] Accumulation interval reached. Processing accumulated point cloud." << std::endl;
        
            // Convert accumulated points to PCL PointXYZ for processing
            pcl::PointCloud<pcl::PointXYZI>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            for (const auto& point : accumulatedCloud->points) {
                pcl::PointXYZI xyz_point;
                xyz_point.x = point.x;
                xyz_point.y = point.y;
                xyz_point.z = point.z;
                xyz_point.intensity = point.intensity;
                xyz_cloud->points.push_back(xyz_point);
            }
            //std::cout << "[DEBUG] Converted accumulated cloud to xyz_cloud with " << xyz_cloud->points.size() << " points." << std::endl;
        
            // Ensure the point cloud is properly organized before GICP
            xyz_cloud->width = xyz_cloud->points.size();
            xyz_cloud->height = 1;  // Unorganized point cloud
            xyz_cloud->is_dense = true;  // Optional
            //std::cout << "[DEBUG] Organized xyz_cloud: width = " << xyz_cloud->width << ", height = " << xyz_cloud->height << std::endl;
        
            // Ensure that the previous cloud is also organized correctly
            previousCloud->width = previousCloud->points.size();
            previousCloud->height = 1;  // Unorganized point cloud
            previousCloud->is_dense = true;  // Optional        
            //std::cout << "[DEBUG] Organized previousCloud: width = " << previousCloud->width << ", height = " << previousCloud->height << std::endl;
        
            // Skip the comparison if this is the first scan (i.e., no previous cloud available)
            if (previousCloud->empty()) {
                //std::cout << "[DEBUG] previousCloud is empty. Storing current xyz_cloud as previousCloud." << std::endl;
                *previousCloud = *xyz_cloud;  // Store the first scan as the previous cloud
                lastSaveTime = currentTime; // Reset the save time
                continue;  // Skip to the next iteration
            }
        
            // Filter the clouds based on height, if needed (optional)
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_source(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_target(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PassThrough<pcl::PointXYZI> pass;
            pass.setInputCloud(xyz_cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0, 25.0); 
            pass.filter(*cloud_filtered_source);
            //std::cout << "[DEBUG] Applied passthrough filter. Source cloud now has " << cloud_filtered_source->points.size() << " points." << std::endl;
        
            pass.setInputCloud(previousCloud);
            pass.filter(*cloud_filtered_target);
            //std::cout << "[DEBUG] Applied passthrough filter. Target cloud now has " << cloud_filtered_target->points.size() << " points." << std::endl;
            
            auto intensityFilter = [&](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
                if (cloud->points.size() > 0) {
                    // Extract intensity values
                    std::vector<float> intensities;
                    intensities.reserve(cloud->points.size());
                    for (const auto& point : cloud->points) {
                        intensities.push_back(point.intensity);
                    }
        
                    // Sort intensity values
                    std::sort(intensities.begin(), intensities.end());
        
                    // Compute percentile indices
                    size_t idx_low = static_cast<size_t>(intensities.size() * low_intensity_bound);
                    size_t idx_high = static_cast<size_t>(intensities.size() * high_intensity_bound);
                    float intensity_low = intensities[idx_low];
                    float intensity_high = intensities[idx_high];
        
                    // Apply intensity filter
                    pcl::PassThrough<pcl::PointXYZI> intensityPass;
                    intensityPass.setFilterFieldName("intensity");
                    intensityPass.setFilterLimits(intensity_low, intensity_high);
                    intensityPass.setInputCloud(cloud);
                    pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_filtered(new pcl::PointCloud<pcl::PointXYZI>());
                    intensityPass.filter(*intensity_filtered);
        
                    return intensity_filtered;
                } else {
                    std::cerr << "Cloud is empty after z-filtering." << std::endl;
                    return pcl::PointCloud<pcl::PointXYZI>::Ptr();
                }
            };
        
            // Filter source and target clouds based on intensity
            cloud_filtered_source = intensityFilter(cloud_filtered_source);
            cloud_filtered_target = intensityFilter(cloud_filtered_target);
        
            if (!cloud_filtered_source || !cloud_filtered_target) {
                std::cerr << "Error in intensity filtering." << std::endl;
                continue;
            }
            Eigen::Matrix4f transformationMatrix;
        
            // Create rotation matrix from gyroYaw
            Eigen::Matrix3f gyroRotationMatrix;
            gyroRotationMatrix = Eigen::AngleAxisf(gyro_z, Eigen::Vector3f::UnitZ()).matrix();
        
            // Create transformation matrix
            Eigen::Matrix4f gyroTransformation = Eigen::Matrix4f::Identity();
            gyroTransformation.block<3,3>(0,0) = gyroRotationMatrix;
        
            // Transform cloud_filtered_source
            pcl::transformPointCloud(*cloud_filtered_source, *cloud_filtered_source, gyroTransformation);
        
            auto startTime = std::chrono::high_resolution_clock::now();
        
            // Use 'template' keyword to disambiguate the function template
            //std::cout << "[DEBUG] Starting GICP alignment." << std::endl;
            bool gicpConverged = crf::utility::visionutility::pointcloud::gicp::template gicp<pcl::PointXYZI>(
                cloud_filtered_target, cloud_filtered_source, transformationMatrix);
            //std::cout << "[DEBUG] GICP alignment completed. Converged: " << (gicpConverged ? "Yes" : "No") << std::endl;
        
            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> processingTime = endTime - startTime;
            //std::cout << "[DEBUG] GICP processing time: " << processingTime.count() << " seconds." << std::endl;
        
            if (!gicpConverged) {
                // GICP did not converge, use wheel odometry
                //std::cout << "[DEBUG] GICP did not converge. Using wheel odometry for transformation." << std::endl;
                double dx = x_l * dt;
                double dy = y_l * dt;
                double dtheta = w_l * dt;
        
                odomTransformation(0, 0) = std::cos(dtheta);
                odomTransformation(0, 1) = -std::sin(dtheta);
                odomTransformation(1, 0) = std::sin(dtheta);
                odomTransformation(1, 1) = std::cos(dtheta);
                odomTransformation(0, 3) = dx;
                odomTransformation(1, 3) = dy;
        
                cumulativeTransformation = cumulativeTransformation * odomTransformation.inverse();
        
                // Extract x, y, yaw from cumulativeTransformation
                float x_translation = cumulativeTransformation(0, 3);
                float y_translation = cumulativeTransformation(1, 3);
                float psi = atan2(cumulativeTransformation(1, 0), cumulativeTransformation(0, 0));
        
                // Save to CSV file
                gicpOdomFile << timeInSeconds << "," << x_translation << "," << y_translation << "," << psi << std::endl;
        
                std::cout << "[DEBUG] Odometry Translation: dx = " << dx
                          << ", dy = " << dy
                          << ", dtheta = " << dtheta << std::endl;
                nonConvergenceCount++;
            } else {
                // Process the transformation as before
                //std::cout << "[DEBUG] GICP converged. Processing transformation matrix." << std::endl;
                Eigen::Matrix3f rotationMatrix = transformationMatrix.block<3,3>(0,0);
                Eigen::Vector3f translationVector = transformationMatrix.block<3,1>(0,3);
                float gicpYaw = atan2(rotationMatrix(1,0), rotationMatrix(0,0));
                //std::cout << "[DEBUG] Extracted yaw from rotation matrix: " << gicpYaw << std::endl;
        
                std::cout << "[DEBUG] Gicp Translation: x = " << translationVector[0]
                          << ", y = " << translationVector[1]
                          << ", yaw = " << gicpYaw << std::endl;
        
                Eigen::Matrix3f newRotationMatrix;
                newRotationMatrix << cos(gicpYaw), -sin(gicpYaw), 0.0f,
                                    sin(gicpYaw),  cos(gicpYaw), 0.0f,
                                        0.0f,      0.0f, 1.0f;
                //std::cout << "[DEBUG] New rotation matrix:\n" << newRotationMatrix << std::endl;
        
                translationVector[2] = 0.0f;
                Eigen::Matrix4f modifiedTransformationMatrix = Eigen::Matrix4f::Identity();
                modifiedTransformationMatrix.block<3,3>(0,0) = newRotationMatrix;
                modifiedTransformationMatrix.block<3,1>(0,3) = translationVector;
                transformationMatrix = modifiedTransformationMatrix * gyroTransformation;
        
                cumulativeTransformation = cumulativeTransformation * transformationMatrix.inverse();
        
                // Extract x, y, yaw from cumulativeTransformation
                float x_translation = cumulativeTransformation(0, 3);
                float y_translation = cumulativeTransformation(1, 3);
                float psi = atan2(cumulativeTransformation(1, 0), cumulativeTransformation(0, 0));
        
                // Save to CSV file
                gicpOdomFile << timeInSeconds << "," << x_translation << "," << y_translation << "," << psi << std::endl;
            }
        
            // Update the previous cloud for the next iteration
            *previousCloud = *xyz_cloud;  // Save the current scan as the previous scan
            //std::cout << "[DEBUG] Updated previousCloud with current xyz_cloud." << std::endl;
        
            comparisonCount++;
            //std::cout << "[DEBUG] Total comparisons: " << comparisonCount << ", Non-convergences: " << nonConvergenceCount << std::endl;
        
            // ------------------------ Kalman Filter Prediction and Correction ------------------------
            //std::cout << "[DEBUG] Retrieving IMU data for Kalman Filter." << std::endl;
            // Use the cumulative transformation matrix to extract x_translation and y_translation
            float x_translation = cumulativeTransformation(0, 3);
            float y_translation = cumulativeTransformation(1, 3);
            float psi = atan2(cumulativeTransformation(1,0), cumulativeTransformation(0,0));
            std::cout << "Gicp/odom cumulitive: x_translation = " << x_translation
                      << ", y_translation = " << y_translation
                      << ", psi (yaw) = " << psi << std::endl;
        
            Eigen::VectorXd control_input_vec(6);
            control_input_vec << x_l, y_l, w_l, a_x, a_y, gyro_z;
            system_model->setControl(control_input_vec);
            //std::cout << "[DEBUG] Set control input vector:\n" << control_input_vec << std::endl;
            Eigen::VectorXd measurement_input_vec(3);
            measurement_input_vec << x_translation, y_translation, psi;
            observation_model->setMeasurement(measurement_input_vec);
            //std::cout << "[DEBUG] Set measurement input vector:\n" << measurement_input_vec << std::endl;
        
            //std::cout << "[DEBUG] Performing Kalman Filter prediction." << std::endl;
            kf.prediction(system_model, state_space);
            //std::cout << "[DEBUG] Prediction step completed." << std::endl;
        
            //std::cout << "[DEBUG] Performing Kalman Filter correction." << std::endl;
            kf.correction(observation_model, state_space);
            //std::cout << "[DEBUG] Correction step completed." << std::endl;
        
            Eigen::VectorXd meanf = state_space->getMean();
        
            // Save Kalman filter mean to CSV file
            kalmanFilterFile << timeInSeconds << "," << meanf[0] << "," << meanf[1] << "," << meanf[2] << std::endl;
        
            // ------------------------ Add Pose and Edge to g2o Graph ------------------------
            //std::cout << "[DEBUG] Adding pose and edge to g2o graph." << std::endl;
            // Extract the current pose from the Kalman Filter
            double current_x = meanf[0];
            double current_y = meanf[1];
            double current_yaw = meanf[2];
            std::cout << "[DEBUG] Current Pose from Kalman Filter: x = " << current_x
                      << ", y = " << current_y << ", yaw = " << current_yaw << std::endl;
        
            // Create a new vertex for the current pose
            g2o::VertexSE2* vi = new g2o::VertexSE2();
            vi->setId(vertex_id);
            vi->setEstimate(g2o::SE2(current_x, current_y, current_yaw));
        
            // If it's the first vertex, fix it
            if (vertex_id == 0) {
                vi->setFixed(true);
                //std::cout << "[DEBUG] Added first fixed vertex with ID " << vertex_id << std::endl;
            } else {
                std::cout << "[DEBUG] Added vertex with ID " << vertex_id << std::endl;
            }
        
            optimizer.addVertex(vi);
        
            // If not the first pose, add an edge from the previous pose
            if (vertex_id > 0) {
                g2o::EdgeSE2* edge = new g2o::EdgeSE2();
                edge->vertices()[0] = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(vertex_id - 1));
                edge->vertices()[1] = vi;
        
                // Get previous pose vertex
                g2o::VertexSE2* v_prev = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(vertex_id - 1));
                g2o::SE2 prev_pose = v_prev->estimate();
                
                // Calculate relative transformation (from previous pose to current pose)
                // This is what the edge should represent
                g2o::SE2 current_pose(current_x, current_y, current_yaw);
                g2o::SE2 relative_pose = prev_pose.inverse() * current_pose;
                
                // Set the measurement as the relative transformation
                edge->setMeasurement(relative_pose);
                
                std::cout << "[DEBUG] Edge measurement (relative pose): dx = " << relative_pose.translation().x()
                        << ", dy = " << relative_pose.translation().y()
                        << ", dtheta = " << relative_pose.rotation().angle() << std::endl;
        
                // Set information matrix (confidence levels)
                Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
                
                // If we're using GICP result, set higher confidence
                if (gicpConverged) {
                    information(0,0) = information_matrix_entries(0); // x confidence
                    information(1,1) = information_matrix_entries(1); // y confidence
                    information(2,2) = information_matrix_entries(2); // rotation confidence
                } else {
                    // Lower confidence for odometry-only measurements
                    information(0,0) = information_matrix_entries(0) * 0.5;
                    information(1,1) = information_matrix_entries(1) * 0.5;
                    information(2,2) = information_matrix_entries(2) * 0.5;
                }
                
                edge->setInformation(information);
                //std::cout << "[DEBUG] Set edge information matrix:\n" << information << std::endl;
        
                optimizer.addEdge(edge);
                //std::cout << "[DEBUG] Added edge between vertex " << vertex_id - 1 << " and vertex " << vertex_id << std::endl;
            }
        
            vertex_id++;
            //std::cout << "[DEBUG] Vertex ID incremented to " << vertex_id << std::endl;
        
            // ------------------------ Perform Graph Optimization ------------------------
            // Define when to optimize (e.g., every N poses)
            if (vertex_id % optimize_every == 0) {
                //std::cout << "[DEBUG] Optimization trigger: " << vertex_id << " vertices reached." << std::endl;
                std::cout << "[DEBUG] Starting graph optimization." << std::endl;
                optimizer.initializeOptimization();
                int optimization_result = optimizer.optimize(10);
                if (optimization_result < 0) {
                    std::cerr << "[ERROR] Graph optimization failed with error code: " << optimization_result << std::endl;
                } else {
                    std::cout << "[DEBUG] Optimization complete. Iterations: " << optimization_result << std::endl;
                }
        
                // Save the optimized poses to CSV file
                for (int id = 0; id < vertex_id; ++id) {
                    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(id));
                    if (v) {
                        g2o::SE2 pose = v->estimate();
                        graphOptimizedPosesFile << timeInSeconds << "," << id << "," << -pose.translation().x() << "," 
                                                << pose.translation().y() << "," << pose.rotation().angle() << std::endl;
                    }
                }
            }
        
            // ------------------------ Transform the Target Cloud ------------------------
            //std::cout << "[DEBUG] Transforming target cloud with cumulative transformation matrix." << std::endl;
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformedTargetCloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*xyz_cloud, *transformedTargetCloud, cumulativeTransformation);
            //std::cout << "[DEBUG] Transformed target cloud has " << transformedTargetCloud->points.size() << " points." << std::endl;
        
            // Reset the accumulated cloud
            accumulatedCloud->clear();
            //std::cout << "[DEBUG] Cleared accumulatedCloud." << std::endl;
        
            // Reset the lastSaveTime
            lastSaveTime = currentTime;
            //std::cout << "[DEBUG] Reset lastSaveTime to current time." << std::endl;
        }
        
        // ------------------------ Close CSV Files ------------------------
        // Close CSV files
        gicpOdomFile.close();
        kalmanFilterFile.close();
        graphOptimizedPosesFile.close();
        
        // ------------------------ Cleanup ------------------------
        //std::cout << "[DEBUG] Exiting main loop. Cleaning up." << std::endl;
        setNonBlockingInput(false);
        std::cout << "[DEBUG] Terminal settings restored. Program exiting." << std::endl;
        
        return 0;
