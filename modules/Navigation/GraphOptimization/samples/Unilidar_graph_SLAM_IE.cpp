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
#include "Laser/Point3D.hpp"
#include "VisionUtility/PointCloud/Gradient.hpp" 

#include <sys/select.h>

// g2o headers
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/types_slam2d.h>





#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/types_slam2d.h>

// Initialize the Robot
using crf::actuators::robotbase::SPSRobot;
using crf::math::kalmanfilter::KalmanFilter;
using crf::math::kalmanfilter::StateSpace;
using crf::math::kalmanfilter::SPSSystemModel;
using crf::math::kalmanfilter::SPSObservationModel;
//using crf::utility::visionutility::pointcloud::pointtypes;


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

namespace po = boost::program_options;
bool ctrl_c_pressed = false;

// Signal handler for Ctrl+C
void ctrlc(int) {
    std::cout << "\n[DEBUG] Ctrl+C pressed. Preparing to exit..." << std::endl;
    ctrl_c_pressed = true;
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
        ("unitree_config", po::value<std::string>(), "UnitreeL1 configuration file");

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

    std::string ethercat_port = vm["ethercat_port"].as<std::string>();
    std::string sps_config_path = vm["sps_config"].as<std::string>();
    std::string device = vm["device"].as<std::string>();
    std::string unitree_config_path = vm["unitree_config"].as<std::string>();

    std::cout << "[DEBUG] Parsed Arguments:" << std::endl;
    std::cout << "  EtherCAT Port: " << ethercat_port << std::endl;
    std::cout << "  SPS Config Path: " << sps_config_path << std::endl;
    std::cout << "  Device: " << device << std::endl;
    std::cout << "  Unitree Config Path: " << unitree_config_path << std::endl;

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

    // ------------------------ Initialize UnitreeL1 IMU ------------------------
    std::cout << "[DEBUG] Initializing UnitreeL1 IMU." << std::endl;
    // Corrected the connection type enumeration from LaserConnectionType to IMUConnectionType
    std::unique_ptr<crf::sensors::imu::UnitreeL1IMU> imu = std::make_unique<crf::sensors::imu::UnitreeL1IMU>(
        crf::sensors::imu::LaserConnectionType::Serial, device, 2000000); // Corrected enumeration

    if (!imu->initialize()) {
        std::cerr << "[ERROR] Could not initialize the UnitreeL1 IMU." << std::endl;
        return -1;
    }
    std::cout << "[DEBUG] UnitreeL1 IMU initialized successfully." << std::endl;

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
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> transformedClouds;

    pcl::PointCloud<Point3D>::Ptr previousCloud(new pcl::PointCloud<Point3D>());

    // Adjusted noise levels
    Eigen::MatrixXd system_noise = Eigen::MatrixXd::Identity(3, 3) * 0.01; // Increased dimension for control inputs
    Eigen::MatrixXd observation_noise = Eigen::MatrixXd::Identity(3, 3) * 0.1; // More noise to LiDAR registration

    // Initial state estimate
    Eigen::VectorXd initial_mean = Eigen::VectorXd::Zero(3); // [x, y, yaw]
    Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(3, 3) * 0.1;

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
    float base_speed = 0.2;

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
    const std::chrono::milliseconds accumulationInterval(100); // Adjust as needed
    auto previousTime = std::chrono::steady_clock::now();

    Eigen::Matrix4f odomTransformation = Eigen::Matrix4f::Identity();
    static float prev_a_x = 0.0f, prev_a_y = 0.0f, prev_gyro_z = 0.0f;
    float a_x = 0.0f, a_y = 0.0f, gyro_z = 0.0f;
    float alpha = 0.5f; 

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
        const float movement_threshold = 1e-1; // Adjust as appropriate
        if (std::abs(x_l) < movement_threshold && std::abs(y_l) < movement_threshold && std::abs(w_l) < movement_threshold) {
            //std::cout << "[DEBUG] Robot is stationary. Skipping scan matching and IMU reading." << std::endl;
            continue;
        } else {
            //std::cout << "[DEBUG] Robot is moving. Proceeding with sensor data processing." << std::endl;
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
            pcl::PointCloud<Point3D>::Ptr xyz_cloud(new pcl::PointCloud<Point3D>());
            for (const auto& point : accumulatedCloud->points) {
                Point3D xyz_point;
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
            //pcl::PointCloud<Point3D>::Ptr cloud_filtered_source(new pcl::PointCloud<Point3D>());
            //pcl::PointCloud<Point3D>::Ptr cloud_filtered_target(new pcl::PointCloud<Point3D>());
            //pcl::PassThrough<pcl::PointXYZ> pass;
            //pass.setInputCloud(xyz_cloud);
            //pass.setFilterFieldName("z");
            //pass.setFilterLimits(0, 25.0); 
            //pass.filter(*cloud_filtered_source);
            ////std::cout << "[DEBUG] Applied passthrough filter. Source cloud now has " << cloud_filtered_source->points.size() << " points." << std::endl;

            //pass.setInputCloud(previousCloud);
            //pass.filter(*cloud_filtered_target);
            //std::cout << "[DEBUG] Applied passthrough filter. Target cloud now has " << cloud_filtered_target->points.size() << " points." << std::endl;


            pcl::PointCloud<Point3D>::Ptr sourceCloud(new pcl::PointCloud<Point3D>());
            pcl::PointCloud<Point3D>::Ptr targetCloud(new pcl::PointCloud<Point3D>());

            // Populate sourceCloud and targetCloud with your data
            // For example, you can load PCD files or create synthetic data

            // Parameters
            unsigned int kNeighbour = 20; // Number of neighbors for sharpness computation
            float mPercent = 10.0f;       // Top M% to select

            // Compute sharpness for both clouds
            crf::utility::visionutility::pointcloud::gradient::computeSharpness(previousCloud, kNeighbour);
            crf::utility::visionutility::pointcloud::gradient::computeSharpness(xyz_cloud, kNeighbour);

            // Select top M% points based on intensity and sharpness
            auto selectedSource = crf::utility::visionutility::pointcloud::gradient::selectTopMPercentPoints(sourceCloud, mPercent);
            auto selectedTarget = crf::utility::visionutility::pointcloud::gradient::selectTopMPercentPoints(targetCloud, mPercent);

            // Find transformation between the selected points
            Eigen::Matrix4f keytransformation = crf::utility::visionutility::pointcloud::gradient::findTransformation(selectedSource, selectedTarget);

            // Output the transformation matrix
            std::cout << "Key detection Transformation Matrix:\n" << keytransformation << std::endl;

            Eigen::Matrix4f transformationMatrix;
            auto startTime = std::chrono::high_resolution_clock::now();

            // Use 'template' keyword to disambiguate the function template
            //std::cout << "[DEBUG] Starting GICP alignment." << std::endl;
            bool gicpConverged = crf::utility::visionutility::pointcloud::gicp::template gicp<Point3D>(
                xyz_cloud, previousCloud, transformationMatrix);
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

                cumulativeTransformation = cumulativeTransformation * odomTransformation;
                std::cout << "[DEBUG] Odometry Translation: dx = " << dx
                          << ", dy = " << dy
                          << ", dtheta = " << dtheta << std::endl;
                nonConvergenceCount++;
            } else {
                // Process the transformation as before
                //std::cout << "[DEBUG] GICP converged. Processing transformation matrix." << std::endl;
                Eigen::Matrix3f rotationMatrix = transformationMatrix.block<3,3>(0,0);
                Eigen::Vector3f translationVector = transformationMatrix.block<3,1>(0,3);
                yaw = atan2(rotationMatrix(1,0), rotationMatrix(0,0));
                //std::cout << "[DEBUG] Extracted yaw from rotation matrix: " << yaw << std::endl;

                std::cout << "[DEBUG] Gicp Translation: x = " << translationVector[0]
                          << ", y = " << translationVector[1]
                          << ", yaw = " << yaw << std::endl;

                Eigen::Matrix3f newRotationMatrix;
                newRotationMatrix << cos(yaw), -sin(yaw), 0.0f,
                                    sin(yaw),  cos(yaw), 0.0f,
                                        0.0f,      0.0f, 1.0f;
                //std::cout << "[DEBUG] New rotation matrix:\n" << newRotationMatrix << std::endl;

                translationVector[2] = 0.0f;
                Eigen::Matrix4f modifiedTransformationMatrix = Eigen::Matrix4f::Identity();
                modifiedTransformationMatrix.block<3,3>(0,0) = newRotationMatrix;
                modifiedTransformationMatrix.block<3,1>(0,3) = translationVector;
                transformationMatrix = modifiedTransformationMatrix;

                cumulativeTransformation = cumulativeTransformation * transformationMatrix.inverse();
                //std::cout << "[DEBUG] Updated cumulative transformation matrix:\n" << cumulativeTransformation << std::endl;
            }

            // Update the previous cloud for the next iteration
            *previousCloud = *xyz_cloud;  // Save the current scan as the previous scan
            //std::cout << "[DEBUG] Updated previousCloud with current xyz_cloud." << std::endl;

            comparisonCount++;
            //std::cout << "[DEBUG] Total comparisons: " << comparisonCount << ", Non-convergences: " << nonConvergenceCount << std::endl;

            // ------------------------ Kalman Filter Prediction and Correction ------------------------
            //std::cout << "[DEBUG] Retrieving IMU data for Kalman Filter." << std::endl;
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
                //std::cout << "[DEBUG] IMU Linear Acceleration: raw_a_x = " << raw_a_x << ", raw_a_y = " << raw_a_y
                //          << " | Filtered: a_x = " << a_x << ", a_y = " << a_y << std::endl;
            }
            if (imuSignals.angularVelocity) {
                float raw_gyro_z = imuSignals.angularVelocity.value()[2];
                // Apply low-pass filter
                gyro_z = alpha * raw_gyro_z + (1 - alpha) * prev_gyro_z;
                prev_gyro_z = gyro_z;
                //std::cout << "[DEBUG] IMU Angular Velocity: raw_gyro_z = " << raw_gyro_z
                //          << " | Filtered: gyro_z = " << gyro_z << std::endl;
            }

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
            //kf.correction(observation_model, state_space);
            //std::cout << "[DEBUG] Correction step completed." << std::endl;

            Eigen::VectorXd meanf = state_space->getMean();
            //Eigen::MatrixXd covf = state_space->getCovariance();

            //std::cout << "[DEBUG] Kalman Filter State Mean:\n" << meanf << std::endl;
            //std::cout << "[DEBUG] Kalman Filter State Covariance:\n" << covf << std::endl;

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

                // Set the measurement as the relative transformation
                if (!gicpConverged){
                    transformationMatrix = odomTransformation;
                }

                float x_translation = transformationMatrix(0, 3);
                float y_translation = transformationMatrix(1, 3);
                float psi = atan2(transformationMatrix(1,0), transformationMatrix(0,0));
                g2o::SE2 relative_pose(x_translation, y_translation, yaw);
                edge->setMeasurement(relative_pose);
                //std::cout << "[DEBUG] Set edge measurement (relative_pose): " << relative_pose.translation().transpose()
                 //         << ", rotation (radians) = " << relative_pose.rotation().angle() << std::endl;

                // Set information matrix (higher confidence in odometry)
                Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
                information(0,0) = 100; // High confidence in x
                information(1,1) = 100; // High confidence in y
                information(2,2) = 100; // High confidence in yaw
                edge->setInformation(information);
                //std::cout << "[DEBUG] Set edge information matrix:\n" << information << std::endl;

                optimizer.addEdge(edge);
                ///std::cout << "[DEBUG] Added edge between vertex " << vertex_id - 1 << " and vertex " << vertex_id << std::endl;
            }

            vertex_id++;
            //std::cout << "[DEBUG] Vertex ID incremented to " << vertex_id << std::endl;

            // ------------------------ Perform Graph Optimization ------------------------
            // Define when to optimize (e.g., every 10 poses)
            const int optimize_every = 10;
            if (vertex_id % optimize_every == 0) {
                //std::cout << "[DEBUG] Optimization trigger: " << vertex_id << " vertices reached." << std::endl;
                std::cout << "[DEBUG] Starting graph optimization." << std::endl;
                optimizer.initializeOptimization();
                int optimization_result = optimizer.optimize(10);
                if (optimization_result < 0) {
                   // std::cerr << "[ERROR] Graph optimization failed with error code: " << optimization_result << std::endl;
                } else {
                    //std::cout << "[DEBUG] Optimization complete. Iterations: " << optimization_result << std::endl;
                }

                // Optionally, you can extract and use the optimized poses here
                // For demonstration, we'll print the latest optimized pose
                g2o::VertexSE2* last_vertex = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(vertex_id - 1));
                if (last_vertex) {
                    g2o::SE2 optimized_pose = last_vertex->estimate();
                    std::cout << "[DEBUG] G2o Optimized Pose " << vertex_id - 1 << ": x = " << optimized_pose.translation().x()
                              << ", y = " << optimized_pose.translation().y()
                              << ", yaw = " << optimized_pose.rotation().angle() << std::endl;
                }
            }

            // ------------------------ Transform the Target Cloud ------------------------
            //std::cout << "[DEBUG] Transforming target cloud with cumulative transformation matrix." << std::endl;
            pcl::PointCloud<Point3D>::Ptr transformedTargetCloud(new pcl::PointCloud<Point3D>());
            pcl::transformPointCloud(*xyz_cloud, *transformedTargetCloud, cumulativeTransformation);
            //std::cout << "[DEBUG] Transformed target cloud has " << transformedTargetCloud->points.size() << " points." << std::endl;

            // Reset the accumulated cloud
            accumulatedCloud->clear();
            //std::cout << "[DEBUG] Cleared accumulatedCloud." << std::endl;

            // Reset the lastSaveTime
            lastSaveTime = currentTime;
            //std::cout << "[DEBUG] Reset lastSaveTime to current time." << std::endl;
        }

    // ------------------------ Cleanup ------------------------
    //std::cout << "[DEBUG] Exiting main loop. Cleaning up." << std::endl;
    //setNonBlockingInput(false);
    std::cout << "[DEBUG] Terminal settings restored. Program exiting." << std::endl;
    
    }

    return 0;

}
