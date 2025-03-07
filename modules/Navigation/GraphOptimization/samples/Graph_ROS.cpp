#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <vector>

// Global constants and variables
const std::string BAG_FILE = "path_to_your_rosbag.bag";
const std::string POINT_CLOUD_TOPIC = "/lidar_topic";
const std::string IMU_TOPIC = "/imu_topic";

// Kalman Filter and Optimization Parameters
Eigen::MatrixXd system_noise(3, 3);
Eigen::MatrixXd observation_noise(3, 3);
Eigen::VectorXd initial_mean(3);
Eigen::MatrixXd initial_covariance(3, 3);
Eigen::Vector3d information_matrix_entries;

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "rosbag_data_reader");
    ros::NodeHandle nh;

    // Open the bag file
    rosbag::Bag bag;
    try {
        bag.open(BAG_FILE, rosbag::bagmode::Read);
    } catch (const rosbag::BagException& e) {
        std::cerr << "[ERROR] Failed to open rosbag: " << e.what() << std::endl;
        return -1;
    }

    std::cout << "[DEBUG] Rosbag opened successfully." << std::endl;

    // Define topics to read from
    std::vector<std::string> topics;
    topics.push_back(POINT_CLOUD_TOPIC);
    topics.push_back(IMU_TOPIC);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Output files
    std::ofstream imu_data_file("imu_data.csv");
    std::ofstream point_cloud_data_file("point_cloud_data.csv");
    std::ofstream kalman_filter_file("kalman_filter_results.csv");
    std::ofstream graph_optimization_file("graph_optimization_results.csv");

    if (!imu_data_file.is_open() || !point_cloud_data_file.is_open() ||
        !kalman_filter_file.is_open() || !graph_optimization_file.is_open()) {
        std::cerr << "[ERROR] Failed to open output files." << std::endl;
        return -1;
    }

    // Write headers to CSV files
    imu_data_file << "time,linear_acceleration_x,linear_acceleration_y,linear_acceleration_z,angular_velocity_x,angular_velocity_y,angular_velocity_z" << std::endl;
    point_cloud_data_file << "time,x,y,z,intensity" << std::endl;
    kalman_filter_file << "time,x,y,yaw" << std::endl;
    graph_optimization_file << "time,vertex_id,x,y,yaw" << std::endl;

    // Initialize Kalman Filter and Graph Optimization
    Eigen::VectorXd state = initial_mean;
    Eigen::MatrixXd covariance = initial_covariance;

    g2o::SparseOptimizer optimizer;
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    std::unique_ptr<LinearSolverType> linearSolver = std::make_unique<LinearSolverType>();
    std::unique_ptr<BlockSolverType> solver = std::make_unique<BlockSolverType>(std::move(linearSolver));
    optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(std::move(solver)));

    int vertex_id = 0;
    g2o::VertexSE2* v0 = new g2o::VertexSE2();
    v0->setId(vertex_id);
    v0->setEstimate(g2o::SE2(0.0, 0.0, 0.0));
    v0->setFixed(true);
    optimizer.addVertex(v0);
    vertex_id++;

    // Iterate through bag messages
    for (const rosbag::MessageInstance& m : view) {
        if (m.getTopic() == POINT_CLOUD_TOPIC || ("/" + m.getTopic()) == POINT_CLOUD_TOPIC) {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_msg) {
                pcl::PointCloud<pcl::PointXYZI> cloud;
                pcl::fromROSMsg(*cloud_msg, cloud);
                double timestamp = cloud_msg->header.stamp.toSec();

                for (const auto& point : cloud) {
                    point_cloud_data_file << timestamp << "," << point.x << "," << point.y << "," << point.z << "," << point.intensity << std::endl;
                }

                // Process point cloud (placeholder for SLAM integration)
                std::cout << "[DEBUG] Processed point cloud at time: " << timestamp << ", points: " << cloud.size() << std::endl;
            }
        } else if (m.getTopic() == IMU_TOPIC || ("/" + m.getTopic()) == IMU_TOPIC) {
            sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
            if (imu_msg) {
                double timestamp = imu_msg->header.stamp.toSec();
                imu_data_file << timestamp << ","
                              << imu_msg->linear_acceleration.x << ","
                              << imu_msg->linear_acceleration.y << ","
                              << imu_msg->linear_acceleration.z << ","
                              << imu_msg->angular_velocity.x << ","
                              << imu_msg->angular_velocity.y << ","
                              << imu_msg->angular_velocity.z << std::endl;

                // Kalman Filter Prediction and Correction (simplified example)
                Eigen::VectorXd control_input(3);
                control_input << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->angular_velocity.z;
                state = state + control_input; // Replace with actual KF equations

                kalman_filter_file << timestamp << "," << state[0] << "," << state[1] << "," << state[2] << std::endl;

                // Graph Optimization (example)
                g2o::VertexSE2* vi = new g2o::VertexSE2();
                vi->setId(vertex_id);
                vi->setEstimate(g2o::SE2(state[0], state[1], state[2]));
                optimizer.addVertex(vi);

                if (vertex_id > 0) {
                    g2o::EdgeSE2* edge = new g2o::EdgeSE2();
                    edge->vertices()[0] = optimizer.vertex(vertex_id - 1);
                    edge->vertices()[1] = vi;

                    Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
                    information(0, 0) = information_matrix_entries[0];
                    information(1, 1) = information_matrix_entries[1];
                    information(2, 2) = information_matrix_entries[2];
                    edge->setInformation(information);

                    optimizer.addEdge(edge);
                }

                vertex_id++;
            }
        }
    }

    // Perform graph optimization
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // Save optimized poses
    for (int id = 0; id < vertex_id; ++id) {
        g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(id));
        if (v) {
            g2o::SE2 pose = v->estimate();
            graph_optimization_file << id << "," << pose.translation()[0] << "," << pose.translation()[1] << "," << pose.rotation().angle() << std::endl;
        }
    }

    // Close files and bag
    imu_data_file.close();
    point_cloud_data_file.close();
    kalman_filter_file.close();
    graph_optimization_file.close();
    bag.close();

    std::cout << "[DEBUG] Processing complete. Files saved." << std::endl;

    return 0;
}
