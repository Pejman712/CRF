/*
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <random>
#include <vector>
#include <string>
#include <unordered_map>
#include <chrono>
#include <signal.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h> // For removing NaN values
#include <pcl/common/transforms.h> // Updated include
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h> // Updated include for PCL 1.14

#include <filesystem> // Include filesystem for directory traversal
namespace fs = std::filesystem;

// Define the point type with sharpness
struct PointXYZS {
    PCL_ADD_POINT4D;  // Quad-word XYZ
    float s;          // Sharpness
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Register the point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZS,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, s, s)
)

bool ctrl_c_pressed = false;

void ctrlc(int) {
    ctrl_c_pressed = true;
}

// Function to compute sharpness (using curvature as a proxy)
pcl::PointCloud<PointXYZS>::Ptr computeEigenSharpness(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int kNeighbour) {

    // Create a KdTree for neighborhood search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // Output cloud with sharpness
    pcl::PointCloud<PointXYZS>::Ptr sharpness_cloud(new pcl::PointCloud<PointXYZS>());

    // Compute normals and curvature
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(kNeighbour);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    // Combine XYZ and curvature into PointXYZS
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        PointXYZS pt;
        pt.x = cloud->points[i].x;
        pt.y = cloud->points[i].y;
        pt.z = cloud->points[i].z;
        pt.s = normals->points[i].curvature; // Use curvature as sharpness
        sharpness_cloud->points.push_back(pt);
    }
    sharpness_cloud->width = sharpness_cloud->points.size();
    sharpness_cloud->height = 1;
    sharpness_cloud->is_dense = true;

    return sharpness_cloud;
}

pcl::PointCloud<PointXYZS>::Ptr filterTopSharpness(
    pcl::PointCloud<PointXYZS>::Ptr sharpness_cloud, float percent) {

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
    if (top_percent_index >= sharpness_values.size()) {
        top_percent_index = sharpness_values.size() - 1;
    }
    float sharpness_threshold = sharpness_values[top_percent_index];

    // Filter points that have sharpness values greater than or equal to the threshold
    pcl::PointCloud<PointXYZS>::Ptr filtered_cloud(new pcl::PointCloud<PointXYZS>());

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

long extractNumber(const std::string& filename) {
    std::string::size_type start = filename.find_last_of('_') + 1;
    std::string::size_type end = filename.find_last_of('.');
    std::string numberStr = filename.substr(start, end - start);
    return std::stol(numberStr);
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <directory_path>" << std::endl;
        return -1;
    }

    std::string directoryPath = argv[1];
    std::vector<std::string> pcdFiles;

    // Read all PCD files in the directory
    for (const auto& entry : fs::directory_iterator(directoryPath)) {
        if (entry.path().extension() == ".pcd") {
            pcdFiles.push_back(entry.path().string());
        }
    }

    // Sort the files based on the numeric part of the filename
    std::sort(pcdFiles.begin(), pcdFiles.end(), [](const std::string& a, const std::string& b) {
        return extractNumber(a) < extractNumber(b);
    });

    if (pcdFiles.size() < 2) {
        std::cerr << "Not enough point clouds for comparison." << std::endl;
        return -1;
    }

    // Setup Ctrl+C handler
    signal(SIGINT, ctrlc);

    int comparisonCount = 0;
    int nonConvergenceCount = 0;
    Eigen::Matrix4f cumulativeTransformation = Eigen::Matrix4f::Identity();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scans;
    std::vector<Eigen::Matrix4f> robotPositions;

    // Variables to keep track of landmark IDs
    int landmark_id_counter = 1000; // Starting ID for landmarks

    unsigned int kNeighbour = 20;
    float percent = 90.0f;  // Percentage for sharpness filtering

    // Prepare the optimizer (outside the loop)
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);

    // Choose the linear solver type
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;

    // Create the linear solver
    std::unique_ptr<LinearSolverType> linearSolver(new LinearSolverType());

    // Create the block solver
    std::unique_ptr<BlockSolverType> blockSolver(new BlockSolverType(std::move(linearSolver)));

    // Create the optimization algorithm
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

    // Set the algorithm to the optimizer
    optimizer.setAlgorithm(algorithm);

    // Initialize the first pose vertex in the optimizer
    {
        g2o::VertexSE3* v_se3 = new g2o::VertexSE3();
        v_se3->setId(0);
        Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
        v_se3->setEstimate(initial_pose);
        v_se3->setFixed(true); // Fix the first pose to anchor the graph
        optimizer.addVertex(v_se3);
    }

    for (size_t idx = 0; idx < pcdFiles.size(); ++idx) {
        if (ctrl_c_pressed) {
            std::cout << "Ctrl+C pressed. Exiting..." << std::endl;
            break;
        }

        std::string currentFile = pcdFiles[idx];
        std::cout << "Processing file: " << currentFile << std::endl;

        // Load the current point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(currentFile, *currentCloud) == -1) {
            std::cerr << "Couldn't read file " << currentFile << std::endl;
            continue;
        }

        // Remove NaN values from the cloud
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*currentCloud, *currentCloud, indices);

        // Skip the comparison if this is the first scan
        if (idx == 0) {
            scans.push_back(currentCloud);
            robotPositions.push_back(cumulativeTransformation);
            continue;
        }

        // Load the previous point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr previousCloud = scans.back();

        // Filter the clouds based on the height
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_source(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_target(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 5.0);

        pass.setInputCloud(previousCloud);
        pass.filter(*cloud_filtered_source);

        pass.setInputCloud(currentCloud);
        pass.filter(*cloud_filtered_target);

        Eigen::Matrix4f transformationMatrix;
        auto startTime = std::chrono::high_resolution_clock::now();

        // Use PCL's GICP implementation
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputSource(cloud_filtered_source);
        gicp.setInputTarget(cloud_filtered_target);
        pcl::PointCloud<pcl::PointXYZ> Final;
        gicp.align(Final);
        bool gicpConverged = gicp.hasConverged();
        transformationMatrix = gicp.getFinalTransformation();

        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> processingTime = endTime - startTime;

        if (!gicpConverged) {
            std::cerr << "GICP did not converge for the current scan comparison." << std::endl;
            nonConvergenceCount++;
            scans.push_back(currentCloud);
            robotPositions.push_back(cumulativeTransformation); // Assume no change
            continue;
        } else {
            // Process the transformation
            // Modify the transformation as per your requirement
            transformationMatrix(2, 3) = 0.0f; // Set Z translation to zero
            transformationMatrix.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity(); // Ignore rotation

            // Accumulate the transformation matrix
            cumulativeTransformation = cumulativeTransformation * transformationMatrix.inverse();

            // Update robot positions
            robotPositions.push_back(cumulativeTransformation);
        }

        comparisonCount++;

        scans.push_back(currentCloud); // Store the current scan

        int N = 10; // Adjust N as needed
        if ((idx + 1) % N == 0 && idx != 0) {
            std::cout << "Performing graph optimization for the last " << N << " scans." << std::endl;

            // Variables to store all landmarks and their associations
            pcl::PointCloud<pcl::PointXYZ>::Ptr all_landmarks(new pcl::PointCloud<pcl::PointXYZ>());
            std::unordered_map<int, std::vector<int>> poseToLandmarks; // Map from pose index to indices of landmarks in all_landmarks
            std::vector<int> pointToClusterIndex; // Map from point index in all_landmarks to cluster index

            // Step 1: For each pose in the last N scans, perform edge detection and store landmarks
            for (size_t i = idx - N + 1; i <= idx; ++i) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scan(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::transformPointCloud(*scans[i], *transformed_scan, robotPositions[i]);

                // Perform edge detection on transformed_scan
                auto sharpness_cloud = computeEigenSharpness(transformed_scan, kNeighbour);

                if (!sharpness_cloud || sharpness_cloud->empty()) {
                    std::cerr << "No sharpness data computed for scan " << i << "!" << std::endl;
                    continue;
                }

                // Filter the sharpness to show only the top N%
                auto filtered_sharpness_cloud = filterTopSharpness(sharpness_cloud, percent);

                // Map from pose index to indices of landmarks in all_landmarks
                std::vector<int> landmarkIndices;
                for (size_t j = 0; j < filtered_sharpness_cloud->points.size(); ++j) {
                    all_landmarks->points.push_back(pcl::PointXYZ(filtered_sharpness_cloud->points[j].x,
                                                                 filtered_sharpness_cloud->points[j].y,
                                                                 filtered_sharpness_cloud->points[j].z));
                    landmarkIndices.push_back(all_landmarks->points.size() - 1); // Index in all_landmarks
                }

                // Store the mapping from this pose to its landmarks
                poseToLandmarks[static_cast<int>(i)] = landmarkIndices;
            }

            if (all_landmarks->empty()) {
                std::cerr << "No landmarks detected in the last " << N << " scans." << std::endl;
                continue;
            }

            // Now, cluster all_landmarks to find consistent landmarks across scans
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            tree->setInputCloud(all_landmarks);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.1); // Adjust as necessary
            ec.setMinClusterSize(1);
            ec.setMaxClusterSize(25000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(all_landmarks);
            ec.extract(cluster_indices);

            pointToClusterIndex.resize(all_landmarks->points.size(), -1);

            int cluster_id = 0;
            for (const auto& cluster : cluster_indices) {
                for (const auto& index : cluster.indices) {
                    pointToClusterIndex[index] = cluster_id;
                }
                cluster_id++;
            }

            // Step 3: Add robot pose vertices for the last N scans
            for (size_t i = idx - N + 1; i <= idx; ++i) {
                if (optimizer.vertex(static_cast<int>(i)) == nullptr) {
                    g2o::VertexSE3* v_se3 = new g2o::VertexSE3();
                    v_se3->setId(static_cast<int>(i));
                    // Convert Eigen::Matrix4f to Eigen::Isometry3d
                    Eigen::Matrix4f pose_matrix = robotPositions[i];
                    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
                    pose.matrix() = pose_matrix.cast<double>();
                    v_se3->setEstimate(pose);
                    optimizer.addVertex(v_se3);
                }
            }

            // Step 4: Add landmark vertices
            // We need to create a mapping from cluster IDs to landmark vertices
            std::unordered_map<int, g2o::VertexPointXYZ*> landmarkVertices;
            for (int cid = 0; cid < cluster_id; ++cid) {
                // Compute the centroid of the cluster to initialize the landmark position
                pcl::CentroidPoint<pcl::PointXYZ> centroid;
                for (const auto& point_idx : cluster_indices[cid].indices) {
                    centroid.add(all_landmarks->points[point_idx]);
                }
                pcl::PointXYZ centroid_point;
                centroid.get(centroid_point);

                // Create the landmark vertex
                g2o::VertexPointXYZ* landmark = new g2o::VertexPointXYZ();
                landmark->setId(landmark_id_counter++);
                Eigen::Vector3d position(centroid_point.x, centroid_point.y, centroid_point.z);
                landmark->setEstimate(position);
                optimizer.addVertex(landmark);

                // Store in the mapping
                landmarkVertices[cid] = landmark;
            }

            // Step 5: Add odometry edges between robot poses
            for (size_t i = idx - N + 1; i < idx; ++i) {
                // Proceed to add the edge directly
                g2o::EdgeSE3* edge = new g2o::EdgeSE3();
                edge->setVertex(0, optimizer.vertex(static_cast<int>(i)));
                edge->setVertex(1, optimizer.vertex(static_cast<int>(i + 1)));

                // Compute the relative transformation between the poses
                Eigen::Isometry3d pose_i = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(static_cast<int>(i)))->estimate();
                Eigen::Isometry3d pose_j = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(static_cast<int>(i + 1)))->estimate();
                Eigen::Isometry3d relative_pose = pose_i.inverse() * pose_j;

                edge->setMeasurement(relative_pose);

                // Set information matrix (inverse of covariance)
                Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
                information.block<3, 3>(0, 0) *= 100; // High confidence in rotation
                information.block<3, 3>(3, 3) *= 100; // High confidence in translation
                edge->setInformation(information);

                optimizer.addEdge(edge);
            }

            // Step 6: Define and add the sensor pose parameter
            g2o::ParameterSE3Offset* sensorPose = new g2o::ParameterSE3Offset();
            sensorPose->setId(10000); // Assign a unique ID to avoid conflicts
            sensorPose->setOffset(Eigen::Isometry3d::Identity()); // Assuming sensor frame == robot frame
            optimizer.addParameter(sensorPose);

            // Step 7: Add landmark observation edges
            for (size_t i = idx - N + 1; i <= idx; ++i) {
                auto& landmarkIndices = poseToLandmarks[static_cast<int>(i)];
                for (const auto& point_idx : landmarkIndices) {
                    int cluster_id = pointToClusterIndex[point_idx];
                    if (cluster_id == -1) {
                        continue; // Invalid cluster id
                    }
                    // Get the landmark vertex
                    g2o::VertexPointXYZ* landmark = landmarkVertices[cluster_id];
                    // Get the robot pose vertex
                    g2o::VertexSE3* pose_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(static_cast<int>(i)));

                    // Create the edge
                    g2o::EdgeSE3PointXYZ* edge = new g2o::EdgeSE3PointXYZ();
                    edge->setVertex(0, pose_vertex);
                    edge->setVertex(1, landmark);

                    // Compute the measurement: the position of the landmark in the robot's frame
                    Eigen::Vector3d landmark_position(all_landmarks->points[point_idx].x,
                                                      all_landmarks->points[point_idx].y,
                                                      all_landmarks->points[point_idx].z);
                    Eigen::Isometry3d pose_estimate = pose_vertex->estimate();
                    Eigen::Vector3d measurement = pose_estimate.inverse() * landmark_position;

                    edge->setMeasurement(measurement);

                    // Information matrix
                    Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * 100;
                    edge->setInformation(information);

                    // Assign the sensor pose parameter to the edge
                    edge->setParameterId(0, sensorPose->id());

                    optimizer.addEdge(edge);
                }
            }

            // Step 8: Optimize the graph
            optimizer.initializeOptimization();
            optimizer.optimize(10);

            // Step 9: Update robot positions and landmarks with optimized values
            for (size_t i = idx - N + 1; i <= idx; ++i) {
                g2o::VertexSE3* pose_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(static_cast<int>(i)));
                Eigen::Isometry3d optimized_pose = pose_vertex->estimate();
                // Update robotPositions
                robotPositions[i] = optimized_pose.matrix().cast<float>();
            }

            // Optionally, update the landmarks as well
            for (const auto& kv : landmarkVertices) {
                int cluster_id = kv.first;
                g2o::VertexPointXYZ* landmark_vertex = kv.second;
                Eigen::Vector3d optimized_position = landmark_vertex->estimate();
                // Update or output the optimized landmark positions as needed
                std::cout << "Optimized landmark " << cluster_id << ": " << optimized_position.transpose() << std::endl;
            }
        }
    }

    std::cout << "Processing completed." << std::endl;
    return 0;
}
*/
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
// Switched to Dense Solver for debugging purposes
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <chrono>
#include <signal.h>
#include <g2o/core/robust_kernel_impl.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h> // For removing NaN values
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h> // Include PCL visualizer

#include <filesystem> // Include filesystem for directory traversal
namespace fs = std::filesystem;

// Include threading header for sleep function
#include <thread>

// Include your GICP function header (assuming it's available)
// Replace with appropriate header if necessary
#include "VisionUtility/PointCloud/Gicp.hpp"

bool ctrl_c_pressed = false;

void ctrlc(int) {
    ctrl_c_pressed = true;
}

long extractNumber(const std::string& filename) {
    std::string::size_type start = filename.find_last_of('_');
    std::string::size_type end = filename.find_last_of('.');

    if (start == std::string::npos || end == std::string::npos || start >= end) {
        std::cerr << "Invalid filename format: " << filename << std::endl;
        return 0; // Or handle the error as appropriate
    }

    start += 1; // Move past the underscore
    std::string numberStr = filename.substr(start, end - start);

    try {
        return std::stol(numberStr);
    } catch (const std::exception& e) {
        std::cerr << "Error parsing number from filename '" << filename << "': " << e.what() << std::endl;
        return 0; // Or handle the error as appropriate
    }
}

// Function to check if a matrix contains finite values
bool isMatrixValid(const Eigen::Matrix4d& mat) {
    return mat.allFinite();
}

// Structure to hold edge data
struct EdgeData {
    int vertex_id0;
    int vertex_id1;
    Eigen::Isometry3d measurement;
};

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <directory_path>" << std::endl;
        return -1;
    }

    std::string directoryPath = argv[1];
    std::vector<std::string> pcdFiles;

    // Read all PCD files in the directory
    for (const auto& entry : fs::directory_iterator(directoryPath)) {
        if (entry.path().extension() == ".pcd") {
            pcdFiles.push_back(entry.path().string());
        }
    }

    // Sort the files based on the numeric part of the filename
    std::sort(pcdFiles.begin(), pcdFiles.end(), [](const std::string& a, const std::string& b) {
        return extractNumber(a) < extractNumber(b);
    });

    if (pcdFiles.size() < 2) {
        std::cerr << "Not enough point clouds for comparison." << std::endl;
        return -1;
    }

    // Setup Ctrl+C handler
    signal(SIGINT, ctrlc);

    int comparisonCount = 0;
    int nonConvergenceCount = 0;
    Eigen::Matrix4d cumulativeTransformation = Eigen::Matrix4d::Identity();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scans;

    // Variables to store robot positions
    std::vector<Eigen::Matrix4d> robotPositionsGICP;        // Positions from GICP
    std::vector<Eigen::Matrix4d> robotPositionsOptimized;   // Positions after optimization

    // Vector to store relative transformations from GICP
    std::vector<EdgeData> edges;

    // Prepare the optimizer (outside the loop)
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);

    // Choose the linear solver type
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    // Switched to Dense linear solver for debugging purposes
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    // Create the linear solver
    std::unique_ptr<LinearSolverType> linearSolver(new LinearSolverType());

    // Create the block solver
    std::unique_ptr<BlockSolverType> blockSolver(new BlockSolverType(std::move(linearSolver)));

    // Create the optimization algorithm (Gauss-Newton)
    g2o::OptimizationAlgorithmGaussNewton* algorithm = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));

    // Set the algorithm to the optimizer
    optimizer.setAlgorithm(algorithm);

    // Initialize the first pose vertex in the optimizer
    {
        g2o::VertexSE3* v_se3 = new g2o::VertexSE3();
        v_se3->setId(0);
        Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
        v_se3->setEstimate(initial_pose);
        v_se3->setFixed(true); // Fix the first pose to anchor the graph
        optimizer.addVertex(v_se3);

        // Also initialize the robot positions
        robotPositionsGICP.push_back(Eigen::Matrix4d::Identity());
        robotPositionsOptimized.push_back(Eigen::Matrix4d::Identity());
    }

    int N = 10; // Number of scans to process before each optimization (adjusted for debugging purposes)

    size_t lastValidIdx = 0; // Keep track of the last valid index where GICP succeeded
    Eigen::Matrix4d lastSuccessfulTransformation = Eigen::Matrix4d::Identity(); // Initialize to identity

    for (size_t idx = 1; idx < pcdFiles.size(); ++idx) {
        if (ctrl_c_pressed) {
            std::cout << "Ctrl+C pressed. Exiting..." << std::endl;
            break;
        }

        std::string currentFile = pcdFiles[idx];
        std::cout << "Processing file: " << currentFile << std::endl;

        // Load the current point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(currentFile, *currentCloud) == -1) {
            std::cerr << "Couldn't read file " << currentFile << std::endl;
            continue;
        }

        // Remove NaN values from the cloud
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*currentCloud, *currentCloud, indices);

        // Use last valid cloud as previousCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr previousCloud;
        if (scans.empty()) {
            // Load the first scan
            std::string firstFile = pcdFiles[0];
            std::cout << "Loading first scan: " << firstFile << std::endl;
            previousCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(firstFile, *previousCloud) == -1) {
                std::cerr << "Couldn't read file " << firstFile << std::endl;
                continue;
            }
            // Remove NaN values from the cloud
            pcl::removeNaNFromPointCloud(*previousCloud, *previousCloud, indices);
            scans.push_back(previousCloud);
        } else {
            // Use last valid cloud
            previousCloud = scans[lastValidIdx];
        }

        // Filter the clouds based on the height
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_source(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_target(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 5.0);

        pass.setInputCloud(previousCloud);
        pass.filter(*cloud_filtered_source);

        pass.setInputCloud(currentCloud);
        pass.filter(*cloud_filtered_target);

        Eigen::Matrix4f transformationMatrix;
        auto startTime = std::chrono::high_resolution_clock::now();

        // Use your GICP implementation
        bool gicpConverged = crf::utility::visionutility::pointcloud::gicp::template gicp<pcl::PointXYZ>(
            cloud_filtered_source, cloud_filtered_target, transformationMatrix);

        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> processingTime = endTime - startTime;

        // Add the vertex at idx to the optimizer
        if (optimizer.vertex(static_cast<int>(idx)) == nullptr) {
            g2o::VertexSE3* v_se3 = new g2o::VertexSE3();
            v_se3->setId(static_cast<int>(idx));
            Eigen::Matrix4d pose_matrix = robotPositionsOptimized.back(); // Use last known pose
            if (!isMatrixValid(pose_matrix)) {
                std::cerr << "Invalid pose matrix at index " << idx << "." << std::endl;
                continue;
            }
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.matrix() = pose_matrix;
            v_se3->setEstimate(pose);
            optimizer.addVertex(v_se3);
        }

        if (!gicpConverged || !transformationMatrix.allFinite()) {
            std::cerr << "GICP did not converge or produced invalid transformation for scan " << idx << "." << std::endl;
            nonConvergenceCount++;

            // Update cumulativeTransformation using lastSuccessfulTransformation
            cumulativeTransformation = cumulativeTransformation * lastSuccessfulTransformation;

            // Update robot positions
            robotPositionsGICP.push_back(cumulativeTransformation);
            robotPositionsOptimized.push_back(cumulativeTransformation);

            // Add edge based on lastSuccessfulTransformation
            EdgeData edge_data;
            edge_data.vertex_id0 = static_cast<int>(lastValidIdx);
            edge_data.vertex_id1 = static_cast<int>(idx);
            edge_data.measurement = Eigen::Isometry3d(lastSuccessfulTransformation);

            // Debug: Print edge data for failed GICP
            std::cout << "Adding edge between " << edge_data.vertex_id0 << " and " << edge_data.vertex_id1 << " using last successful transformation." << std::endl;
            std::cout << "Measurement:\n" << edge_data.measurement.matrix() << std::endl;

            // Add edge to optimizer
            if (optimizer.vertex(edge_data.vertex_id0) && optimizer.vertex(edge_data.vertex_id1)) {
                g2o::EdgeSE3* edge = new g2o::EdgeSE3();
                edge->setVertex(0, optimizer.vertex(edge_data.vertex_id0));
                edge->setVertex(1, optimizer.vertex(edge_data.vertex_id1));
                edge->setMeasurement(edge_data.measurement);

                Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
                information *= 100.0; // Adjust as needed
                edge->setInformation(information);

                edge->setRobustKernel(new g2o::RobustKernelHuber());

                optimizer.addEdge(edge);
            } else {
                std::cerr << "One of the vertices for the edge does not exist at index " << idx << "." << std::endl;
            }

            // Update lastValidIdx to current idx
            lastValidIdx = idx;

            // Add the current scan
            scans.push_back(currentCloud);

            continue;
        } else {
            // GICP succeeded
            // Convert to double precision
            Eigen::Matrix4d transformationMatrix_d = transformationMatrix.cast<double>();

            // Update cumulativeTransformation
            cumulativeTransformation = robotPositionsGICP[lastValidIdx] * transformationMatrix_d;

            // Update lastSuccessfulTransformation
            lastSuccessfulTransformation = transformationMatrix_d;

            // Debug: Print the transformation matrix
            std::cout << "Transformation Matrix for scan " << idx << ":\n" << transformationMatrix_d << std::endl;

            // Update robot positions from GICP
            robotPositionsGICP.push_back(cumulativeTransformation);
            robotPositionsOptimized.push_back(cumulativeTransformation);

            // Convert the transformation matrix to Isometry3d and store it
            Eigen::Isometry3d relative_transformation = Eigen::Isometry3d::Identity();
            relative_transformation.matrix() = transformationMatrix_d;

            // Store the edge data
            EdgeData edge_data;
            edge_data.vertex_id0 = static_cast<int>(lastValidIdx);
            edge_data.vertex_id1 = static_cast<int>(idx);
            edge_data.measurement = relative_transformation;
            edges.push_back(edge_data);

            // Debug: Print edge data
            std::cout << "Adding edge between " << edge_data.vertex_id0 << " and " << edge_data.vertex_id1 << std::endl;
            std::cout << "Measurement:\n" << edge_data.measurement.matrix() << std::endl;

            // Add the edge using the current edge_data
            if (optimizer.vertex(edge_data.vertex_id0) && optimizer.vertex(edge_data.vertex_id1)) {
                g2o::EdgeSE3* edge = new g2o::EdgeSE3();
                edge->setVertex(0, optimizer.vertex(edge_data.vertex_id0));
                edge->setVertex(1, optimizer.vertex(edge_data.vertex_id1));
                edge->setMeasurement(edge_data.measurement);

                Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
                information *= 100.0; // Adjust as needed
                edge->setInformation(information);

                edge->setRobustKernel(new g2o::RobustKernelHuber());

                optimizer.addEdge(edge);
            } else {
                std::cerr << "One of the vertices for the edge does not exist at index " << idx << "." << std::endl;
            }

            // Update lastValidIdx to current idx
            lastValidIdx = idx;

            // Add the current scan
            scans.push_back(currentCloud);
        }

        comparisonCount++;

        // Perform optimization every N scans
        if ((idx + 1) % N == 0 && idx != 0) {
            std::cout << "Performing graph optimization up to scan " << idx << "." << std::endl;

            // Debug: Print number of vertices and edges before optimization
            std::cout << "Optimizer has " << optimizer.vertices().size() << " vertices and "
                      << optimizer.edges().size() << " edges before optimization." << std::endl;

            // Optional: Print all vertex IDs
            std::cout << "Vertices IDs: ";
            for (const auto& vertex_pair : optimizer.vertices()) {
                std::cout << vertex_pair.first << " ";
            }
            std::cout << std::endl;

            // Optional: Print all edge connections
            std::cout << "Edges connections:" << std::endl;
            for (const auto& edge_ptr : optimizer.edges()) {
                const g2o::EdgeSE3* edge = dynamic_cast<const g2o::EdgeSE3*>(edge_ptr);
                if (edge) {
                    std::cout << "Edge from " << edge->vertex(0)->id() << " to " << edge->vertex(1)->id() << std::endl;
                }
            }

            // Optimize the graph
            optimizer.initializeOptimization();
            try {
                optimizer.optimize(10);
            } catch (const std::exception& e) {
                std::cerr << "Optimization threw an exception: " << e.what() << std::endl;
                return -1;
            } catch (...) {
                std::cerr << "Optimization threw an unknown exception." << std::endl;
                return -1;
            }

            // Update robot positions with optimized values
            for (size_t i = 0; i <= idx; ++i) {
                g2o::VertexSE3* pose_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(static_cast<int>(i)));
                if (!pose_vertex) {
                    std::cerr << "Pose vertex at index " << i << " not found." << std::endl;
                    continue;
                }
                Eigen::Isometry3d optimized_pose = pose_vertex->estimate();
                if (!optimized_pose.matrix().allFinite()) {
                    std::cerr << "Invalid optimized pose at index " << i << "." << std::endl;
                    continue;
                }
                // Update robotPositionsOptimized
                robotPositionsOptimized[i] = optimized_pose.matrix();
            }

            // Optional: Print updated robot positions
            std::cout << "Updated Optimized Robot Positions:" << std::endl;
            for (size_t i = 0; i <= idx; ++i) {
                std::cout << "Position " << i << ":\n" << robotPositionsOptimized[i] << std::endl;
            }
        }
    }

    // Final optimization after processing all scans
    std::cout << "Performing final graph optimization." << std::endl;

    // Debug: Print number of vertices and edges before final optimization
    std::cout << "Optimizer has " << optimizer.vertices().size() << " vertices and "
              << optimizer.edges().size() << " edges before final optimization." << std::endl;

    optimizer.initializeOptimization();
    try {
        optimizer.optimize(50);
    } catch (const std::exception& e) {
        std::cerr << "Final optimization threw an exception: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "Final optimization threw an unknown exception." << std::endl;
        return -1;
    }

    // Update robot positions with optimized values
    for (size_t i = 0; i < robotPositionsOptimized.size(); ++i) {
        g2o::VertexSE3* pose_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(static_cast<int>(i)));
        if (!pose_vertex) {
            std::cerr << "Pose vertex at index " << i << " not found during final optimization." << std::endl;
            continue;
        }
        Eigen::Isometry3d optimized_pose = pose_vertex->estimate();
        if (!optimized_pose.matrix().allFinite()) {
            std::cerr << "Invalid optimized pose at index " << i << " during final optimization." << std::endl;
            continue;
        }
        // Update robotPositionsOptimized
        robotPositionsOptimized[i] = optimized_pose.matrix();
    }

    // Visualize the final point clouds transformed using both GICP and optimized positions
    std::cout << "Visualizing the final point clouds with GICP positions (green) and optimized positions (red)." << std::endl;

    // Prepare point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gicp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_optimized(new pcl::PointCloud<pcl::PointXYZ>());

    for (size_t i = 0; i < scans.size(); ++i) {
        // Transform scans using GICP positions
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scan_gicp(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*scans[i], *transformed_scan_gicp, robotPositionsGICP[i].cast<float>());
        *cloud_gicp += *transformed_scan_gicp;

        // Transform scans using optimized positions
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scan_optimized(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*scans[i], *transformed_scan_optimized, robotPositionsOptimized[i].cast<float>());
        *cloud_optimized += *transformed_scan_optimized;
    }

    // Initialize the visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("GICP vs Optimized"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    // Add GICP cloud in green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler_gicp(cloud_gicp, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_gicp, color_handler_gicp, "cloud_gicp");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_gicp");

    // Add optimized cloud in red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler_optimized(cloud_optimized, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_optimized, color_handler_optimized, "cloud_optimized");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_optimized");

    // Keep the viewer open until the user closes it
    std::cout << "Press 'Q' in the visualizer window to exit." << std::endl;
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    viewer->close();

    // After processing all scans, merge and save the final registered point cloud using optimized positions
    std::cout << "Merging transformed scans and saving the final registered point cloud using optimized positions." << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_optimized(new pcl::PointCloud<pcl::PointXYZ>());

    // Transform and concatenate all scans using the optimized robot positions
    for (size_t i = 0; i < scans.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scan(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*scans[i], *transformed_scan, robotPositionsOptimized[i].cast<float>());
        *merged_cloud_optimized += *transformed_scan;
    }

    // Save the merged point cloud to a PCD file
    std::string output_filename = "final_registered_cloud_optimized.pcd";
    pcl::io::savePCDFileBinary(output_filename, *merged_cloud_optimized);
    std::cout << "Final registered point cloud saved to " << output_filename << std::endl;

    // Optionally, save the merged point cloud using GICP positions
    std::cout << "Merging transformed scans and saving the final registered point cloud using GICP positions." << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_gicp(new pcl::PointCloud<pcl::PointXYZ>());

    // Transform and concatenate all scans using the GICP robot positions
    for (size_t i = 0; i < scans.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scan(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*scans[i], *transformed_scan, robotPositionsGICP[i].cast<float>());
        *merged_cloud_gicp += *transformed_scan;
    }

    // Save the merged point cloud to a PCD file
    std::string output_filename_gicp = "final_registered_cloud_gicp.pcd";
    pcl::io::savePCDFileBinary(output_filename_gicp, *merged_cloud_gicp);
    std::cout << "Final registered point cloud saved to " << output_filename_gicp << std::endl;

    std::cout << "Processing completed." << std::endl;
    return 0;
}
