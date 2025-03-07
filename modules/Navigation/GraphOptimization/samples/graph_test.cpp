#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

// g2o headers
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/types_slam2d.h>

struct Pose2D {
    double x;
    double y;
    double yaw;
};

typedef Eigen::Matrix<double, 4, 4> Matrix4d;

// Function to read robot poses from a file
std::vector<Pose2D> readRobotPoses(const std::string& filename) {
    std::vector<Pose2D> poses;
    std::ifstream infile(filename);
    if (!infile) {
        std::cerr << "Cannot open robot poses file: " << filename << std::endl;
        return poses;
    }

    std::string line;
    while (std::getline(infile, line)) {
        Pose2D pose;
        std::istringstream iss(line);
        if (!(iss >> pose.x >> pose.y >> pose.yaw)) { break; }
        poses.push_back(pose);
    }
    return poses;
}

// Function to read transformation matrices from a file
std::vector<Matrix4d> readTransformations(const std::string& filename) {
    std::vector<Matrix4d> transformations;
    std::ifstream infile(filename);
    if (!infile) {
        std::cerr << "Cannot open transformations file: " << filename << std::endl;
        return transformations;
    }

    std::string line;
    Matrix4d mat;
    int row = 0;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double a, b, c, d;
        if (!(iss >> a >> b >> c >> d)) { break; }
        mat(row, 0) = a;
        mat(row, 1) = b;
        mat(row, 2) = c;
        mat(row, 3) = d;
        row++;
        if (row == 4) {
            transformations.push_back(mat);
            row = 0;
        }
    }
    return transformations;
}

int main(int argc, char** argv) {
    // Read robot poses and transformations
    std::vector<Pose2D> robotPoses = readRobotPoses("robot_poses.txt");
    std::vector<Matrix4d> transformations = readTransformations("transformations.txt");

    if (robotPoses.empty() || transformations.empty()) {
        std::cerr << "No data to process." << std::endl;
        return -1;
    }

    // Initialize g2o optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);

    // Choose the solver
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3, 3> > BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    std::unique_ptr<LinearSolverType> linearSolver (new LinearSolverType());
    std::unique_ptr<BlockSolverType> solver (new BlockSolverType(std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(solver));

    optimizer.setAlgorithm(optimizationAlgorithm);

    // Add vertices
    for (size_t i = 0; i < robotPoses.size(); ++i) {
        g2o::VertexSE2* v = new g2o::VertexSE2();
        v->setId(i);
        v->setEstimate(g2o::SE2(robotPoses[i].x, robotPoses[i].y, robotPoses[i].yaw));
        if (i == 0) {
            v->setFixed(true); // Fix the first pose
        }
        optimizer.addVertex(v);
    }

    // Add edges based on transformation matrices
    for (size_t i = 0; i < transformations.size(); ++i) {
        if (i + 1 >= robotPoses.size()) break;

        // Extract relative pose from transformation matrix
        Eigen::Matrix3d relativePose;
        relativePose << transformations[i](0,0), transformations[i](0,1), transformations[i](0,3),
                        transformations[i](1,0), transformations[i](1,1), transformations[i](1,3),
                        0, 0, 1;

        g2o::EdgeSE2* edge = new g2o::EdgeSE2();
        edge->vertices()[0] = optimizer.vertex(i);
        edge->vertices()[1] = optimizer.vertex(i+1);
        edge->setMeasurement(g2o::SE2(relativePose(0,2), relativePose(1,2), atan2(relativePose(1,0), relativePose(0,0))));

        // Set information matrix (assuming some uncertainty)
        Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
        edge->setInformation(information);

        optimizer.addEdge(edge);
    }

    // Initialize optimization
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // Output optimized poses
    std::cout << "Optimized Poses:" << std::endl;
    for (size_t i = 0; i < robotPoses.size(); ++i) {
        g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(i));
        if (v) {
            g2o::SE2 est = v->estimate();
            std::cout << "Pose " << i << ": x=" << est.translation().x()
                      << ", y=" << est.translation().y()
                      << ", yaw=" << est.rotation().angle() << std::endl;
        }
    }

    return 0;
}
