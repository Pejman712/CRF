/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include "GraphOptimization/GraphOptimization.hpp"
#include "GraphOptimization/GraphOptimizationConfiguration.hpp"
#include "Mapper3d/Mapper3d.hpp"

// g2o graph
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/core/eigen_types.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>
#include <g2o/types/slam3d/se3quat.h>

// g2o optimer
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include "../libraries/matplotlibcpp.h"

#include <iostream>
#include <boost/optional.hpp>
#include <string>
#include <fstream>
#include <ctime>
#include <nlohmann/json.hpp>
#include <vector>
#include <memory>
#include <utility>

using crf::applications::graphoptimization::GraphOptimizationConfiguration;
using crf::applications::graphoptimization::NeighboursSearch;
using crf::applications::graphoptimization::Solvers;
using crf::applications::graphoptimization::Simulations;
using crf::applications::graphoptimization::SlamParameters;

namespace crf {
namespace applications {
namespace graphoptimization {

GraphOptimization::GraphOptimization(const std::string& mapperConfigFile):
    log_("GraphOptimization"),
    map_(mapperConfigFile),
    kdtreeCloud_ (new pcl::PointCloud<pcl::PointXYZ>()),
    relationKnn_{},
    nodePointClouds_{},
    nodePointCloudsNormal_{},
    optimizer_(),
    pointsSearch_{},
    correctComparison_{},
    fitnessScoreNeighbours_{},
    crudePositionsX{},
    crudePositionsY{},
    groundTruthX{},
    groundTruthY{},
    odometryX{},
    odometryY{},
    knnMax_{0},
    closestSearch_{false},
    radius_{0.0},
    iterations_{0},
    robustKernel_{false},
    appliedParse_{false},
    idVertex_{0},
    Knn_{0},
    organizedPointCloud_{false},
    fitnessScore_{0.0},
    graphConfig_{} {
    log_->debug("CTor");
}

GraphOptimization::~GraphOptimization() {
    log_->debug("DTor");
    resetGraph();
}

bool GraphOptimization::parse(const std::string& configFileName) {
    graphConfig_ = std::make_shared<GraphOptimizationConfiguration>();
    if (!graphConfig_->parse(configFileName)) {
        log_->error("Something wrong in the configFile");
        return false;
    }

    // Get the information
    boost::optional<NeighboursSearch> boostNeighboursData =
        graphConfig_->getNeighbourgsSearchData();
    boost::optional<Solvers> boostSolversData = graphConfig_->getSolversData();
    boost::optional<SlamParameters> boostSlamData = graphConfig_->getSlamData();
    if (!boostNeighboursData) {
        log_->warn("There was a problem taking neighboursData");
        return false;
    }
    if (!boostSolversData) {
        log_->warn("There was a problem taking solversData");
        return false;
    }
    if (!boostSlamData) {
        log_->warn("There was a problem taking slamData");
        return false;
    }
    NeighboursSearch neighboursData = boostNeighboursData.get();
    Solvers solversData = boostSolversData.get();
    SlamParameters slamData = boostSlamData.get();

    // kNN parameters
    knnMax_ = neighboursData.value;
    correctComparison_.clear();
    for (unsigned int i = 0; i < knnMax_; i++) {
        correctComparison_.push_back(false);
    }
    if (neighboursData.type == NeighboursSearch::Closest) {
        closestSearch_ = true;
    } else {
        closestSearch_ = false;
    }
    radius_ = neighboursData.radius;

    // Optimizer
    iterations_ = solversData.iterations;
    robustKernel_ = solversData.robustKernel;
    optimizer_.setVerbose(solversData.verbose);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver_;

    // Linear solver selection (and creation)
    if (solversData.linearType == Solvers::LinearSolverType::Dense) {
        log_->debug("Dense solver stablished");
        linearSolver_ = g2o::make_unique
            <g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    } else if (solversData.linearType == Solvers::LinearSolverType::Choldmod) {
        log_->debug("Cholmod solver stablished");
        linearSolver_ = g2o::make_unique
            <g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
    } else if (solversData.linearType == Solvers::LinearSolverType::Csparse) {
        log_->debug("CSparse solver stablished");
        log_->warn("CSparse is not available");
        return false;
        // linearSolver_ = g2o::make_unique
        //     <g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>>();
    } else if (solversData.linearType == Solvers::LinearSolverType::Eigen) {
        log_->debug("Eigen solver stablished");
        linearSolver_ = g2o::make_unique
            <g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    } else {
        log_->debug("PCG solver stablished");
        linearSolver_ = g2o::make_unique
            <g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>>();
    }

    // Block solver creation
    std::unique_ptr<g2o::BlockSolver_6_3> block_solver =
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver_));

    // Non linear optimization creation
    if (solversData.nonLinearType == Solvers::NonLinearSolverType::Levenberg) {
        g2o::OptimizationAlgorithmLevenberg* solver_ =
            new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
        if (solversData.lambdaSelection) {
            solver_->g2o::OptimizationAlgorithmLevenberg::setUserLambdaInit(solversData.lambda);
        }
        if (solversData.trialsSelection) {
            solver_->
                g2o::OptimizationAlgorithmLevenberg::setMaxTrialsAfterFailure(solversData.trials);
        }
        optimizer_.setAlgorithm(solver_);
    } else {
        g2o::OptimizationAlgorithm* solver_ =
            new g2o::OptimizationAlgorithmGaussNewton(std::move(block_solver));
        optimizer_.setAlgorithm(solver_);
    }

    organizedPointCloud_ = slamData.organizedPointCloud;

    log_->debug("Configuring optimizer... Done");

    appliedParse_ = true;
    return true;
}

bool GraphOptimization::addVertex(const Eigen::Matrix4f &realMotion,
    const Eigen::Matrix4f &previousPosition,
    boost::optional <const Eigen::Vector3f> boostFixPose,
    boost::optional <const Eigen::Vector3f> odometryPosition,
    boost::optional <const Eigen::Vector3f> groundTruthPosition) {
    // ----------------------------------------------------------
    // 3 - Add the new vertex to the graph
    // ----------------------------------------------------------
    if (!appliedParse_) {
        log_->error("Parse function must be called before add any vertex to the graph");
        return false;
    }
    // Test if realMotion is good
    for (int i = 0; i < 3; i++) {
        float sum = 0;
        for (int j = 0; j < 3; j++) {
            sum += realMotion(j, i) * realMotion(j, i);
        }
        if ((sum < 0.99) || (sum > 1.01)) {
            log_->error("Input rotation matrix to the graph is strange. Something was wrong");
            return false;
        }
    }

    log_->info("3 - Adding the new vertex to the graph... ");

    // Start the chrono
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};
    start = std::chrono::system_clock::now();

    // --- Correct the position
    Eigen::Matrix4f realMotion_;
    g2o::Isometry3 isoRealMotion_;
    Eigen::Matrix4f relativeMotion;
    g2o::Isometry3 isoRelativeMotion;

    // Take vertex previous position
    if (idVertex_ > 1) {
        // Take vertex position
        boost::optional <Eigen::Matrix4f> boostPositionVertex = readVertexPosition(idVertex_ - 1);

        if (!boostPositionVertex) {
            log_->error("Unable to read the position of the vertex");
            return false;
        }

        Eigen::Matrix4f previousPosition_ = boostPositionVertex.get();

        log_->debug("Transforming vertex position {0} to real one", idVertex_ - 1);

        realMotion_ = previousPosition_ * previousPosition.inverse() * realMotion;

        isoRealMotion_ = matrix4ftoIsometry3(realMotion_);
    } else {
        isoRealMotion_ = matrix4ftoIsometry3(realMotion);
        realMotion_ = realMotion;
    }

    relativeMotion = previousPosition.inverse() * realMotion;
    isoRelativeMotion = matrix4ftoIsometry3(relativeMotion);

    log_->info("Position of the vertex {0} before optimize", idVertex_);
    log_->info("{0}, {1}, {2}, {3}", isoRealMotion_(0, 0), isoRealMotion_(0, 1),
        isoRealMotion_(0, 2), isoRealMotion_(0, 3));
    log_->info("{0}, {1}, {2}, {3}", isoRealMotion_(1, 0), isoRealMotion_(1, 1),
        isoRealMotion_(1, 2), isoRealMotion_(1, 3));
    log_->info("{0}, {1}, {2}, {3}", isoRealMotion_(2, 0), isoRealMotion_(2, 1),
        isoRealMotion_(2, 2), isoRealMotion_(2, 3));
    log_->info("{0}, {1}, {2}, {3}", isoRealMotion_(3, 0), isoRealMotion_(3, 1),
        isoRealMotion_(3, 2), isoRealMotion_(3, 3));

    // Create the vertex
    log_->debug("Adding vertex {0} to graph", idVertex_);
    g2o::VertexSE3 *v_se3 = new g2o::VertexSE3();
    v_se3->setId(idVertex_);
    if (boostFixPose) {
        v_se3->setFixed(true);
    }

    // Take the rotation and translation
    g2o::Matrix3 R;
    R << isoRealMotion_(0, 0), isoRealMotion_(0, 1), isoRealMotion_(0, 2),
         isoRealMotion_(1, 0), isoRealMotion_(1, 1), isoRealMotion_(1, 2),
         isoRealMotion_(2, 0), isoRealMotion_(2, 1), isoRealMotion_(2, 2);
    g2o::Vector3 t;
    t << isoRealMotion_(0, 3), isoRealMotion_(1, 3), isoRealMotion_(2, 3);

    if (boostFixPose) {
        Eigen::Vector3f fixPose = boostFixPose.get();
        t << fixPose(0), fixPose(1), fixPose(2);
    }

    // Position of the new vertex in the map
    g2o::SE3Quat pose(R, t);
    v_se3->setEstimate(pose);

    // Add the vertex to the optimizer
    optimizer_.addVertex(v_se3);

    // Save all the positions
    crudePositionsX.push_back(realMotion(0, 3));
    crudePositionsY.push_back(realMotion(1, 3));
    if (groundTruthPosition) {
        groundTruthX.push_back(groundTruthPosition.value()(0));
        groundTruthY.push_back(groundTruthPosition.value()(1));
    }
    if (odometryPosition) {
        odometryX.push_back(odometryPosition.value()(0));
        odometryY.push_back(odometryPosition.value()(1));
    }

    log_->info("3 - Adding the new vertex to the graph... Done");
    end = std::chrono::system_clock::now();
    elapsedMilliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    log_->debug("3 - Add the new vertex to the graph execution time: {0}", elapsedMilliseconds);

    Eigen::Vector3f pointPose;
    pointPose << isoRealMotion_(0, 3), isoRealMotion_(1, 3), isoRealMotion_(2, 3);

    // 5 - Calculation of the K NN
    if (!determineKNN(pointPose)) {
        log_->error("There was a problem determining the k NN");
        return false;
    }

    // 6 - PC comparation with the K NN (less with the previous one)
    if (!knnComparison(realMotion_)) {
        log_->error("There was a problem comparing the point clouds");
        return false;
    }

    // 7 - Add the edges to the graph
    if (!addEdges(isoRelativeMotion)) {
        log_->error("There was a problem adding the edges");
        return false;
    }

    // 8 - Pose estimation of the new pose
    if (!graphOptimize()) {
        log_->error("There was a problem optimizing the graph");
        return false;
    }

    // 9 - Add the new pose to the kdtree
    if (!updateKdTree()) {
        log_->error("There was a problem updating the KdTree");
        return false;
    }

    // Take vertex position
    boost::optional <Eigen::Matrix4f> boostPositionVertex = readVertexPosition(idVertex_);

    if (!boostPositionVertex) {
        log_->error("Unable to read the position of the vertex");
        return false;
    }

    Eigen::Matrix4f positionVertex = boostPositionVertex.get();

    log_->info("Position of the vertex {0} after optimization", idVertex_);
    log_->info("{0}, {1}, {2}, {3}", positionVertex(0, 0), positionVertex(0, 1),
        positionVertex(0, 2), positionVertex(0, 3));
    log_->info("{0}, {1}, {2}, {3}", positionVertex(1, 0), positionVertex(1, 1),
        positionVertex(1, 2), positionVertex(1, 3));
    log_->info("{0}, {1}, {2}, {3}", positionVertex(2, 0), positionVertex(2, 1),
        positionVertex(2, 2), positionVertex(2, 3));
    log_->info("{0}, {1}, {2}, {3}", positionVertex(3, 0), positionVertex(3, 1),
        positionVertex(3, 2), positionVertex(3, 3));

    idVertex_++;

    return true;
}

boost::optional<Eigen::Matrix4f> GraphOptimization::comparePointClouds(
    const Eigen::Matrix4f &sourceSensorOrigin,
    const Eigen::Matrix4f &targetSensorOrigin,
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &sourcePointCloud,
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &targetPointCloud) {
    // Calculate the new position of the vertex using Mapper3d
    log_->info("3 - Comparing point clouds...");
    // Start the chrono
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};
    start = std::chrono::system_clock::now();

    boost::optional<Eigen::Matrix4f> boostRealMotion;
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals (new pcl::PointCloud<pcl::Normal>());

    if (idVertex_ == 1) {
        pcl::PointCloud<pcl::Normal>::Ptr targetNormals (new pcl::PointCloud<pcl::Normal>());
        boostRealMotion = map_.comparePointClouds(organizedPointCloud_, false, false,
            sourceSensorOrigin, targetSensorOrigin, sourcePointCloud, targetPointCloud,
            sourceNormals, targetNormals);

        // Add point clouds of the vertex 0
        nodePointClouds_.push_back(targetPointCloud);
        nodePointCloudsNormal_.push_back(targetNormals);
    } else {
        // Take previous point clouds
        pcl::PointCloud<pcl::Normal>::Ptr targetNormals =
            nodePointCloudsNormal_.at(nodePointCloudsNormal_.size() - 1);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetPointCloud =
            nodePointClouds_.at(nodePointClouds_.size() - 1);

        boostRealMotion = map_.comparePointClouds(organizedPointCloud_, false, true,
            sourceSensorOrigin, targetSensorOrigin, sourcePointCloud, targetPointCloud,
            sourceNormals, targetNormals);
    }

    if (!boostRealMotion) {
        log_->error("Unable to compare the two provided point clouds");
        return boost::none;
    }

    nodePointClouds_.push_back(sourcePointCloud);
    nodePointCloudsNormal_.push_back(sourceNormals);
    fitnessScore_ = map_.getFitnessScore();

    // Execute time
    log_->info("3 - Comparing point clouds... Done");
    end = std::chrono::system_clock::now();
    elapsedMilliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    log_->info("3 - Comparing point clouds execution time: {0}", elapsedMilliseconds);

    return boostRealMotion.get();
}

boost::optional<std::vector <pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>>
    GraphOptimization::getNodePointclouds() {
    if (!(nodePointClouds_.size() > 0)) {
        log_->warn("Impossible to get Node Pointclouds. No pointcloud stored");
        return boost::none;
    }
    return nodePointClouds_;
}

boost::optional <std::vector <Eigen::Matrix4f, Eigen::aligned_allocator <Eigen::Matrix4f>>>
    GraphOptimization::getNodePositions() {
    std::vector <Eigen::Matrix4f, Eigen::aligned_allocator <Eigen::Matrix4f>> nodePositions;

    // Take vertex position
    g2o::HyperGraph::VertexIDMap container = optimizer_.vertices();

    if (!(container.size() > 0)) {
        log_->warn("Impossible to get Node Positions. No points stored");
        return boost::none;
    }

    for (size_t i = 0; i < container.size(); i++) {
        // Take vertex position
        boost::optional <Eigen::Matrix4f> boostPositionVertex = readVertexPosition(i);

        if (!boostPositionVertex) {
            log_->warn("Unable to read the position of the vertex");
            return boost::none;
        }

        Eigen::Matrix4f positionVertex = boostPositionVertex.get();
        nodePositions.push_back(positionVertex);
    }

    return nodePositions;
}

bool GraphOptimization::resetGraph() {
    // Reset all variables
    kdtreeCloud_->points.clear();
    relationKnn_.clear();
    nodePointClouds_.clear();
    optimizer_.clear();
    optimizer_.clearParameters();
    pointsSearch_.clear();
    correctComparison_.clear();
    crudePositionsX.clear();
    crudePositionsY.clear();
    groundTruthX.clear();
    groundTruthY.clear();
    odometryX.clear();
    odometryY.clear();
    knnMax_ = 0;
    closestSearch_ = false;
    radius_ = 0.0;
    iterations_ = 0;
    robustKernel_ = false;
    appliedParse_ = 0;
    idVertex_ = -1;
    Knn_ = 0;
    organizedPointCloud_ = false;

    return true;
}

bool GraphOptimization::saveGraph(const std::string& filename) {
    // Save data before optimize
    log_->info("Saving the pure graph...");
    try {
        std::string save_name = "../modules/Applications/GraphOptimization/results/";
        save_name += filename;
        save_name += ".g2o";

        std::filebuf fb;
        fb.open(save_name, std::ios::out);
        std::ostream os(&fb);

        optimizer_.save(os);
    }
    catch (std::exception &except) {
        return false;
    }
    log_->info("Saving the pure graph... Done");

    return true;
}

bool GraphOptimization::saveResults(const std::string& filename) {
    // Get the optimize node positions
    boost::optional <std::vector <Eigen::Matrix4f, Eigen::aligned_allocator <Eigen::Matrix4f>>>
        boostNodePosition = getNodePositions();
    if (!boostNodePosition) {
        log_->warn("Unable to read the node point cloud");
        return false;
    }
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> nodePosition =
        boostNodePosition.get();
    std::vector<double> correctedPositionsX;
    std::vector<double> correctedPositionsY;

    if (!(nodePosition.size() > 0)) {
        log_->warn("Node Position vector empty");
        return false;
    }

    for (size_t i = 0; i < nodePosition.size(); i++) {
        Eigen::Matrix4f singleCorrectedMatrixPosition = nodePosition.at(i);
        correctedPositionsX.push_back(singleCorrectedMatrixPosition(0, 3));
        correctedPositionsY.push_back(singleCorrectedMatrixPosition(1, 3));
    }

    // Save the result
    std::string save_name = "../modules/Applications/GraphOptimization/results/";
    save_name += filename;
    save_name += ".png";
    matplotlibcpp::plot(crudePositionsX, crudePositionsY, "-k", groundTruthX, groundTruthY, "-g",
        correctedPositionsX, correctedPositionsY, "-r", odometryX, odometryY, "-b");
    matplotlibcpp::title("Position register");
    matplotlibcpp::save(save_name);

    return true;
}

boost::optional <NeighboursSearch> GraphOptimization::getNeighbourgsSearchData() {
    if (!appliedParse_) {
        log_->error("Parse function must be called before get information of the configfile");
        return boost::none;
    }
    boost::optional <NeighboursSearch> neighboursData = graphConfig_->getNeighbourgsSearchData();
    return neighboursData;
}

boost::optional <Solvers> GraphOptimization::getSolversData() {
    if (!appliedParse_) {
        log_->error("Parse function must be called before get information of the configfile");
        return boost::none;
    }
    boost::optional <Solvers> solversData = graphConfig_->getSolversData();
    return solversData;
}

boost::optional <Simulations> GraphOptimization::getSimulationsData() {
    if (!appliedParse_) {
        log_->error("Parse function must be called before get information of the configfile");
        return boost::none;
    }
    boost::optional <Simulations> simulationsData = graphConfig_->getSimulationsData();
    return simulationsData;
}

boost::optional <SlamParameters> GraphOptimization::getSlamData() {
    if (!appliedParse_) {
        log_->error("Parse function must be called before get information of the configfile");
        return boost::none;
    }
    boost::optional <SlamParameters> slamData = graphConfig_->getSlamData();
    return slamData;
}

boost::optional <Eigen::Matrix4f> GraphOptimization::getTransformationData() {
    if (!appliedParse_) {
        log_->error("Parse function must be called before get information of the configfile");
        return boost::none;
    }
    boost::optional <Eigen::Matrix4f> tranformationData = graphConfig_->getTransformationData();
    return tranformationData;
}

boost::optional <Eigen::Matrix4f> GraphOptimization::getInitialPositionData() {
    if (!appliedParse_) {
        log_->error("Parse function must be called before get information of the configfile");
        return boost::none;
    }
    boost::optional <Eigen::Matrix4f> tranformationData = graphConfig_->getInitialPositionData();
    return tranformationData;
}

boost::optional <std::vector <std::vector <float>>> GraphOptimization::getDoorsPosition() {
    if (!appliedParse_) {
        log_->error("Parse function must be called before get information of the configfile");
        return boost::none;
    }
    boost::optional <std::vector <std::vector <float>>> doorsPosition =
        graphConfig_->getDoorsPosition();
    return doorsPosition;
}

bool GraphOptimization::determineKNN(const Eigen::Vector3f &pointPose) {
    if (idVertex_ < knnMax_) {
        Knn_++;
    }

    if ((idVertex_ <= 1) || (Knn_ <= 1)) {
        return true;
    }

    log_->info("4 - Calculation of the K NN...");

    // Start the chrono
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};
    start = std::chrono::system_clock::now();

    // init the tree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
    kdtree_.setInputCloud(kdtreeCloud_);

    pcl::PointXYZ searchPoint;
    searchPoint.x = pointPose(0);
    searchPoint.y = pointPose(1);
    searchPoint.z = pointPose(2);

    pointsSearch_.clear();
    std::vector<float> pointSquareDistance_;

    if (closestSearch_ == 1) {
        // Space reserve
        pointsSearch_.reserve(Knn_);
        pointSquareDistance_.reserve(Knn_);

        log_->debug("K nearest neighbor search at ({0},{1},{2}) with K={3}", searchPoint.x,
            searchPoint.y, searchPoint.z, Knn_);

        // Knn search
        if (kdtree_.nearestKSearch(searchPoint, Knn_, pointsSearch_,
            pointSquareDistance_) > 0) {
            for (std::size_t i = 0; i < pointsSearch_.size (); ++i)
                log_->debug(" * ({0},{1},{2}) (squared distance: {3})",
                    kdtreeCloud_->points[ pointsSearch_.at(i) ].x,
                    kdtreeCloud_->points[ pointsSearch_.at(i) ].y,
                    kdtreeCloud_->points[ pointsSearch_.at(i) ].z,
                    pointSquareDistance_.at(i));
        }

    } else {
        log_->debug("Neighbors within radius search at ({0},{1},{2}) with radius={3}",
            searchPoint.x, searchPoint.y, searchPoint.z, radius_);

        if (kdtree_.radiusSearch(searchPoint, radius_, pointsSearch_,
            pointSquareDistance_) > 0) {
            for (std::size_t i = 0; i < pointsSearch_.size(); ++i)
                log_->debug(" * ({0},{1},{2}) (squared distance: {3})",
                    kdtreeCloud_->points[ pointsSearch_.at(i) ].x,
                    kdtreeCloud_->points[ pointsSearch_.at(i) ].y,
                    kdtreeCloud_->points[ pointsSearch_.at(i) ].z,
                    pointSquareDistance_.at(i));
        }
    }

    // Execute time
    log_->info("4 - Calculation of the K NN... Done");
    end = std::chrono::system_clock::now();
    elapsedMilliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    log_->debug("4 - Calculate of the K NN execution time: {0}", elapsedMilliseconds);

    return true;
}

bool GraphOptimization::knnComparison(const Eigen::Matrix4f &realMotion) {
    if (idVertex_ <= 1)
        return true;

    log_->info("5 - PC comparation with the K NN...");

    // Start the chrono
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};
    start = std::chrono::system_clock::now();

    relationKnn_.clear();
    correctComparison_.clear();
    fitnessScoreNeighbours_.clear();
    for (unsigned int i = 0; i < knnMax_; i++) {
        correctComparison_.push_back(false);
    }

    for (std::size_t i = 0; i < pointsSearch_.size(); i++) {
        // Function if, in case the point is the previous one
        // Pass if (x, y, z) are the same than the previous point
        if (!((kdtreeCloud_->points[pointsSearch_.at(i)].x ==
            kdtreeCloud_->points[idVertex_-1].x) &&
            (kdtreeCloud_->points[pointsSearch_.at(i)].y ==
            kdtreeCloud_->points[idVertex_-1].y) &&
            (kdtreeCloud_->points[pointsSearch_.at(i)].z ==
            kdtreeCloud_->points[idVertex_-1].z))) {
            // Take vertex position
            log_->debug("Taken vertex {} position", pointsSearch_.at(i));
            boost::optional <Eigen::Matrix4f> boostPreviousPosition =
                readVertexPosition(pointsSearch_.at(i));
            if (!boostPreviousPosition) {
                log_->warn("Unable to read the position of the vertex");
                return false;
            }
            Eigen::Matrix4f previousPosition = boostPreviousPosition.get();

            // Take common point clouds
            log_->debug("Taken normals and RGB PCs of {} and {}", pointsSearch_.at(i), idVertex_);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud = nodePointClouds_.at(idVertex_);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr previousPointCloud =
                nodePointClouds_.at(pointsSearch_.at(i));
            pcl::PointCloud<pcl::Normal>::Ptr sourceNormals = nodePointCloudsNormal_.at(idVertex_);
            pcl::PointCloud<pcl::Normal>::Ptr targetNormals =
                nodePointCloudsNormal_.at(pointsSearch_.at(i));

            // Compare with the NN
            log_->debug("Comparing point clouds");
            boost::optional<Eigen::Matrix4f> boostRealMotionkNN = map_.comparePointClouds(
                organizedPointCloud_, true, true, realMotion, previousPosition, inPointCloud,
                previousPointCloud, sourceNormals, targetNormals);

            if (!boostRealMotionkNN) {
                log_->warn("Unable to calculate the relation between {0} with {1}",
                    pointsSearch_.at(i), idVertex_);
                correctComparison_.at(i) = false;
                continue;
            }

            Eigen::Matrix4f realMotionkNN = boostRealMotionkNN.get();

            // Obtain relation between previous position and actual position
            realMotionkNN = previousPosition.inverse()*realMotionkNN;

            // Save the k NN Isometrys in a vector to put them in the edges
            // From homogenuous matrix to isometry
            g2o::Matrix3 Rot;
            Rot << realMotionkNN(0, 0), realMotionkNN(0, 1), realMotionkNN(0, 2),
                   realMotionkNN(1, 0), realMotionkNN(1, 1), realMotionkNN(1, 2),
                   realMotionkNN(2, 0), realMotionkNN(2, 1), realMotionkNN(2, 2);
            g2o::Vector3 eulerVector = g2o::internal::toEuler(Rot);
            g2o::Vector6 vectorET;
            vectorET << realMotionkNN(0, 3), realMotionkNN(1, 3), realMotionkNN(2, 3),
                eulerVector;

            log_->debug("Pushing back information of the last position...");
            relationKnn_.push_back(g2o::internal::fromVectorET(vectorET));
            correctComparison_.at(i) = true;
            fitnessScoreNeighbours_.push_back(map_.getFitnessScore());
            log_->debug("Pushing back information of the last position... Done");
        }
    }

    // Execute time
    log_->info("5 - PC comparation with the K NN... Done");
    end = std::chrono::system_clock::now();
    elapsedMilliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    log_->debug("5 - PC comparation with the K NN execution time: {0}", elapsedMilliseconds);

    return true;
}

bool GraphOptimization::addEdges(const g2o::Isometry3 &realMotion_) {
    if (idVertex_ <= 0)
        return true;

    log_->info("6 - Add the edges to the graph...");

    // Start the chrono
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};
    start = std::chrono::system_clock::now();

    // Add the previous one
    g2o::EdgeSE3 *e_se3 = new g2o::EdgeSE3();

    e_se3->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>
        (optimizer_.vertices().find(idVertex_ - 1)->second));
    e_se3->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
        (optimizer_.vertices().find(idVertex_)->second));
    e_se3->setMeasurement(realMotion_);
    e_se3->information() = fitnessScore_ * g2o::EdgeSE3::InformationType::Identity();

    e_se3->setParameterId(0, 0);
    optimizer_.addEdge(e_se3);

    log_->debug("Added edge {0}-{1}", idVertex_ - 1, idVertex_);

    // Add the k NN
    int passed {0};
    for (size_t i = 0; i < pointsSearch_.size(); i++) {
        if ((!((kdtreeCloud_->points[pointsSearch_.at(i)].x ==
            kdtreeCloud_->points[idVertex_-1].x) &&
            (kdtreeCloud_->points[pointsSearch_.at(i)].y ==
            kdtreeCloud_->points[idVertex_-1].y) &&
            (kdtreeCloud_->points[pointsSearch_.at(i)].z ==
            kdtreeCloud_->points[idVertex_-1].z))) &&
            (correctComparison_.at(i) == true)) {
            int iterator = i - passed;

            log_->info("Comparison between {0}-{1}", pointsSearch_.at(i), idVertex_);
            log_->info("{0}, {1}, {2}, {3}",
                relationKnn_.at(iterator)(0, 0), relationKnn_.at(iterator)(0, 1),
                relationKnn_.at(iterator)(0, 2), relationKnn_.at(iterator)(0, 3));
            log_->info("{0}, {1}, {2}, {3}",
                relationKnn_.at(iterator)(1, 0), relationKnn_.at(iterator)(1, 1),
                relationKnn_.at(iterator)(1, 2), relationKnn_.at(iterator)(1, 3));
            log_->info("{0}, {1}, {2}, {3}",
                relationKnn_.at(iterator)(2, 0), relationKnn_.at(iterator)(2, 1),
                relationKnn_.at(iterator)(2, 2), relationKnn_.at(iterator)(2, 3));
            log_->info("{0}, {1}, {2}, {3}",
                relationKnn_.at(iterator)(3, 0), relationKnn_.at(iterator)(3, 1),
                relationKnn_.at(iterator)(3, 2), relationKnn_.at(iterator)(3, 3));

            // Create the edge
            g2o::EdgeSE3 *e_se3 = new g2o::EdgeSE3();

            e_se3->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                (optimizer_.vertices().find(pointsSearch_.at(i))->second));
            e_se3->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                (optimizer_.vertices().find(idVertex_)->second));
            e_se3->setMeasurement(relationKnn_.at(iterator));
            e_se3->information() =
                fitnessScoreNeighbours_.at(iterator) * g2o::EdgeSE3::InformationType::Identity();
            // e_se3->setInformation( g2o::EdgeSE3::InformationType::Identity());
            // e_se3->setInformation( Eigen::MatrixXd::Identity(6,6) * 10.);

            e_se3->setParameterId(0, 0);
            if (robustKernel_) {
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e_se3->setRobustKernel(rk);
            }
            optimizer_.addEdge(e_se3);

            log_->debug("Added edge {0}-{1}", pointsSearch_.at(i), idVertex_);
            continue;
        }
        passed = 1;
    }

    // Execute time
    log_->info("6 - Add the edges to the graph... Done");
    end = std::chrono::system_clock::now();
    elapsedMilliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    log_->debug("6 - Add the edges to the graph execution time: {0}", elapsedMilliseconds);

    return true;
}

bool GraphOptimization::graphOptimize() {
    if (idVertex_ <= 1)
        return true;

    log_->info("7 - Pose estimation of the new pose...");

    // Start the chrono
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};
    start = std::chrono::system_clock::now();

    log_->info("Optimizing the graph...");
    optimizer_.initializeOptimization();
    optimizer_.optimize(iterations_);
    log_->info("Optimizing the graph... Done");

    // Execute time
    log_->info("7 - Pose estimation of the new pose... Done");
    end = std::chrono::system_clock::now();
    elapsedMilliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    log_->debug("7 - Pose estimation of the new pose execution time: {0}", elapsedMilliseconds);

    return true;
}

bool GraphOptimization::updateKdTree() {
    log_->info("8 - Add the new pose to the kdtree...");

    // Start the chrono
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};
    start = std::chrono::system_clock::now();

    // Declare the kdtree points
    kdtreeCloud_->points.clear();
    kdtreeCloud_->points.resize(idVertex_ + 1);

    for (unsigned int i = 0; i <= idVertex_; i++) {
        // Take vertex position
        boost::optional <Eigen::Matrix4f> boostPositionVertex = readVertexPosition(i);

        if (!boostPositionVertex) {
            log_->warn("Unable to read the position of the vertex");
            return false;
        }

        Eigen::Matrix4f positionVertex = boostPositionVertex.get();

        // Fill the kdtree pointcloud
        kdtreeCloud_->points[i].x = positionVertex(0, 3);
        kdtreeCloud_->points[i].y = positionVertex(1, 3);
        kdtreeCloud_->points[i].z = positionVertex(2, 3);
    }

    // Execute time
    log_->info("8 - Add the new pose to the kdtree... Done");
    end = std::chrono::system_clock::now();
    elapsedMilliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    log_->debug("8 - Add the new pose to the kdtree execution time: {0}", elapsedMilliseconds);

    return true;
}

g2o::Isometry3 GraphOptimization::matrix4ftoIsometry3(const Eigen::Matrix4f &realMotion) {
    // From homogenuous matrix to isometry
    g2o::Matrix3 Rot;
    Rot << realMotion(0, 0), realMotion(0, 1), realMotion(0, 2),
           realMotion(1, 0), realMotion(1, 1), realMotion(1, 2),
           realMotion(2, 0), realMotion(2, 1), realMotion(2, 2);
    g2o::Vector3 eulerVector = g2o::internal::toEuler(Rot);
    g2o::Vector6 vectorET;
    vectorET << realMotion(0, 3), realMotion(1, 3), realMotion(2, 3), eulerVector;
    g2o::Isometry3 transfMatrixIsometry = g2o::internal::fromVectorET(vectorET);

    return transfMatrixIsometry;
}

Eigen::Matrix4f GraphOptimization::isometry3toMatrix4f(const g2o::Isometry3 &isometry) {
    // From isometry to homogenuous matrix
    Eigen::Matrix4f realMotion;
    realMotion << isometry(0, 0), isometry(0, 1), isometry(0, 2), isometry(0, 3),
                  isometry(1, 0), isometry(1, 1), isometry(1, 2), isometry(1, 3),
                  isometry(2, 0), isometry(2, 1), isometry(2, 2), isometry(2, 3),
                  0, 0, 0, 1;

    return realMotion;
}

boost::optional <Eigen::Matrix4f> GraphOptimization::readVertexPosition(int identifier) {
    g2o::HyperGraph::VertexIDMap container = optimizer_.vertices();
    g2o::VertexSE3 *vert = static_cast<g2o::VertexSE3*> (container[identifier]);

    // From stringstream to position
    std::stringstream is_pos;
    if (!(vert->write(is_pos))) {
        log_->warn("Unable to read the position of the vertex");
        return boost::none;
    }
    std::string str_pos;
    std::vector<double> poses;
    for (int j = 0; j < 7; j++) {
        is_pos >> str_pos;
        poses.push_back(std::stod(str_pos));
    }

    // Generate the matrix of translation and rotation of the k NN i
    g2o::Vector7 vector_poses;
    vector_poses << poses[0], poses[1], poses[2], poses[3],
                    poses[4], poses[5], poses[6];
    g2o::Isometry3 poseIsometry = g2o::internal::fromVectorQT(vector_poses);
    return isometry3toMatrix4f(poseIsometry);
}

}  // namespace graphoptimization
}  // namespace applications
}  // namespace crf
