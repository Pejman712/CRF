#pragma once

/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include "GraphOptimization/IGraphOptimization.hpp"
#include "GraphOptimization/GraphOptimizationConfiguration.hpp"

#include "EventLogger/EventLogger.hpp"
#include "Mapper3d/Mapper3d.hpp"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/eigen_types.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <boost/optional.hpp>
#include <string>
#include <vector>
#include <memory>

using crf::applications::graphoptimization::GraphOptimizationConfiguration;
using crf::applications::graphoptimization::NeighboursSearch;
using crf::applications::graphoptimization::Solvers;
using crf::applications::graphoptimization::Simulations;
using crf::applications::graphoptimization::SlamParameters;

namespace crf {
namespace applications {
namespace graphoptimization {

class GraphOptimization final: public IGraphOptimization {
 public:
    explicit GraphOptimization(const std::string& mapperConfigFile);
    ~GraphOptimization() override;
    bool parse(const std::string& configFileName) override;
    bool addVertex(
        const Eigen::Matrix4f &realMotion,
        const Eigen::Matrix4f &previousPosition,
        boost::optional <const Eigen::Vector3f> boostFixPose,
        boost::optional <const Eigen::Vector3f> odometryPosition,
        boost::optional <const Eigen::Vector3f> groundTruthPosition) override;
    boost::optional<Eigen::Matrix4f> comparePointClouds(
        const Eigen::Matrix4f &sourceSensorOrigin,
        const Eigen::Matrix4f &targetSensorOrigin,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &sourcePointCloud,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &targetPointCloud) override;
    boost::optional <std::vector <pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>>
        getNodePointclouds() override;
    boost::optional <std::vector <Eigen::Matrix4f, Eigen::aligned_allocator <Eigen::Matrix4f>>>
        getNodePositions() override;

    bool resetGraph() override;
    /* 
     * Returns:
     *  - True if the graph script has been successfully saved
     *  - False if there has been a problem
     */
    bool saveGraph(const std::string& filename);
    /* 
     * Returns:
     *  - True if the graph results (2D plot) has been successfully saved
     *  - False if there has been a problem
     */
    bool saveResults(const std::string& filename);
    /* 
     * Returns:
     *  - Data information about the configFile
     */
    boost::optional <NeighboursSearch> getNeighbourgsSearchData();
    boost::optional <Solvers> getSolversData();
    boost::optional <Simulations> getSimulationsData();
    boost::optional <SlamParameters> getSlamData();
    boost::optional <Eigen::Matrix4f> getTransformationData();
    boost::optional <Eigen::Matrix4f> getInitialPositionData();
    boost::optional <std::vector <std::vector <float>>> getDoorsPosition();

 private:
    crf::utility::logger::EventLogger log_;

    crf::applications::mapper3d::Mapper3d map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr kdtreeCloud_;

    std::vector <g2o::Isometry3, Eigen::aligned_allocator <g2o::Isometry3>> relationKnn_;
    std::vector <pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> nodePointClouds_;
    std::vector <pcl::PointCloud<pcl::Normal>::Ptr> nodePointCloudsNormal_;
    g2o::SparseOptimizer optimizer_;
    std::vector<int> pointsSearch_;
    std::vector<bool> correctComparison_;
    std::vector<float> fitnessScoreNeighbours_;
    std::vector<float> crudePositionsX;
    std::vector<float> crudePositionsY;
    std::vector<float> groundTruthX;
    std::vector<float> groundTruthY;
    std::vector<float> odometryX;
    std::vector<float> odometryY;

    // Configuration variables
    unsigned int knnMax_;
    bool closestSearch_;
    float radius_;
    int iterations_;
    bool robustKernel_;

    // global variables
    bool appliedParse_;
    unsigned int idVertex_;
    unsigned int Knn_;
    bool organizedPointCloud_;
    float fitnessScore_;

    std::shared_ptr<GraphOptimizationConfiguration> graphConfig_;

    // Functions
    // Point 5: Calculation of the knn
    bool determineKNN(const Eigen::Vector3f &pointPose);
    // Point 6: Pointcloud comparison with the knn
    bool knnComparison(const Eigen::Matrix4f &realMotion);
    // Point 7: Add edges to the graph
    bool addEdges(const g2o::Isometry3 &RealMotion_);
    // Point 8: Graph optimization (pose optimization)
    bool graphOptimize();
    // Point 9: kdTree generation
    bool updateKdTree();
    // Utilities
    g2o::Isometry3 matrix4ftoIsometry3(const Eigen::Matrix4f &realMotion);
    Eigen::Matrix4f isometry3toMatrix4f(const g2o::Isometry3 &isometry);
    boost::optional <Eigen::Matrix4f> readVertexPosition(int identifier);
};

}  // namespace graphoptimization
}  // namespace applications
}  // namespace crf
