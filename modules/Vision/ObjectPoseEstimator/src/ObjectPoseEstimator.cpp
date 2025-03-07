/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */
#define PCL_NO_PRECOMPILE
#include "ObjectPoseEstimator/ObjectPoseEstimator.hpp"
#include "RGBDVisionUtility/PCLUtils.hpp"

#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <ctime>
#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/geometry.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/shot_omp.h>

#ifdef BUILD_VISUALIZATION
    #include <pcl/visualization/cloud_viewer.h>
#endif

namespace crf {
namespace applications {
namespace objectposeestimator {

ObjectPoseEstimator::ObjectPoseEstimator(const std::string& configFileName):
        log_("ObjectPoseEstimator"),
        configuration_(),
        best3DBoundingBox_(),
        sourcePointCloud_(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
        currentObjectCluster_(new pcl::PointCloud<pcl::PointXYZRGBNormal>()) {
    log_->debug("CTor");

    bestFitnessScore_ = 0.0;
    bestInverseFitnessScore_ =  0.0;
    bestComposeFitnessScore_  = 0.0;
    bestPose_ = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};

    if (!configuration_.parse(configFileName)) {
        throw std::invalid_argument("Impossible to parse configuration file");
    }
    // Load prescan model
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (configuration_.getModelPath(),
        *sourcePointCloud_) == -1) {
        throw std::invalid_argument("Couldn't read PCD file");
    }
    log_->debug("Configuration parameters initialized");
}

ObjectPoseEstimator::~ObjectPoseEstimator() {
    log_->debug("DTor");
}

boost::optional<PoseEstimationData> ObjectPoseEstimator::computeTargetPose(
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetPointCloud) {
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr clusteredCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr clusteredTransformedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    if (bestComposeFitnessScore_ == 2.0f) {
        log_->warn("Fitness score already 2.0, no reason to compute pose again for this object");
        return boost::none;
    }

    // Check if the input cloud is empty
    if (targetPointCloud->empty()) {
        log_->warn("Impossible to compute target pose with and empty pointcloud");
        return boost::none;
    }

    // Pose of the model coordinate system in the scanned world
    PoseEstimationData computedPose;


    // Provisional cluster extraction until the MaskRCNN is finished
    // After that it will be optional, just in case bounding box is the only segmentation available
    boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>
        boostClusteredCloud = extractCluster(targetPointCloud);
    if (!boostClusteredCloud)
        return boost::none;
    *clusteredCloud = *boostClusteredCloud.get();
    currentObjectCluster_->clear();
    *currentObjectCluster_ = *clusteredCloud;

    // Extract Masscenter pose as a transformation matrix from world coordinate system
    boost::optional<Eigen::Matrix4f> boostMassCenter = estimateMassCenter(clusteredCloud);
    Eigen::Matrix4f massCenter;
    if (!boostMassCenter)
        return boost::none;
    massCenter = boostMassCenter.get();

    // ICP to match prescan model with current object segmented pointcloud
    // Transform to be applied to the model to be aligned with target(scaned cloud)
    Eigen::Matrix4f icpTransform = Eigen::Matrix4f::Identity();
    boost::optional<Eigen::Matrix4f> boostICPTransform = icpAligment(clusteredCloud);
    if (!boostICPTransform)
        return boost::none;

    icpTransform = boostICPTransform.get();
    Eigen::Matrix4f finalPose = icpTransform;
    computedPose.pose = {finalPose(0, 0), finalPose(0, 1), finalPose(0, 2), finalPose(1, 0),
    finalPose(1, 1), finalPose(1, 2), finalPose(2, 0), finalPose(2, 1),
    finalPose(2, 2), finalPose(0, 3), finalPose(1, 3), finalPose(2, 3)};

    // Transform target pointcloud to compute fitness score
    Eigen::Matrix4f clusterPose = finalPose.inverse();
    pcl::transformPointCloudWithNormals(*clusteredCloud, *clusteredTransformedCloud, clusterPose);

    // Check and update best fitness score, best pose and best Bounding Box
    if (configuration_.getNearestNeighborDistance() <= 0) {
        log_->error(
            "Impossible to estimate fitness, NearestNeighborDistance should be greater than 0");
        return boost::none;
    }
    float currentFitnessScore = getFitness(clusteredTransformedCloud,
        configuration_.getNearestNeighborDistance());
    log_->info("currentFitnessScore : {0} ", currentFitnessScore);

    float currentInverseFitnessScore = getInverseFitness(clusteredTransformedCloud,
        configuration_.getNearestNeighborDistance());
    log_->info("currentInverseFitnessScore : {0} ", currentInverseFitnessScore);

    float currentComposeFitnessScore = getComposeFitness(currentFitnessScore,
        currentInverseFitnessScore);
    log_->info("currentComposeFitnessScore : {0} ", currentComposeFitnessScore);

    if (currentComposeFitnessScore > bestComposeFitnessScore_) {
        log_->info("Current pose estimation is better than previous ones");
        bestComposeFitnessScore_ = currentComposeFitnessScore;
        bestFitnessScore_ = currentFitnessScore;
        bestInverseFitnessScore_ = currentInverseFitnessScore;
        bestPose_ = computedPose.pose;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sourcePointCloudTransformed
            (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        pcl::transformPointCloudWithNormals(*sourcePointCloud_, *sourcePointCloudTransformed,
            finalPose);
        boost::optional<Object3DBoundingBox> boostBest3DBoundingBox =
            estimate3DBoundingBox(sourcePointCloudTransformed);
        if (!boostBest3DBoundingBox) {
            log_->error("Impossible to estimate 3D BoundingBox");
            return boost::none;
        }
        best3DBoundingBox_ = boostBest3DBoundingBox.get();
    }

    end = std::chrono::system_clock::now();
    int elapsed_milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    log_->info("Full Target Pose computation elapsed time: {0} millisecs", elapsed_milliseconds);

    return computedPose;
}

float ObjectPoseEstimator::getBestFitnessScore() {
    return bestFitnessScore_;
}

bool ObjectPoseEstimator::clearBestFitnessScore() {
    float bestFitnessScore = 0.0;
    if (bestFitnessScore_ != bestFitnessScore) {
        bestFitnessScore_ = 0.0;
        return true;
    } else {
        log_->warn("Best fitness score already cleared");
        return false;
    }
}

float ObjectPoseEstimator::getBestInverseFitnessScore() {
    return bestInverseFitnessScore_;
}

bool ObjectPoseEstimator::clearBestInverseFitnessScore() {
    float bestInverseFitnessScore = 0.0;
    if (bestInverseFitnessScore_ != bestInverseFitnessScore) {
        bestInverseFitnessScore_ = 0.0;
        return true;
    } else {
        log_->warn("Best Inverse fitness score already cleared");
        return false;
    }
}

float ObjectPoseEstimator::getBestComposeFitnessScore() {
    return bestComposeFitnessScore_;
}

bool ObjectPoseEstimator::clearBestComposeFitnessScore() {
    float bestComposeFitnessScore = 0.0;
    if (bestComposeFitnessScore_ != bestComposeFitnessScore) {
        bestComposeFitnessScore_ = 0.0;
        return true;
    } else {
        log_->warn("Best Compose fitness score already cleared");
        return false;
    }
}

std::array<float, 12> ObjectPoseEstimator::getBestPose() {
    return bestPose_;
}

bool ObjectPoseEstimator::clearBestPose() {
    std::array<float, 12> bestPose = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
    if (bestPose_ != bestPose) {
        bestPose_ = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
        return true;
    } else {
        log_->warn("Best pose already cleared");
        return false;
    }
}

boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>
    ObjectPoseEstimator::getCurrentObjectCluster() {
    if (currentObjectCluster_->empty()) {
        log_->warn("Empty object cluster pointcloud");
        return boost::none;
    }
    return currentObjectCluster_;
}

bool ObjectPoseEstimator::clearCurrentObjectCluster() {
    if (currentObjectCluster_->empty()) {
        log_->error("Unable to clear the pointcloud map");
        return false;
    } else {
        currentObjectCluster_->clear();
    }
    return true;
}

boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>
    ObjectPoseEstimator::getObjectModel() {
    if (sourcePointCloud_->empty()) {
        log_->warn("Empty object cluster pointcloud");
        return boost::none;
    }
        return sourcePointCloud_;
}

Object3DBoundingBox ObjectPoseEstimator::getBestObject3DBoundingBox() {
    return best3DBoundingBox_;
}

bool ObjectPoseEstimator::clearBest3DObjectBoundingBox() {
    Object3DBoundingBox cleared3DBoundingBox;
    // Check if current best bb is different from a cleared one
    if (best3DBoundingBox_.rotationalMatrixOBB == cleared3DBoundingBox.rotationalMatrixOBB) {
        log_->warn("Best bounding box already cleared");
        return false;
    }
    best3DBoundingBox_ = cleared3DBoundingBox;
    return true;
}

bool ObjectPoseEstimator::setICPRefinement(bool icpFlag) {
    if (!configuration_.setICPRefinement(icpFlag))
        return false;
    return true;
}

boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> ObjectPoseEstimator::extractCluster(
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetPointCloud) {
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    log_->debug("Extracting cluster");

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr clusteredCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr subsampledCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    // Downsample pointcloud
    if (!crf::utility::rgbdvisionutility::PCLUtils::subsample(
        targetPointCloud, subsampledCloud, configuration_.getSubsamplingSideLength())) {
        log_->error("Impossible to subsample pointcloud");
        return boost::none;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree->setInputCloud(subsampledCloud);

    // Extract all the possible clusters
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;
    ec.setClusterTolerance(configuration_.getClusterTolerance());
    ec.setMinClusterSize(configuration_.getMinClusterSize());
    ec.setMaxClusterSize(configuration_.getMaxClusterSize());
    ec.setSearchMethod(tree);
    ec.setInputCloud(subsampledCloud);
    ec.extract(clusterIndices);

#ifdef BUILD_VISUALIZATION
    std::unique_ptr<pcl::visualization::PCLVisualizer> viewer(nullptr);
    if (configuration_.getCluserVisualizationValue()) {
        viewer.reset(new pcl::visualization::PCLVisualizer("Clustering Viewer"));
        viewer->setBackgroundColor(0.0, 0.0, 0.5);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal>
                originalColor(targetPointCloud, 255, 0, 0);
        std::string cloudName  = "original cloud";
        viewer->addPointCloud(targetPointCloud, originalColor, cloudName);
        viewer->spin();
        viewer->removeAllPointClouds();
    }
#endif

    // Search for the biggest cluster
    pcl::PointXYZ robotBasePoint(0, 0, 0);
    int j = 0;
    int bestCluster = 0;
    double  massCenterMinDistance = 10.0;

    // Clusters are ordered from biggest to smallest
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin();
        it != clusterIndices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudCluster(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end();
        ++pit) {
          cloudCluster->points.push_back(subsampledCloud->points[*pit]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        boost::optional<Eigen::Matrix4f> boostClusterMassCenter = estimateMassCenter(cloudCluster);
        Eigen::Matrix4f clusterMassCenter;
        if (!boostClusterMassCenter) {
            log_->warn("Unable to compute cluster {0} mass center", j);
            return boost::none;
        }
        clusterMassCenter = boostClusterMassCenter.get();
        pcl::PointXYZ massCenterPoint(clusterMassCenter(0, 3), clusterMassCenter(1, 3),
            clusterMassCenter(2, 3));
        // compute distance between mass center and robot base
        double distance = pcl::geometry::distance(robotBasePoint, massCenterPoint);
        log_->debug("Cluster {0} distance from robot: {1}", j, distance);

        // Best cluster selection
        if (distance < massCenterMinDistance) {
            log_->debug("Cluster {0} size: {1} points", j, cloudCluster->points.size());
            clusteredCloud->clear();
            *clusteredCloud = *cloudCluster;
            massCenterMinDistance = distance;
            bestCluster = j;
        }

#ifdef BUILD_VISUALIZATION
        if (configuration_.getCluserVisualizationValue() && viewer) {
            log_->debug("Adding {0} cluster to viewer", j);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal>
                color(cloudCluster, 0, 255, 0);
            std::string cloudId = "cloud" + std::to_string(j);
            viewer->addPointCloud(cloudCluster, color, cloudId);
            viewer->spin();
        }
#endif
        j++;
    }
    log_->debug("Selected cluster: {0}", bestCluster);

    end = std::chrono::system_clock::now();
    int elapsed_milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    log_->info("Cluster extraction elapsed time: {0} millisecs", elapsed_milliseconds);

    return clusteredCloud;
}

boost::optional<Eigen::Matrix4f> ObjectPoseEstimator::estimateMassCenter(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetPointCloud) {

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    log_->debug("Computing cloud mass center");

    Eigen::Matrix4f massCenterEstimation = Eigen::Matrix4f::Identity();
    std::vector <float> momentOfInertia;
    pcl::PointXYZRGBNormal minPointOBB;
    pcl::PointXYZRGBNormal maxPointOBB;
    pcl::PointXYZRGBNormal positionOBB;
    Eigen::Matrix3f rotationalMatrixOBB = Eigen::Matrix3f::Identity();
    float majorValue, middleValue, minorValue;
    Eigen::Vector3f majorVector, middleVector, minorVector;
    Eigen::Vector3f massCenter;

    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBNormal> featureExtractor;
    featureExtractor.setInputCloud(targetPointCloud);
    featureExtractor.compute();

    if (featureExtractor.getMomentOfInertia(momentOfInertia)) {
        featureExtractor.getOBB(minPointOBB, maxPointOBB, positionOBB, rotationalMatrixOBB);
        // Represent the amount of change of the pointcloud in each eigen vector direction
        featureExtractor.getEigenValues(majorValue, middleValue, minorValue);
        // Compare eigen values with objects aspect ratio in order to know if
        // the extracted pointcloud is similar to the real one
        // Principal axis of change in the pointcloud
        featureExtractor.getEigenVectors(majorVector, middleVector, minorVector);
        featureExtractor.getMassCenter(massCenter);
        massCenterEstimation <<
        rotationalMatrixOBB(0, 0), rotationalMatrixOBB(0, 1), rotationalMatrixOBB(0, 2), massCenter(0), // NOLINT
        rotationalMatrixOBB(1, 0), rotationalMatrixOBB(1, 1), rotationalMatrixOBB(1, 2), massCenter(1), // NOLINT
        rotationalMatrixOBB(2, 0), rotationalMatrixOBB(2, 1), rotationalMatrixOBB(2, 2), massCenter(2), // NOLINT
        0, 0, 0, 1;

#ifdef BUILD_VISUALIZATION
        std::unique_ptr<pcl::visualization::PCLVisualizer> viewer(nullptr);
        if (configuration_.getMomentOfInertiaVisualizationValue() &&
            !configuration_.getCluserVisualizationValue()) {
            viewer.reset(new pcl::visualization::PCLVisualizer("Moment of Inertia Viewer"));
            viewer->setBackgroundColor(0, 0, 0);
            viewer->addCoordinateSystem(0.25);
            viewer->initCameraParameters();
            viewer->addPointCloud<pcl::PointXYZRGBNormal>(targetPointCloud, "target cloud");

            Eigen::Vector3f position(positionOBB.x, positionOBB.y, positionOBB.z);
            Eigen::Quaternionf quat(rotationalMatrixOBB);
            viewer->addCube(position, quat, maxPointOBB.x - minPointOBB.x,
                maxPointOBB.y - minPointOBB.y, maxPointOBB.z - minPointOBB.z, "OBB");
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

            pcl::PointXYZ center(massCenter(0), massCenter(1), massCenter(2));
            pcl::PointXYZ xAxis(majorVector(0) + massCenter(0), majorVector(1) + massCenter(1),
                majorVector(2) + massCenter(2));
            pcl::PointXYZ yAxis(middleVector(0) + massCenter(0), middleVector(1) + massCenter(1),
                middleVector(2) + massCenter(2));
            pcl::PointXYZ zAxis(minorVector(0) + massCenter(0), minorVector(1) + massCenter(1),
                minorVector(2) + massCenter(2));
            viewer->addLine(center, xAxis, 1.0f, 0.0f, 0.0f, "major eigen vector");
            viewer->addLine(center, yAxis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
            viewer->addLine(center, zAxis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

            viewer->spin();
        }
#endif

    } else {
        log_->error("Moment of Inertia estimation without valid values");
        return boost::none;
    }

    end = std::chrono::system_clock::now();
    int elapsed_milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    log_->debug("Cloud mass center computation elapsed time: {0} millisecs", elapsed_milliseconds);

    return massCenterEstimation;
}


boost::optional<Object3DBoundingBox> ObjectPoseEstimator::estimate3DBoundingBox(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetPointCloud) {
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    log_->debug("Computing cloud 3D BoundingBox");

    Object3DBoundingBox object3DBB;

    std::vector <float> momentOfInertia;
    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBNormal> featureExtractor;
    featureExtractor.setInputCloud(targetPointCloud);
    featureExtractor.compute();

    if (featureExtractor.getMomentOfInertia(momentOfInertia)) {
        featureExtractor.getOBB(object3DBB.minPointOBB, object3DBB.maxPointOBB,
            object3DBB.positionOBB, object3DBB.rotationalMatrixOBB);
    } else {
        log_->error("3D Bounding Box estimation without valid values");
        return boost::none;
    }

    end = std::chrono::system_clock::now();
    int elapsed_milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    log_->info("3D BoundingBox computation elapsed time: {0} millisecs", elapsed_milliseconds);

    return object3DBB;
}

boost::optional<Eigen::Matrix4f> ObjectPoseEstimator::icpAligment(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &tgtFullCloud) {
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    log_->debug("ICP Aligment");

    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals2(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals2(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr srcPointCloud2(
        new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tgtPointCloud2(
        new pcl::PointCloud<pcl::PointXYZRGBA>());

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sourceFullCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    *sourceFullCloud = *sourcePointCloud_;

    Eigen::Matrix4d finalTransform(Eigen::Matrix4d::Identity());

    // Create separate clouds from target full cloud
    for (int i = 0; i < static_cast<int>(tgtFullCloud->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = tgtFullCloud->points[i].normal_x;
            normal.normal_y = tgtFullCloud->points[i].normal_y;
            normal.normal_z = tgtFullCloud->points[i].normal_z;
            targetNormals2->push_back(normal);

            pcl::PointXYZRGBA point;
            point.x = tgtFullCloud->points[i].x;
            point.y = tgtFullCloud->points[i].y;
            point.z = tgtFullCloud->points[i].z;
            tgtPointCloud2->push_back(point);
    }

    // Create separate clouds from source full cloud
    for (int i = 0; i < static_cast<int>(sourceFullCloud->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = sourceFullCloud->points[i].normal_x;
            normal.normal_y = sourceFullCloud->points[i].normal_y;
            normal.normal_z = sourceFullCloud->points[i].normal_z;
            sourceNormals2->push_back(normal);

            pcl::PointXYZRGBA point;
            point.x = sourceFullCloud->points[i].x;
            point.y = sourceFullCloud->points[i].y;
            point.z = sourceFullCloud->points[i].z;
            srcPointCloud2->push_back(point);
    }

    boost::optional<std::vector< int >> boostSrcRemovedIndices;
    boost::optional<std::vector< int >> boostTgtRemovedIndices;
    std::vector< int > srcRemovedIndices;
    std::vector< int > tgtRemovedIndices;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        srcPointCloud3 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        tgtPointCloud3 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals3 (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals3 (new pcl::PointCloud<pcl::Normal>);


    if (configuration_.getCovarianceSubsamplingValue()) {
        // Covariance subsampling
        boostSrcRemovedIndices = crf::utility::rgbdvisionutility::PCLUtils::covarianceSubsampling(
            srcPointCloud2, sourceNormals2,
            configuration_.getCovarianceSamplingTotalPointsDivision());
        if (!boostSrcRemovedIndices)
            return boost::none;
        srcRemovedIndices = boostSrcRemovedIndices.get();
        boostTgtRemovedIndices = crf::utility::rgbdvisionutility::PCLUtils::covarianceSubsampling(
            tgtPointCloud2, targetNormals2,
            configuration_.getCovarianceSamplingTotalPointsDivision());
        if (!boostTgtRemovedIndices)
            return boost::none;
        tgtRemovedIndices = boostTgtRemovedIndices.get();
    } else {
        // Normals subsampling
        boostSrcRemovedIndices = crf::utility::rgbdvisionutility::PCLUtils::normalsSubsampling(
            srcPointCloud2, sourceNormals2, configuration_.getNormalSamplingTotalPointsDivision(),
            configuration_.getNormalSamplingBinSize());
        if (!boostSrcRemovedIndices)
            return boost::none;
        srcRemovedIndices = boostSrcRemovedIndices.get();
        boostTgtRemovedIndices = crf::utility::rgbdvisionutility::PCLUtils::normalsSubsampling(
            tgtPointCloud2, targetNormals2, configuration_.getNormalSamplingTotalPointsDivision(),
            configuration_.getNormalSamplingBinSize());
        if (!boostTgtRemovedIndices)
            return boost::none;
        tgtRemovedIndices = boostTgtRemovedIndices.get();
    }

    // Actual pointclouds and normals subsampling with the removed indices
    for (int i = 0; i < static_cast<int>(srcRemovedIndices.size()); i++) {
            srcPointCloud3->push_back(srcPointCloud2->points[srcRemovedIndices[i]]);
            sourceNormals3->push_back(sourceNormals2->points[srcRemovedIndices[i]]);
    }
    for (int i = 0; i < static_cast<int>(sourceNormals3->points.size()); i++) {
      if (!pcl::isFinite<pcl::Normal>(sourceNormals3->points[i])) {
        PCL_WARN("normals[%d] is not finite\n", i);
      }
    }
    log_->debug("Source sampled pointcloud size: {0}", srcPointCloud3->size());
    for (int i = 0; i < static_cast<int>(tgtRemovedIndices.size()); i++) {
            tgtPointCloud3->push_back(tgtPointCloud2->points[tgtRemovedIndices[i]]);
            targetNormals3->push_back(targetNormals2->points[tgtRemovedIndices[i]]);
    }
    for (int i = 0; i < static_cast<int>(targetNormals3->points.size()); i++) {
      if (!pcl::isFinite<pcl::Normal>(targetNormals3->points[i])) {
        PCL_WARN("normals[%d] is not finite\n", i);
      }
    }
    log_->debug("Target sampled pointcloud size: {0}", tgtPointCloud3->size());

#ifdef BUILD_VISUALIZATION
    if (configuration_.getIcpSourceNormalsVisualizationValue()) {
        pcl::visualization::PCLVisualizer viewerNormals("PCL Source Normals Viewer");
        viewerNormals.setBackgroundColor(0.0, 0.0, 0.5);
        viewerNormals.addPointCloud(tgtPointCloud3, "src cloud");
        viewerNormals.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>
            (tgtPointCloud3, targetNormals3, 1);

        while (!viewerNormals.wasStopped()) {
          viewerNormals.spinOnce();
        }
    }
#endif

    // ICP Initial Aligment using 3D descriptors
    boost::optional<Eigen::Matrix4f> boostInitialTransform = initialAligment(sourceFullCloud,
         tgtFullCloud);
    if (boostInitialTransform == boost::none)
        return boost::none;
    Eigen::Matrix4f initialTransform = boostInitialTransform.get();
    finalTransform = initialTransform.cast <double> ();

    // Icp optimization if selected in configuration file
    if (configuration_.getIcpRefinementValue()) {
        log_->info("ICP Refinement");
        // Transform estimation
        int iteration = 0;
        Eigen::Matrix4d estimatedTransform(Eigen::Matrix4d::Identity());
        pcl::CorrespondencesPtr correspondences (new pcl::Correspondences ());
        pcl::registration::DefaultConvergenceCriteria<double>
            convergenceCriteria(iteration, estimatedTransform, *correspondences);
        if (configuration_.getTranslationThreshold() <=0 ||
            configuration_.getRotationThreshold() <=0 ||
            configuration_.getMaximumIterations() <=0) {
            log_->error("Translation, rotation and maximumIterations should be greater than 0");
            return boost::none;
        }
        convergenceCriteria.setTranslationThreshold(configuration_.getTranslationThreshold());
        float fthres = cos(configuration_.getRotationThreshold());
        convergenceCriteria.setRotationThreshold(fthres);  // rad
        convergenceCriteria.setMaximumIterations(configuration_.getMaximumIterations());

        pcl::PointCloud<pcl::PointNormal>::Ptr
            source_normalsAligned(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
            srcPointCloud3_aligned (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sourceXYZ (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr targetXYZ (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointNormal>::Ptr
            sourcePointNormals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr
            targetPointNormals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::copyPointCloud(*srcPointCloud3, *sourceXYZ);
        pcl::copyPointCloud(*tgtPointCloud3, *targetXYZ);
        pcl::concatenateFields(*sourceXYZ, *sourceNormals3, *sourcePointNormals);
        pcl::concatenateFields(*targetXYZ, *targetNormals3, *targetPointNormals);

#ifdef BUILD_VISUALIZATION
        std::unique_ptr<pcl::visualization::PCLVisualizer> viewer(nullptr);
        if (configuration_.getIcpStepsVisualizationValue()) {
            viewer.reset(new pcl::visualization::PCLVisualizer("PCL ICP Viewer"));
            viewer->setBackgroundColor(0.0, 0.0, 0.5);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>
                firstColor(srcPointCloud3, 0, 255, 0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>
                secondColor(tgtPointCloud3, 255, 0, 0);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
                srcPointCloud3Aux (new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::transformPointCloud(*srcPointCloud3, *srcPointCloud3Aux, finalTransform);
            viewer->addPointCloud(srcPointCloud3Aux, firstColor, "src cloud");
            viewer->addPointCloud(tgtPointCloud3, secondColor, "tgt cloud");
            viewer->spin();
            viewer->removeAllPointClouds();
        }
#endif

        do {
            pcl::transformPointCloud(*srcPointCloud3, *srcPointCloud3_aligned, finalTransform);
            pcl::transformPointCloudWithNormals(*sourcePointNormals,
                *source_normalsAligned, finalTransform);

            boost::optional<pcl::CorrespondencesPtr> boostCorrespondences =
                crf::utility::rgbdvisionutility::PCLUtils::correspondenceEstimation(
                srcPointCloud3_aligned, tgtPointCloud3,
                configuration_.getCorrespondencesMaximumDistance());
            if (!boostCorrespondences)
                return boost::none;
            *correspondences = *boostCorrespondences.get();

            pcl::CorrespondencesPtr remainingCorrespondences(new pcl::Correspondences ());
            boost::optional<pcl::CorrespondencesPtr> boostRemainingCorrespondences =
                crf::utility::rgbdvisionutility::PCLUtils::medianDistanceCorrespondeceRejection(
                correspondences, configuration_.getCorrespondencesMaximumMedianDistanceFactor());
            if (!boostRemainingCorrespondences)
                return boost::none;
            *remainingCorrespondences = *boostRemainingCorrespondences.get();

            pcl::CorrespondencesPtr remainingCorrespondences2 (new pcl::Correspondences ());
            boost::optional<pcl::CorrespondencesPtr> boostRemainingCorrespondences2 =
                crf::utility::rgbdvisionutility::PCLUtils::surfaceNormalCorrespondeceRejection(
                remainingCorrespondences, srcPointCloud3_aligned, tgtPointCloud3,
                source_normalsAligned, targetPointNormals,
                configuration_.getCorrespondencesMaximunAngleInRad());
            if (!boostRemainingCorrespondences2)
                return boost::none;
            *remainingCorrespondences2 = *boostRemainingCorrespondences2.get();

#ifdef BUILD_VISUALIZATION
            if (configuration_.getIcpStepsVisualizationValue() && viewer) {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>
                    firstColor(srcPointCloud3_aligned, 0, 255, 0);
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>
                    secondColor(tgtPointCloud3, 255, 0, 0);
                viewer->addPointCloud(srcPointCloud3_aligned, firstColor, "src cloud");
                viewer->addPointCloud(tgtPointCloud3, secondColor, "tgt cloud");
                viewer->addCorrespondences<pcl::PointXYZRGBA> (srcPointCloud3_aligned,
                    tgtPointCloud3, *remainingCorrespondences2, "correspondences");
                viewer->spin();
                viewer->removeAllPointClouds();
                viewer->removeCorrespondences("correspondences");
            }
#endif

            estimatedTransform = crf::utility::rgbdvisionutility::PCLUtils::transformEstimation(
                remainingCorrespondences2, srcPointCloud3_aligned, source_normalsAligned,
                targetPointNormals, configuration_.getWeightedTransformEstimationValue(),
                configuration_.getPassThroughMinLimit());
            // Update variables
            iteration++;
            finalTransform = estimatedTransform * finalTransform;
        } while (!convergenceCriteria.hasConverged());

        log_->debug("Convergence state {0}", convergenceCriteria.getConvergenceState());
        log_->debug("Number of iteration needed: {0}", iteration);
    }

    log_->debug("Final estimated transform: \n {0}", finalTransform);

    Eigen::Matrix4f lastIcpTransform(Eigen::Matrix4f::Identity());
    lastIcpTransform = finalTransform.cast <float> ();

    end = std::chrono::system_clock::now();
    int elapsed_milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    log_->info("Full Aligment elapsed time: {0} millisecs", elapsed_milliseconds);

    return lastIcpTransform;
}

boost::optional<Eigen::Matrix4f> ObjectPoseEstimator::initialAligment(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr srcPointCloud,
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tgtPointCloud) {

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    log_->debug("Initial Aligment");
    Eigen::Matrix4f initialTransform(Eigen::Matrix4f::Identity());

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr subsampledSrcPointCloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr subsampledTgtPointCloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    // Downsample pointcloud
    if (!crf::utility::rgbdvisionutility::PCLUtils::subsample(
        srcPointCloud, subsampledSrcPointCloud, configuration_.getSubsamplingSideLength())) {
        log_->error("Impossible to subsample pointcloud");
        return boost::none;
    }
    if (!crf::utility::rgbdvisionutility::PCLUtils::subsample(
        tgtPointCloud, subsampledTgtPointCloud, configuration_.getSubsamplingSideLength())) {
        log_->error("Impossible to subsample pointcloud");
        return boost::none;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr sourceCloudNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetCloudNormals(new pcl::PointCloud<pcl::Normal>);
    // Split full clouds into xyz and normals
    for (int i = 0; i < static_cast<int>(subsampledSrcPointCloud->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = subsampledSrcPointCloud->points[i].normal_x;
            normal.normal_y = subsampledSrcPointCloud->points[i].normal_y;
            normal.normal_z = subsampledSrcPointCloud->points[i].normal_z;
            sourceCloudNormals->push_back(normal);

            pcl::PointXYZ point;
            point.x = subsampledSrcPointCloud->points[i].x;
            point.y = subsampledSrcPointCloud->points[i].y;
            point.z = subsampledSrcPointCloud->points[i].z;
            sourceCloud2->push_back(point);
    }
    for (int i = 0; i < static_cast<int>(subsampledTgtPointCloud->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = subsampledTgtPointCloud->points[i].normal_x;
            normal.normal_y = subsampledTgtPointCloud->points[i].normal_y;
            normal.normal_z = subsampledTgtPointCloud->points[i].normal_z;
            targetCloudNormals->push_back(normal);

            pcl::PointXYZ point;
            point.x = subsampledTgtPointCloud->points[i].x;
            point.y = subsampledTgtPointCloud->points[i].y;
            point.z = subsampledTgtPointCloud->points[i].z;
            targetCloud2->push_back(point);
    }

    // Keypoints estimation and descriptors
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences ());

    if (configuration_.getSHOTColorFeaturesValue()) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourceKeypointsColor(
            new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetKeypointsColor(
            new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::SHOT1344>::Ptr
        sourceFeatures(new pcl::PointCloud<pcl::SHOT1344>);
        pcl::PointCloud<pcl::SHOT1344>::Ptr
            targetFeatures(new pcl::PointCloud<pcl::SHOT1344>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourceCloudColor(
            new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetCloudColor(
            new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::copyPointCloud(*subsampledSrcPointCloud, *sourceCloudColor);
        pcl::copyPointCloud(*subsampledTgtPointCloud, *targetCloudColor);

        if (!crf::utility::rgbdvisionutility::PCLUtils::computeShotColorKeypoints(sourceCloudColor,
            sourceCloudNormals, sourceFeatures, sourceKeypointsColor,
            configuration_.getSHOTColormultiscaleFeaturePersistenceScaleValue1(),
            configuration_.getSHOTColormultiscaleFeaturePersistenceScaleValue2(),
            configuration_.getSHOTColormultiscaleFeaturePersistenceScaleValue3(),
            configuration_.getSHOTColorPersintenceAlpha(),
            configuration_.getSHOTComputationThreads())) {
            log_->error("Impossible to compute SHOTColor Keypoints");
            return boost::none;
        }
        if (!crf::utility::rgbdvisionutility::PCLUtils::computeShotColorKeypoints(targetCloudColor,
            targetCloudNormals, targetFeatures, targetKeypointsColor,
            configuration_.getSHOTColormultiscaleFeaturePersistenceScaleValue1(),
            configuration_.getSHOTColormultiscaleFeaturePersistenceScaleValue2(),
            configuration_.getSHOTColormultiscaleFeaturePersistenceScaleValue3(),
            configuration_.getSHOTColorPersintenceAlpha(),
            configuration_.getSHOTComputationThreads())) {
            log_->error("Impossible to compute SHOTColor Keypoints");
            return boost::none;
        }

        targetCloudColor->sensor_origin_.head<3>().fill(0);
        targetCloudColor->sensor_orientation_.setIdentity();
        sourceCloudColor->sensor_origin_.head<3>().fill(0);
        sourceCloudColor->sensor_orientation_.setIdentity();
        targetKeypointsColor->sensor_origin_.head<3>().fill(0);
        targetKeypointsColor->sensor_orientation_.setIdentity();
        sourceKeypointsColor->sensor_origin_.head<3>().fill(0);
        sourceKeypointsColor->sensor_orientation_.setIdentity();

        pcl::copyPointCloud(*sourceKeypointsColor, *sourceKeypoints);
        pcl::copyPointCloud(*targetKeypointsColor, *targetKeypoints);

        *correspondences = *crf::utility::rgbdvisionutility::PCLUtils::correspondenceEstimationShotColor(sourceFeatures, targetFeatures); //NOLINT

    } else {
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr
        sourceFeatures(new pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr
            targetFeatures(new pcl::PointCloud<pcl::FPFHSignature33>);

        if (!crf::utility::rgbdvisionutility::PCLUtils::computeFPFHKeypoints(sourceCloud2,
            sourceCloudNormals, sourceFeatures, sourceKeypoints,
            configuration_.getMultiscaleFeaturePersistenceScaleValue1(),
            configuration_.getMultiscaleFeaturePersistenceScaleValue2(),
            configuration_.getMultiscaleFeaturePersistenceScaleValue3(),
            configuration_.getFPFHPersintenceAlpha())) {
            log_->error("Impossible to compute FPFH Keypoints");
            return boost::none;
        }
        if (!crf::utility::rgbdvisionutility::PCLUtils::computeFPFHKeypoints(targetCloud2,
            targetCloudNormals, targetFeatures, targetKeypoints,
            configuration_.getMultiscaleFeaturePersistenceScaleValue1(),
            configuration_.getMultiscaleFeaturePersistenceScaleValue2(),
            configuration_.getMultiscaleFeaturePersistenceScaleValue3(),
            configuration_.getFPFHPersintenceAlpha())) {
            log_->error("Impossible to compute FPFH Keypoints");
            return boost::none;
        }

        targetCloud2->sensor_origin_.head<3>().fill(0);
        targetCloud2->sensor_orientation_.setIdentity();
        sourceCloud2->sensor_origin_.head<3>().fill(0);
        sourceCloud2->sensor_orientation_.setIdentity();
        targetKeypoints->sensor_origin_.head<3>().fill(0);
        targetKeypoints->sensor_orientation_.setIdentity();
        sourceKeypoints->sensor_origin_.head<3>().fill(0);
        sourceKeypoints->sensor_orientation_.setIdentity();

        *correspondences = *crf::utility::rgbdvisionutility::PCLUtils::correspondenceEStimationFPFHSignature33(sourceFeatures, targetFeatures); //NOLINT
    }

    pcl::CorrespondencesPtr remainingCorrespondences (new pcl::Correspondences());
    boost::optional<pcl::CorrespondencesPtr> boostRemainingCorrespondences =
        crf::utility::rgbdvisionutility::PCLUtils::sampleConsensusCorrespondeceRejection(
        correspondences, sourceKeypoints, targetKeypoints,
        configuration_.getSampleConsensusInlierThreshold(),
        configuration_.getSampleConsensusMaximumIterations(),
        configuration_.getSampleConsensusRefineModelValue());
    if (!boostRemainingCorrespondences)
        return boost::none;
    *remainingCorrespondences = *boostRemainingCorrespondences.get();

#ifdef BUILD_VISUALIZATION
    if (configuration_.getIcpInitialAligmentVisualizationValue()) {
        pcl::visualization::PCLVisualizer *viewer;
        viewer = new pcl::visualization::PCLVisualizer ("PCL Initial Aligment Viewer");
        viewer->setBackgroundColor(0.0, 0.0, 0.5);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            firstColor(sourceKeypoints, 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            secondColor(targetKeypoints, 255, 0, 0);
        viewer->addPointCloud(sourceKeypoints, firstColor, "src cloud");
        viewer->addPointCloud(targetKeypoints, secondColor, "tgt cloud");
        viewer->addCorrespondences<pcl::PointXYZ> (sourceKeypoints,
            targetKeypoints, *remainingCorrespondences, "correspondences");
        viewer->spin();
        viewer->removeAllPointClouds();
        viewer->removeCorrespondences("correspondences");
    }
#endif

    // Transformation estimation
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,
        pcl::PointXYZ> transformationEstimation;
    transformationEstimation.estimateRigidTransformation(*sourceKeypoints, *targetKeypoints,
        *remainingCorrespondences, initialTransform);
    log_->debug("Initial estimated transform: \n {0}", initialTransform);


    end = std::chrono::system_clock::now();
    int elapsed_milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    log_->info("ICP Initial Aligment elapsed time: {0} millisecs", elapsed_milliseconds);

    return initialTransform;
}

float ObjectPoseEstimator::getFitness(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &targetPointCloud, float inlierTreshold) {
    const float maxRange = inlierTreshold * inlierTreshold;
    pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>);
    tree->setInputCloud(targetPointCloud);

    // For each point in the source find its nearest point in the target cloud
    int inTresholdCount = 0;
    size_t size = sourcePointCloud_->points.size();
    for (size_t i = 0; i < size ; ++i) {
        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);
        tree->nearestKSearch(sourcePointCloud_->points[i], 1, nn_indices, nn_dists);
        if (nn_dists[0] < inlierTreshold) {
          inTresholdCount++;
        }
    }

    return static_cast<float>(inTresholdCount) / size;
}

float ObjectPoseEstimator::getInverseFitness(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &targetPointCloud, float inlierTreshold) {
    const float maxRange = inlierTreshold * inlierTreshold;
    pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>);
    tree->setInputCloud(sourcePointCloud_);

    // For each point in the source find its nearest point in the target cloud
    int inTresholdCount = 0;
    size_t size = targetPointCloud->points.size();
    for (size_t i = 0; i < size ; ++i) {
        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);
        tree->nearestKSearch(targetPointCloud->points[i], 1, nn_indices, nn_dists);
        if (nn_dists[0] < inlierTreshold) {
          inTresholdCount++;
        }
    }

    return static_cast<float>(inTresholdCount) / size;
}


float ObjectPoseEstimator::getComposeFitness(float fitnessScore, float inverseFitnessScore) {
    float composeFitnessScore = 0.0f;
    composeFitnessScore = (fitnessScore*fitnessScore) + (inverseFitnessScore*inverseFitnessScore);
    return composeFitnessScore;
}



}  // namespace objectposeestimator
}  // namespace applications
}  // namespace crf
