/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include "Mapper3d/HandHeldMapper3d.hpp"
#include "Mapper3d/NormalsColorOctree.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <fstream>
#include <string>
#include <math.h>
#include <limits>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/normal_space.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <pcl/filters/random_sample.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/normal_3d.h>
#include <octomap/octomap.h>
#include <Eigen/Dense>
#include <boost/optional.hpp>
#include <stdio.h>
#include <sys/time.h>
#include <memory>
#include <array>

namespace crf {
namespace applications {
namespace mapper3d {

HandHeldMapper3d::HandHeldMapper3d(const std::string& configFileName):log_("HandHeldMapper3d"),
        accummulatedTransform_{},
        currentCloudNormals_(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
        fullCloudNormals_(new pcl::PointCloud<pcl::PointXYZRGBNormal>()) {
    log_->debug("CTor");
    // Read configuration names from .json
    std::ifstream config(configFileName);
    nlohmann::json jConfig;
    config >> jConfig;

    maxNumberOfPoints_ = jConfig.at("pointCloudParameters").at("maxNumberOfPoints").get<int>();
    octreeResolutionInMeters_ =
        jConfig.at("octreeParameters").at("octreeResolutionInMeters").get<float>();
    updateOctree_  = jConfig.at("octreeParameters").at("updateOctree").get<bool>();
    maxNumberOfNodes_ = jConfig.at("octreeParameters").at("maxNumberOfNodes").get<int>();
    enableChangeDetection_ =
     jConfig.at("octreeParameters").at("enableChangeDetection").get<bool>();
    probabilityHit_ =
        jConfig.at("octreeParameters").at("probabilityHit").get<float>();
    probabilityMiss_ =
        jConfig.at("octreeParameters").at("probabilityMiss").get<float>();
    clampingThresholdMin_ =
        jConfig.at("octreeParameters").at("clampingThresholdMin").get<float>();
    clampingThresholdMax_ =
        jConfig.at("octreeParameters").at("clampingThresholdMax").get<float>();
    occupancyThreshold_ =
        jConfig.at("octreeParameters").at("occupancyThreshold").get<float>();
    fastBilateral_  = jConfig.at("filterSelection").at("fastBilateral").get<bool>();
    passThrough_  = jConfig.at("filterSelection").at("passThrough").get<bool>();
    voxelGrid_  = jConfig.at("filterSelection").at("voxelGrid").get<bool>();
    statisticalOutlier_ = jConfig.at("filterSelection").at("statisticalOutlier").get<bool>();
    mls_ = jConfig.at("filterSelection").at("mls").get<bool>();
    fastBilateralSigmaR_ = jConfig.at("filterParameters").at("fastBilateralSigmaR").get<float>();
    fastBilateralSigmaS_ = jConfig.at("filterParameters").at("fastBilateralSigmaS").get<float>();
    passThroughMinLimit_ = jConfig.at("filterParameters").at("passThroughMinLimit").get<float>();
    if (passThroughMinLimit_ < 0) {
         throw std::invalid_argument("PassThrough minimum limit should be greater or equal to 0");
    }
    passThroughMaxLimit_ = jConfig.at("filterParameters").at("passThroughMaxLimit").get<float>();
    voxelGridLeafSizeX_ = jConfig.at("filterParameters").at("voxelGridLeafSizeX").get<float>();
    voxelGridLeafSizeY_ = jConfig.at("filterParameters").at("voxelGridLeafSizeY").get<float>();
    voxelGridLeafSizeZ_ = jConfig.at("filterParameters").at("voxelGridLeafSizeZ").get<float>();
    statisticalOutlierMeanK_ =
        jConfig.at("filterParameters").at("statisticalOutlierMeanK").get<int>();
    statisticalOutlierStddevMulThresh_ =
        jConfig.at("filterParameters").at("statisticalOutlierStddevMulThresh").get<float>();
    mlsPolynomialOrder_ = jConfig.at("filterParameters").at("mlsPolynomialOrder").get<int>();
    mlsSearchRadius_ = jConfig.at("filterParameters").at("mlsSearchRadius").get<float>();
    icpStepsVisualization_ = jConfig.at("icpParameters").at("icpStepsVisualization").get<bool>();
    icpSourceNormalsVisualization_ =
        jConfig.at("icpParameters").at("icpSourceNormalsVisualization").get<bool>();
    weightedTransformEstimation_ =
        jConfig.at("icpParameters").at("weightedTransformEstimation").get<bool>();
    randomSampling_ = jConfig.at("icpParameters").at("randomSampling").get<bool>();
    normalMaxDepthChangeFactor_ =
        jConfig.at("icpParameters").at("normalMaxDepthChangeFactor").get<float>();
    normalSmoothingSize_ = jConfig.at("icpParameters").at("normalSmoothingSize").get<float>();
    normalSamplingTotalPointsDivision_ =
        jConfig.at("icpParameters").at("normalSamplingTotalPointsDivision").get<int>();
    randomlSamplingTotalPointsDivision_ =
        jConfig.at("icpParameters").at("randomlSamplingTotalPointsDivision").get<int>();
    normalSamplingBinSize_ = jConfig.at("icpParameters").at("normalSamplingBinSize").get<int>();
    correspondencesMaximumDistance_ =
        jConfig.at("icpParameters").at("correspondencesMaximumDistance").get<float>();
    correspondencesMaximumMedianDistanceFactor_ =
        jConfig.at("icpParameters").at("correspondencesMaximumMedianDistanceFactor").get<float>();
    correspondencesMaximunAngleInRad_ =
        jConfig.at("icpParameters").at("correspondencesMaximunAngleInRad").get<float>();
    translationThreshold_ = jConfig.at("icpParameters").at("translationThreshold").get<double>();
    rotationThreshold_ = jConfig.at("icpParameters").at("rotationThreshold").get<double>();
    maximumIterations_ = jConfig.at("icpParameters").at("maximumIterations").get<int>();
    icpInitialAlignment_ =
        jConfig.at("initialAlignmentParameters").at("icpInitialAlignment").get<bool>();
    icpInitialAligmentVisualization_ =
        jConfig.at("initialAlignmentParameters").at("icpInitialAligmentVisualization").get<bool>();
    sampleConsensusInlierThreshold_ =
        jConfig.at("initialAlignmentParameters").at("sampleConsensusInlierThreshold").get<float>();
    sampleConsensusMaximumIterations_ =
        jConfig.at("initialAlignmentParameters").at("sampleConsensusMaximumIterations").get<int>();
    sampleConsensusRefineModel_ =
        jConfig.at("initialAlignmentParameters").at("sampleConsensusRefineModel").get<bool>();
    subsamplingSideLength_ =
        jConfig.at("initialAlignmentParameters").at("subsamplingSideLength").get<float>();
    normalEstimationSearchRadius_ =
        jConfig.at("initialAlignmentParameters").at("normalEstimationSearchRadius").get<float>();
    FPFHPersintenceAlpha_ =
        jConfig.at("initialAlignmentParameters").at("FPFHPersintenceAlpha").get<float>();
    multiscaleFeaturePersistenceScaleValue1_ =
        jConfig.at("initialAlignmentParameters").
        at("multiscaleFeaturePersistenceScaleValue1").get<float>();
    multiscaleFeaturePersistenceScaleValue2_ =
        jConfig.at("initialAlignmentParameters").
        at("multiscaleFeaturePersistenceScaleValue2").get<float>();
    multiscaleFeaturePersistenceScaleValue3_ =
        jConfig.at("initialAlignmentParameters").
        at("multiscaleFeaturePersistenceScaleValue3").get<float>();

    if (octreeResolutionInMeters_ <= 0) {
         throw std::invalid_argument("Octree resolution should be greater than 0");
    }

    fullNormalsColorOctree_.reset(new NormalsColorOctree(octreeResolutionInMeters_));
    fullNormalsColorOctree_->setProbHit(probabilityHit_);
    fullNormalsColorOctree_->setProbMiss(probabilityMiss_);
    fullNormalsColorOctree_->setClampingThresMin(clampingThresholdMin_);
    fullNormalsColorOctree_->setClampingThresMax(clampingThresholdMax_);
    fullNormalsColorOctree_->setOccupancyThres(occupancyThreshold_);
    fullNormalsColorOctree_->enableChangeDetection(enableChangeDetection_);

    // Set accummulatedTransform_ to identity
    accummulatedTransform_[0] = 1.0;
    accummulatedTransform_[1] = 0.0;
    accummulatedTransform_[2] = 0.0;
    accummulatedTransform_[3] = 0.0;
    accummulatedTransform_[4] = 1.0;
    accummulatedTransform_[5] = 0.0;
    accummulatedTransform_[6] = 0.0;
    accummulatedTransform_[7] = 0.0;
    accummulatedTransform_[8] = 1.0;
    accummulatedTransform_[9] = 0.0;
    accummulatedTransform_[10] = 0.0;
    accummulatedTransform_[11] = 0.0;

    log_->debug("Configuration parameters initialized");
}

HandHeldMapper3d::~HandHeldMapper3d() {
    log_->debug("DTor");
}

boost::optional<Eigen::Matrix4f> HandHeldMapper3d::updateMap(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud,
    const Eigen::Matrix4f &sensorOrigin) {

    struct timeval start, end;
    int64_t mtime, seconds, useconds;
    gettimeofday(&start, NULL);

    Eigen::Matrix4f finalCameraPose;
    Eigen::Matrix4f cameraPose = sensorOrigin;
    bool accummulateTransform = false;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredPointCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr checkedPointCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());

    boost::optional<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
        boostCheckedCloud = checkForArtifactsInPointcloud(inPointCloud);
    if (boostCheckedCloud == boost::none)
        return boost::none;
    *checkedPointCloud = *boostCheckedCloud.get();

    boost::optional<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
        boostFilterCloud = filterInputPointcloud(checkedPointCloud);
    if (boostFilterCloud == boost::none)
        return boost::none;
    *filteredPointCloud = *boostFilterCloud.get();

    if (cameraPose == Eigen::Matrix4f::Identity()) {
        Eigen::Matrix4f accummulatedTransform;
        accummulatedTransform.setIdentity();
        accummulatedTransform(0, 0) = accummulatedTransform_[0];
        accummulatedTransform(0, 1) = accummulatedTransform_[1];
        accummulatedTransform(0, 2) = accummulatedTransform_[2];
        accummulatedTransform(1, 0) = accummulatedTransform_[3];
        accummulatedTransform(1, 1) = accummulatedTransform_[4];
        accummulatedTransform(1, 2) = accummulatedTransform_[5];
        accummulatedTransform(2, 0) = accummulatedTransform_[6];
        accummulatedTransform(2, 1) = accummulatedTransform_[7];
        accummulatedTransform(2, 2) = accummulatedTransform_[8];
        accummulatedTransform(0, 3) = accummulatedTransform_[9];
        accummulatedTransform(1, 3) = accummulatedTransform_[10];
        accummulatedTransform(2, 3) = accummulatedTransform_[11];
        cameraPose *= accummulatedTransform;
        accummulateTransform = true;
    }

    pcl::transformPointCloud(*filteredPointCloud, *transformedCloud, cameraPose);
    transformedCloud->sensor_origin_ = Eigen::Vector4f(cameraPose(0, 3),
        cameraPose(1, 3), cameraPose(2, 3), 0.0f);

    if (fullCloudNormals_->empty()) {
        if (!insertFirstPointcloud(transformedCloud, cameraPose)) {
            log_->error("Impossible to insert first Pointcloud");
            return boost::none;
        }
        finalCameraPose = cameraPose;
    } else {
        boost::optional<Eigen::Matrix4f> boostFinalCameraPose =
            insertNextPointcloud(transformedCloud, cameraPose, accummulateTransform);
        if (!boostFinalCameraPose) {
            log_->error("Impossible to insert next Pointcloud");
            return boost::none;
        }
        finalCameraPose = boostFinalCameraPose.get();
    }

    log_->info("Pointcloud current size: {0} points", fullCloudNormals_->size());
    if (static_cast<int>(fullCloudNormals_->size()) >= maxNumberOfPoints_) {
            log_->error("Pointcloud size exceeding maximum number of points defined");
            return boost::none;
    }

    if (updateOctree_) {
        log_->info("Octree current size: {0} nodes", fullNormalsColorOctree_->size());
        if (static_cast<int>(fullNormalsColorOctree_->size()) >= maxNumberOfNodes_) {
            log_->error("Octree size exceeding maximum number of nodes defined");
            return boost::none;
        }
    }

    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    log_->info("Map update full elapsed time: {0} milliseconds", mtime);

    return finalCameraPose;
}

boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> HandHeldMapper3d::getPointCloudMap() {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outPointCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::copyPointCloud(*fullCloudNormals_, *outPointCloud);
    if (outPointCloud->empty()) {
        log_->warn("Not enough points stored to return the map");
        return outPointCloud;
    }

    return outPointCloud;
}

boost::optional<NormalsColorOctree> HandHeldMapper3d::getOctreeMap() {
    if (fullNormalsColorOctree_->size() <= 0) {
        log_->warn("Octree Map empty");
        return boost::none;
    }
    return *fullNormalsColorOctree_;
}


bool HandHeldMapper3d::clearMap() {
    if (fullCloudNormals_->empty()) {
        log_->error("Unable to clear the pointcloud map");
        return false;
    } else {
        fullCloudNormals_->clear();
    }

    if (fullNormalsColorOctree_->size() == 0) {
        log_->error("Unable to clear the octree map");
        return false;
    } else {
        fullNormalsColorOctree_->clear();
    }

    return true;
}

int HandHeldMapper3d::getPointCloudMapSize() {
    return fullCloudNormals_->size();
}

int HandHeldMapper3d::getOctreeMapSize() {
    return fullNormalsColorOctree_->size();
}

bool HandHeldMapper3d::savePointCloudToDisk(const std::string& savingPathWithoutExtension,
        bool plyflag) {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outPointCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    bool pointCloudNameFlag = false;
    int pc_num = 0;
    std::string save_name = savingPathWithoutExtension;
    std::string extension = ".pcd";
    if (save_name.find(extension) != std::string::npos) {
        log_->error("Impossible to save Pointcloud, remove extension from path");
        return false;
    }
    save_name.append(extension);

    pcl::copyPointCloud(*fullCloudNormals_, *outPointCloud);

    // Check if the pointcloud file already exits to avoid overwriting
    while (!pointCloudNameFlag) {
        if (!isFileExist(save_name)) {
            if (plyflag == true) {
                save_name = save_name.substr(0, save_name.size()-3);
                std::string extension2 = "ply";
                save_name.append(extension2);
                pcl::io::savePLYFileBinary(save_name, *outPointCloud);
            } else {
                pcl::io::savePCDFileBinary(save_name, *outPointCloud);
            }
            log_->info("Pointcloud {0} saved", save_name);
            pointCloudNameFlag = true;
        } else {
            if (pc_num == 0) {
                save_name = save_name.substr(0, save_name.size()-4);
            } else {
                if (pc_num > 9) {
                    save_name = save_name.substr(0, save_name.size()-8);
                } else {
                    save_name = save_name.substr(0, save_name.size()-7);
                }
            }
            pc_num++;
            save_name.append("(");
            std::string pc_num_str = std::to_string(pc_num);
            save_name.append(pc_num_str);
            save_name.append(")");
            save_name.append(extension);
        }
    }
    pointCloudNameFlag = false;
    return true;
}

bool HandHeldMapper3d::saveOctreeToDisk(const std::string& savingPathWithoutExtension) {
    bool octreeNameFlag = false;
    int oc_num = 0;
    std::string save_name = savingPathWithoutExtension;
    std::string extension = ".ot";
    if (save_name.find(extension) != std::string::npos) {
        log_->error("Impossible to save Octree, remove extension from path");
        return false;
    }
    save_name.append(extension);

    // Check if the octree file already exits to avoid overwriting
    while (!octreeNameFlag) {
        if (!isFileExist(save_name)) {
            fullNormalsColorOctree_->write(save_name);
            log_->info("Octree {0} saved", save_name);
            octreeNameFlag = true;
        } else {
            if (oc_num == 0) {
                save_name = save_name.substr(0, save_name.size()-3);
            } else {
                save_name = save_name.substr(0, save_name.size()-6);
            }
            oc_num++;
            save_name.append("(");
            std::string pc_num_str = std::to_string(oc_num);
            save_name.append(pc_num_str);
            save_name.append(")");
            save_name.append(extension);
        }
    }
    octreeNameFlag = false;
    return true;
}


boost::optional<Eigen::Matrix4f> HandHeldMapper3d::comparePointClouds(
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourcePointCloud,
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetPointCloud,
        const Eigen::Matrix4f &sourceSensorOrigin,
        const Eigen::Matrix4f &targetSensorOrigin) {
    return boost::none;
}

boost::optional<Eigen::Matrix4f> HandHeldMapper3d::comparePointClouds(
    bool organizedPointCloud,
    bool srcNormalCalculated,
    bool tgtNormalCalculated,
    const Eigen::Matrix4f &sourceSensorOrigin,
    const Eigen::Matrix4f &targetSensorOrigin,
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourcePointCloud,
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetPointCloud,
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals,
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals) {
    return boost::none;
}

boost::optional<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
        HandHeldMapper3d::checkForArtifactsInPointcloud(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
    log_->debug("Artifacts checking");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr auxCloud;
    auxCloud = cloud;
    // If there are points closer than passThroughMinLimit cm,
    // dont take into account this pointcloud to avoid artifacts
     for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = auxCloud->begin();
            it != auxCloud->end(); it++) {
        if (it->x == 0 && it->y == 0 && it->z == 0) {
            it->x = std::numeric_limits<float>::quiet_NaN();
            it->y = std::numeric_limits<float>::quiet_NaN();
            it->z = std::numeric_limits<float>::quiet_NaN();
        }
        if (it->z < passThroughMinLimit_ && it->z > 0) {
            log_->warn("Found points too close to the camera");
            log_->warn("Removing pointcloud from pipeline");
            return boost::none;
        }
    }
    log_->debug("Artifacts checked");
    return auxCloud;
}

bool HandHeldMapper3d::insertFirstPointcloud(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &transformedCloud,
        const Eigen::Matrix4f &cameraPose) {
    log_->debug("Inserting first pointcloud");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloudXYZRGB
            (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sourceFullCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals (new pcl::PointCloud<pcl::Normal>);

    // Compute normals from organized pointcloud
    boost::optional<pcl::PointCloud<pcl::Normal>::Ptr>
    boostSourceNormals = computeIntegralNormals(transformedCloud,
        cameraPose(0, 3), cameraPose(1, 3), cameraPose(2, 3));
    if (!boostSourceNormals)
        return false;
    *sourceNormals = *boostSourceNormals.get();

    // Remove NaN from Normals and Pointcloud
    std::vector<int> srcInd;
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        transformedCloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::removeNaNNormalsFromPointCloud(*sourceNormals, *sourceNormals2, srcInd);
    for (int i = 0; i < static_cast<int>(srcInd.size()); i++) {
            transformedCloud2->push_back(transformedCloud->points[srcInd[i]]);
    }

    // Create PointXYZRGBNormal pointcloud
    pcl::copyPointCloud(*transformedCloud2, *transformedCloudXYZRGB);
    pcl::concatenateFields(*transformedCloudXYZRGB, *sourceNormals2, *sourceFullCloud);

    // Filter PointXYZRGBNormal cloud
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
    downsampledPointCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> boostFinalPointCloud
        = filterOutputPointcloud(sourceFullCloud);
    if (!boostFinalPointCloud)
        return false;
    *downsampledPointCloud = *boostFinalPointCloud.get();

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
        finalPointCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    if (updateOctree_) {
        if (octreeUpdate(downsampledPointCloud, cameraPose(0, 3),
                cameraPose(1, 3), cameraPose(2, 3))) {
            *finalPointCloud = *getOccupiedPointCloudFromOctree().get();
        } else {
            log_->info("Impossible to update Octree");
            return false;
        }
    } else {
         *finalPointCloud = *downsampledPointCloud;
    }

    // Store first fullcloud with normals after all filtering
    *fullCloudNormals_ = *finalPointCloud;

    log_->debug("First pointcloud inserted");
    return true;
}

boost::optional<Eigen::Matrix4f> HandHeldMapper3d::insertNextPointcloud(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &transformedCloud,
        const Eigen::Matrix4f &cameraPose,
        bool accummulateTransform) {
    log_->debug("Inserting next pointcloud");

    Eigen::Matrix4f icpTransform;
    Eigen::Matrix4f finalCameraPose = cameraPose;

    // ICP refinement comparing with the full cloud
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    *auxCloud = *fullCloudNormals_;
    boost::optional<Eigen::Matrix4f> boostICPTransform =
        icpRefinement(transformedCloud, auxCloud);
    if (boostICPTransform == boost::none)
        return boost::none;
    icpTransform = boostICPTransform.get();

    // Store estimated transform and add it to the accumlated transform
    if (accummulateTransform) {
        Eigen::Matrix4f accummulatedTransform;
        accummulatedTransform.setIdentity();
        accummulatedTransform(0, 0) = accummulatedTransform_[0];
        accummulatedTransform(0, 1) = accummulatedTransform_[1];
        accummulatedTransform(0, 2) = accummulatedTransform_[2];
        accummulatedTransform(1, 0) = accummulatedTransform_[3];
        accummulatedTransform(1, 1) = accummulatedTransform_[4];
        accummulatedTransform(1, 2) = accummulatedTransform_[5];
        accummulatedTransform(2, 0) = accummulatedTransform_[6];
        accummulatedTransform(2, 1) = accummulatedTransform_[7];
        accummulatedTransform(2, 2) = accummulatedTransform_[8];
        accummulatedTransform(0, 3) = accummulatedTransform_[9];
        accummulatedTransform(1, 3) = accummulatedTransform_[10];
        accummulatedTransform(2, 3) = accummulatedTransform_[11];

        accummulatedTransform = icpTransform * accummulatedTransform;

        accummulatedTransform_[0] = static_cast<float>(accummulatedTransform(0, 0));
        accummulatedTransform_[1] = static_cast<float>(accummulatedTransform(0, 1));
        accummulatedTransform_[2] = static_cast<float>(accummulatedTransform(0, 2));
        accummulatedTransform_[3] = static_cast<float>(accummulatedTransform(1, 0));
        accummulatedTransform_[4] = static_cast<float>(accummulatedTransform(1, 1));
        accummulatedTransform_[5] = static_cast<float>(accummulatedTransform(1, 2));
        accummulatedTransform_[6] = static_cast<float>(accummulatedTransform(2, 0));
        accummulatedTransform_[7] = static_cast<float>(accummulatedTransform(2, 1));
        accummulatedTransform_[8] = static_cast<float>(accummulatedTransform(2, 2));
        accummulatedTransform_[9] = static_cast<float>(accummulatedTransform(0, 3));
        accummulatedTransform_[10] = static_cast<float>(accummulatedTransform(1, 3));
        accummulatedTransform_[11] = static_cast<float>(accummulatedTransform(2, 3));

        finalCameraPose = accummulatedTransform;
    } else {
        finalCameraPose *= icpTransform;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr currentCloudNormalsTransformed
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::transformPointCloud(*currentCloudNormals_, *currentCloudNormalsTransformed, icpTransform);

    // Filter output pointcloud before octree update
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
    downsampledPointCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> boostFinalPointCloud
        = filterOutputPointcloud(currentCloudNormalsTransformed);
    if (boostFinalPointCloud == boost::none)
        return boost::none;
    *downsampledPointCloud = *boostFinalPointCloud.get();


    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
    finalPointCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    if (updateOctree_) {
        if (octreeUpdate(downsampledPointCloud, finalCameraPose(0, 3),
                finalCameraPose(1, 3), finalCameraPose(2, 3))) {
            *finalPointCloud = *getOccupiedPointCloudFromOctree().get();
            fullCloudNormals_->clear();
            *fullCloudNormals_ = *finalPointCloud;
        } else {
            log_->info("Impossible to update Octree");
            return boost::none;
        }
    } else {
            *fullCloudNormals_ += *downsampledPointCloud;
    }

    log_->debug("Next pointcloud inserted");
    return finalCameraPose;
}

bool HandHeldMapper3d::octreeUpdate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr newPointcloud,
        float originX, float originY, float originZ) {
    struct timeval start, end;
    int64_t mtime, seconds, useconds;
    gettimeofday(&start, NULL);

    log_->debug("Updating octree");

    if (newPointcloud->size() <= 0) {
        log_->warn("Empty pointcloud, impossible to update octree");
        return false;
    }

    std::vector<int> srcInd;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
        newPointcloudNoNaN (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::removeNaNFromPointCloud(*newPointcloud, *newPointcloudNoNaN, srcInd);

    octomap::Pointcloud octocloud;
    for (auto p : newPointcloudNoNaN->points)
        octocloud.push_back(p.x, p.y, p.z);
    fullNormalsColorOctree_->insertPointCloud(octocloud, octomap::point3d(originX,
        originY, originZ), passThroughMaxLimit_, false, true);
    for (auto p : newPointcloudNoNaN->points) {
        fullNormalsColorOctree_->integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
        fullNormalsColorOctree_->integrateNodeNormals(p.x, p.y, p.z,
            p.normal_x, p.normal_y, p.normal_z);
    }

    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    log_->info("Octree update elapsed time: {0} milliseconds", mtime);

    return true;
}

boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>
    HandHeldMapper3d::getOccupiedPointCloudFromOctree() {
    struct timeval start, end;
    int64_t mtime, seconds, useconds;
    gettimeofday(&start, NULL);

    log_->debug("Getting Occupied Pointcloud from Octree");
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
        outputPointCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    for (NormalsColorOctree::leaf_iterator it = fullNormalsColorOctree_->begin_leafs();
        it != fullNormalsColorOctree_->end_leafs(); ++it) {
        if (fullNormalsColorOctree_->isNodeOccupied(*it)) {
            pcl::PointXYZRGBNormal point;
            point.x = it.getX();
            point.y = it.getY();
            point.z = it.getZ();
            NormalsColorOctreeNode node =
                *fullNormalsColorOctree_->search(point.x, point.y, point.z, 0);
            point.r = node.getColor().r;
            point.g = node.getColor().g;
            point.b = node.getColor().b;
            point.normal_x = node.getNormals().normal_x;
            point.normal_y = node.getNormals().normal_y;
            point.normal_z = node.getNormals().normal_z;
            outputPointCloud->push_back(point);
        }
    }

    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    log_->info("Occupied Pointcloud extraction elapsed time: {0} milliseconds", mtime);

    return outputPointCloud;
}

boost::optional<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
    HandHeldMapper3d::filterInputPointcloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rawCloud) {
    struct timeval start, end;
    int64_t mtime, seconds, useconds;
    gettimeofday(&start, NULL);
    log_->debug("Filtering input cloud");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trimmedPointCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredPointCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr smothedPointCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());

    if (!rawCloud->isOrganized()) {
        if (passThrough_ || statisticalOutlier_ || fastBilateral_) {
            log_->error("Input cloud is not organized, not possible to apply filters");
            return boost::none;
        }
    }

    // Remove points distant more than passThroughMaxLimit meters
    // from camera (se mantiene organizada la poincloud)
    if (passThrough_) {
        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setKeepOrganized(true);
        pass.setInputCloud(rawCloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(passThroughMinLimit_, passThroughMaxLimit_);
        pass.filter(*trimmedPointCloud);
    } else {
        *trimmedPointCloud = *rawCloud;
    }

    // Remove outliers
    if (statisticalOutlier_) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
        sor.setKeepOrganized(true);
        sor.setInputCloud(trimmedPointCloud);
        sor.setMeanK(statisticalOutlierMeanK_);
        sor.setStddevMulThresh(statisticalOutlierStddevMulThresh_);
        sor.filter(*filteredPointCloud);
    } else {
        *filteredPointCloud = *trimmedPointCloud;
    }

    // Smooth cloud
    if (fastBilateral_) {
        pcl::FastBilateralFilter<pcl::PointXYZRGBA> bilateral_filter;
        bilateral_filter.setInputCloud(filteredPointCloud);
        bilateral_filter.setSigmaS(fastBilateralSigmaS_);
        bilateral_filter.setSigmaR(fastBilateralSigmaR_);
        bilateral_filter.filter(*smothedPointCloud);
    } else {
        *smothedPointCloud = *filteredPointCloud;
    }

    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    log_->info("Input cloud filtering elapsed time: {0} milliseconds", mtime);

    return smothedPointCloud;
}

boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>
    HandHeldMapper3d::filterOutputPointcloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
    rawCloud) {
    log_->debug("Filtering output cloud");
    if (voxelGridLeafSizeX_ <= 0) {
        log_->error("voxelGridLeafSizeX should be greater than 0");
        return boost::none;
    }
    if (mlsSearchRadius_ <= 0 || mlsPolynomialOrder_ <= 0) {
        log_->error("MLS parameters should be greater than 0");
        return boost::none;
    }

    struct timeval start, end;
    int64_t mtime, seconds, useconds;
    gettimeofday(&start, NULL);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
        smothedPointCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
        downsampledPointCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    // Downsample cloud
    if (voxelGrid_) {
        pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
        grid.setLeafSize(voxelGridLeafSizeX_, voxelGridLeafSizeY_, voxelGridLeafSizeZ_);
        grid.setInputCloud(rawCloud);
        grid.filter(*downsampledPointCloud);
    } else {
        downsampledPointCloud = rawCloud;
    }

    // Smooth cloud with MLS
    if (mls_) {
        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr
            tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
        pcl::MovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> mls;
        mls.setInputCloud(downsampledPointCloud);
        mls.setPolynomialOrder(mlsPolynomialOrder_);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(mlsSearchRadius_);
        mls.process(*smothedPointCloud);

    } else {
        smothedPointCloud = downsampledPointCloud;
    }

    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    log_->info("Output cloud filtering elapsed time: {0} milliseconds", mtime);

    return smothedPointCloud;
}

boost::optional<Eigen::Matrix4f> HandHeldMapper3d::icpRefinement
    (const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr srcPointCloud,
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tgtFullCloud) {

    struct timeval start, end;
    int64_t mtime, seconds, useconds;
    gettimeofday(&start, NULL);

    log_->debug("ICP Refinement");

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sourceFullCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceXYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>());


    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        srcTransformedPointcloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        finalSrcTransformedPointcloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        srcNormalSampledPointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        tgtNormalSampledPointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::Normal>::Ptr sourceSampledNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetSampledNormals (new pcl::PointCloud<pcl::Normal>);
    Eigen::Matrix4d finalTransform(Eigen::Matrix4d::Identity());

    Eigen::Vector4f sourceSensorOrigin = srcPointCloud->sensor_origin_;

    // Compute normals from organized pointcloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals (new pcl::PointCloud<pcl::Normal>);
    boost::optional<pcl::PointCloud<pcl::Normal>::Ptr>
    boostSourceNormals = computeIntegralNormals(srcPointCloud, sourceSensorOrigin(0),
        sourceSensorOrigin(1), sourceSensorOrigin(2));
    if (!boostSourceNormals)
        return boost::none;
    *sourceNormals = *boostSourceNormals.get();


    // Remove NaN from source clouds
    std::vector<int> srcInd;
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        srcPointCloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        tgtPointCloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::removeNaNNormalsFromPointCloud(*sourceNormals, *sourceNormals2, srcInd);
    for (int i = 0; i < static_cast<int>(srcInd.size()); i++) {
            srcPointCloud2->push_back(srcPointCloud->points[srcInd[i]]);
    }

    // Store source full cloud
    pcl::copyPointCloud(*srcPointCloud2, *sourceXYZRGB);
    pcl::concatenateFields(*sourceXYZRGB, *sourceNormals2, *sourceFullCloud);
    currentCloudNormals_->clear();
    *currentCloudNormals_ = *sourceFullCloud;

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

    if (randomSampling_) {
        boostSrcRemovedIndices = randomSubsampling(srcPointCloud2);
        if (!boostSrcRemovedIndices)
            return boost::none;
        srcRemovedIndices = boostSrcRemovedIndices.get();
        boostTgtRemovedIndices = randomSubsampling(tgtPointCloud2);
        if (!boostTgtRemovedIndices)
            return boost::none;
        tgtRemovedIndices = boostTgtRemovedIndices.get();
    } else {
        // Compute subsamplingMultiplication
         if (normalSamplingTotalPointsDivision_ <= 0) {
            log_->error("normalSamplingTotalPointsDivision should be greater than 0");
            return boost::none;
        }
        int subsamplingMultiplication = 1;
        int sourceCloudSize, targetCloudSize, aux = 0;
        targetCloudSize =
            static_cast<int>(tgtPointCloud2->size()) / normalSamplingTotalPointsDivision_;
        sourceCloudSize =
            static_cast<int>(srcPointCloud2->size()) / normalSamplingTotalPointsDivision_;
        aux = sourceCloudSize / targetCloudSize;
        if (aux != 0)
            subsamplingMultiplication = aux;
        boostSrcRemovedIndices = normalsSubsampling(srcPointCloud2,
            sourceNormals2, subsamplingMultiplication);
        if (!boostSrcRemovedIndices)
            return boost::none;
        srcRemovedIndices = boostSrcRemovedIndices.get();
        boostTgtRemovedIndices = normalsSubsampling(tgtPointCloud2, targetNormals2, 1);
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

    if (icpSourceNormalsVisualization_) {
        pcl::visualization::PCLVisualizer viewerNormals("PCL Source Normals Viewer");
        viewerNormals.setBackgroundColor(0.0, 0.0, 0.5);
        viewerNormals.addPointCloud(srcPointCloud3, "src cloud");
        viewerNormals.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>
            (srcPointCloud3, sourceNormals3, 1);

        while (!viewerNormals.wasStopped()) {
          viewerNormals.spinOnce();
        }
    }

    if (icpInitialAlignment_) {
        boost::optional<Eigen::Matrix4f> boostInitialTransform = icpInitialAligment(srcPointCloud2,
             tgtPointCloud2);
        if (boostInitialTransform == boost::none)
            return boost::none;
        Eigen::Matrix4f initialTransform = boostInitialTransform.get();
        finalTransform = initialTransform.cast <double> ();
    }

    // Transform estimation
    int iteration = 0;
    Eigen::Matrix4d estimatedTransform(Eigen::Matrix4d::Identity());
    pcl::CorrespondencesPtr correspondences (new pcl::Correspondences ());
    pcl::registration::DefaultConvergenceCriteria<double>
        convergenceCriteria(iteration, estimatedTransform, *correspondences);
    if (translationThreshold_ <=0 || rotationThreshold_ <=0 || maximumIterations_ <=0) {
        log_->error("Translation, rotation and maximumIterations should be greater than 0");
        return boost::none;
    }
    convergenceCriteria.setTranslationThreshold(translationThreshold_);  // meters
    float fthres = cos(rotationThreshold_);
    convergenceCriteria.setRotationThreshold(fthres);  // rad
    convergenceCriteria.setMaximumIterations(maximumIterations_);

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

    std::unique_ptr<pcl::visualization::PCLVisualizer> viewer(nullptr);
    if (icpStepsVisualization_) {
        viewer.reset(new pcl::visualization::PCLVisualizer("PCL ICP Viewer"));
        viewer->setBackgroundColor(0.0, 0.0, 0.5);
    }

    do {
        pcl::transformPointCloud(*srcPointCloud3, *srcPointCloud3_aligned, finalTransform);
        pcl::transformPointCloudWithNormals(*sourcePointNormals,
            *source_normalsAligned, finalTransform);

        *correspondences = *correspondenceEstimation(srcPointCloud3_aligned, tgtPointCloud3);

        pcl::CorrespondencesPtr remainingCorrespondences(new pcl::Correspondences ());
        *remainingCorrespondences = *medianDistanceCorrespondeceRejection(correspondences);

        pcl::CorrespondencesPtr remainingCorrespondences2 (new pcl::Correspondences ());
        boost::optional<pcl::CorrespondencesPtr> boostRemainingCorrespondences =
            surfaceNormalCorrespondeceRejection(remainingCorrespondences, srcPointCloud3_aligned,
            tgtPointCloud3, source_normalsAligned, targetPointNormals);
        if (!boostRemainingCorrespondences)
            return boost::none;
        *remainingCorrespondences2 = *boostRemainingCorrespondences.get();


        if (icpStepsVisualization_ && viewer) {
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

        estimatedTransform = transformEstimation(remainingCorrespondences2, srcPointCloud3_aligned,
            source_normalsAligned, targetPointNormals);
        // Update variables
        iteration++;
        finalTransform = estimatedTransform * finalTransform;
    } while (!convergenceCriteria.hasConverged());

    Eigen::Matrix4f accummulatedTransform(Eigen::Matrix4f::Identity());
    accummulatedTransform = finalTransform.cast <float> ();

    log_->debug("Convergence state {0}", convergenceCriteria.getConvergenceState());
    log_->debug("Final estimated transform: \n {0}", finalTransform);
    log_->debug("Number of iteration needed: {0}", iteration);

    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    log_->info("ICP Refinement elapsed time: {0} milliseconds", mtime);

    return accummulatedTransform;
}

boost::optional<Eigen::Matrix4f> HandHeldMapper3d::icpInitialAligment(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr srcPointCloud,
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tgtPointCloud) {

    struct timeval start, end;
    int64_t mtime, seconds, useconds;
    gettimeofday(&start, NULL);

    log_->debug("Initial Aligment");
    Eigen::Matrix4f initialTransform(Eigen::Matrix4f::Identity());

    // Downsample pointclouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceInput(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudFiltered2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetInput(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloudFiltered2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*srcPointCloud, *sourceInput);
    pcl::copyPointCloud(*tgtPointCloud, *targetInput);
    if (!subsample(sourceInput, sourceCloudFiltered)) {
        log_->error("Impossible to subsample pointcloud");
        return boost::none;
    }
    if (!subsample(targetInput, targetCloudFiltered)) {
        log_->error("Impossible to subsample pointcloud");
        return boost::none;
    }

    // Compute normals
    pcl::PointCloud<pcl::Normal>::Ptr sourceCloudNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr sourceCloudNormals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetCloudNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetCloudNormals2 (new pcl::PointCloud<pcl::Normal>);
    std::vector<int> tgtInd;
    std::vector<int> srcInd;
    if (!computeNormals(sourceCloudFiltered, sourceCloudNormals)) {
        log_->error("Impossible to compute normals");
        return boost::none;
    }
    if (!computeNormals(targetCloudFiltered, targetCloudNormals)) {
        log_->error("Impossible to compute normals");
        return boost::none;
    }

    pcl::removeNaNNormalsFromPointCloud(*sourceCloudNormals, *sourceCloudNormals2, srcInd);
    pcl::removeNaNNormalsFromPointCloud(*targetCloudNormals, *targetCloudNormals2, tgtInd);
    for (int i = 0; i < static_cast<int>(srcInd.size()); i++) {
        sourceCloudFiltered2->push_back(sourceCloudFiltered->points[srcInd[i]]);
    }
    for (int i = 0; i < static_cast<int>(tgtInd.size()); i++) {
        targetCloudFiltered2->push_back(targetCloudFiltered->points[tgtInd[i]]);
    }

    // Keypoints estimation and descriptors
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr
        sourceFeatures(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr
        targetFeatures(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
    if (!computeFPFHKeypoints(sourceCloudFiltered2, sourceCloudNormals2,
        sourceFeatures, sourceKeypoints)) {
        log_->error("Impossible to compute FPFH Keypoints");
        return boost::none;
    }
    if (!computeFPFHKeypoints(targetCloudFiltered2, targetCloudNormals2,
        targetFeatures, targetKeypoints)) {
        log_->error("Impossible to compute FPFH Keypoints");
        return boost::none;
    }

    targetCloudFiltered2->sensor_origin_.head<3>().fill(0);
    targetCloudFiltered2->sensor_orientation_.setIdentity();
    sourceCloudFiltered2->sensor_origin_.head<3>().fill(0);
    sourceCloudFiltered2->sensor_orientation_.setIdentity();
    targetKeypoints->sensor_origin_.head<3>().fill(0);
    targetKeypoints->sensor_orientation_.setIdentity();
    sourceKeypoints->sensor_origin_.head<3>().fill(0);
    sourceKeypoints->sensor_orientation_.setIdentity();

    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences ());
    *correspondences = *correspondenceEStimationFPFHSignature33(sourceFeatures, targetFeatures);

    pcl::CorrespondencesPtr remainingCorrespondences (new pcl::Correspondences());
    boost::optional<pcl::CorrespondencesPtr> boostRemainingCorrespondences =
        sampleConsensusCorrespondeceRejection(correspondences, sourceKeypoints, targetKeypoints);
    if (!boostRemainingCorrespondences)
        return boost::none;
    *remainingCorrespondences = *boostRemainingCorrespondences.get();

    if (icpInitialAligmentVisualization_) {
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

    // Transformation estimation
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,
        pcl::PointXYZ> transformationEstimation;
    transformationEstimation.estimateRigidTransformation(*sourceKeypoints, *targetKeypoints,
        *remainingCorrespondences, initialTransform);
    log_->debug("Initial estimated transform: \n {0}", initialTransform);

    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    log_->info("ICP Initial Aligment elapsed time: {0} milliseconds", mtime);

    return initialTransform;
}

pcl::CorrespondencesPtr HandHeldMapper3d::correspondenceEstimation(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputSource,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputTarget) {
        log_->debug("Correspondence estimation");
        pcl::CorrespondencesPtr correspondences (new pcl::Correspondences ());
        pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGBA,
            pcl::PointXYZRGBA> correspKdtree;
        correspKdtree.setInputSource(inputSource);
        correspKdtree.setInputTarget(inputTarget);
        correspKdtree.determineCorrespondences(*correspondences, correspondencesMaximumDistance_);
        log_->debug("Remaining Correspondences after distance rejection: {0}",
            correspondences->size());

        return correspondences;
}

pcl::CorrespondencesPtr HandHeldMapper3d::correspondenceEStimationFPFHSignature33(
        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &sourceFeatures,
        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &targetFeatures) {
        log_->debug("FPFHSignature33 Correspondence estimation");

        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences ());
        pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
        est.setInputSource(sourceFeatures);
        est.setInputTarget(targetFeatures);
        est.determineCorrespondences(*correspondences);
        log_->debug("FPFHSignature33 Correspondences size: {0}", correspondences->size());
        return correspondences;
}

pcl::CorrespondencesPtr HandHeldMapper3d::medianDistanceCorrespondeceRejection(
        const pcl::CorrespondencesPtr &initialCorrespondences) {
    log_->debug("Median Distance Correspondece Rejection estimation");
    pcl::CorrespondencesPtr remainingCorrespondences (new pcl::Correspondences ());
    pcl::registration::CorrespondenceRejectorMedianDistance medDistanceRej;
    medDistanceRej.setMedianFactor(correspondencesMaximumMedianDistanceFactor_);
    medDistanceRej.setInputCorrespondences(initialCorrespondences);
    medDistanceRej.getCorrespondences(*remainingCorrespondences);
    log_->debug("Remaining Correspondences after median distance rejection: {0}",
        remainingCorrespondences->size());

    return remainingCorrespondences;
}

boost::optional<pcl::CorrespondencesPtr> HandHeldMapper3d::surfaceNormalCorrespondeceRejection(
        const pcl::CorrespondencesPtr &initialCorrespondences,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputSource,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputTarget,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &inputNormals,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &targetNormals) {
    log_->debug("Surface Normal Correspondece Rejection estimation");
    pcl::CorrespondencesPtr remainingCorrespondences (new pcl::Correspondences ());
    pcl::registration::CorrespondenceRejectorSurfaceNormal normalRej;
    normalRej.initializeDataContainer<pcl::PointXYZRGBA, pcl::PointNormal>();
    normalRej.setInputSource<pcl::PointXYZRGBA>(inputSource);
    normalRej.setInputTarget<pcl::PointXYZRGBA>(inputTarget);
    normalRej.setInputNormals<pcl::PointXYZRGBA, pcl::PointNormal>(inputNormals);
    normalRej.setTargetNormals<pcl::PointXYZRGBA, pcl::PointNormal>(targetNormals);
    normalRej.setInputCorrespondences(initialCorrespondences);
    float fthres = cos(correspondencesMaximunAngleInRad_);
    normalRej.setThreshold(fthres);
    normalRej.getCorrespondences(*remainingCorrespondences);
    log_->debug("Remaining Correspondences after normals angle rejection: {0}",
            remainingCorrespondences->size());
    if (remainingCorrespondences->size() < 4) {
        log_->error("Not enough points to estimate a transform");
        log_->info("Tune correspondences rejection parameters");
        return boost::none;
    }

    return remainingCorrespondences;
}

boost::optional<pcl::CorrespondencesPtr> HandHeldMapper3d::sampleConsensusCorrespondeceRejection(
        const pcl::CorrespondencesPtr &initialCorrespondences,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &sourceKeypoints,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &targetKeypoints) {
    log_->debug("Sample Consensus Correspondece Rejection");
    pcl::CorrespondencesPtr remainingCorrespondences (new pcl::Correspondences());
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejectorSac;
    rejectorSac.setInputSource(sourceKeypoints);
    rejectorSac.setInputTarget(targetKeypoints);
    rejectorSac.setInlierThreshold(sampleConsensusInlierThreshold_);
    rejectorSac.setMaximumIterations(sampleConsensusMaximumIterations_);
    rejectorSac.setRefineModel(sampleConsensusRefineModel_);
    rejectorSac.setInputCorrespondences(initialCorrespondences);;
    rejectorSac.getCorrespondences(*remainingCorrespondences);

    if (remainingCorrespondences->size() < 4) {
        log_->error("Not enough points to estimate an initial transform");
        log_->info("Tune Initial aligment parameters");
        return boost::none;
    }

    return remainingCorrespondences;
}

Eigen::Matrix4d HandHeldMapper3d::transformEstimation(
        const pcl::CorrespondencesPtr &correspondences,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &sourcePointcloud,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &sourceNormals,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &targetNormals) {
    log_->debug("Transform Estimation");

    Eigen::Matrix4d estimatedTransform(Eigen::Matrix4d::Identity());
    pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal,
        pcl::PointNormal, double> transformEstimator;
    pcl::registration::TransformationEstimationPointToPlaneWeighted<pcl::PointNormal,
        pcl::PointNormal, double> weightedTransformEstimator;

    // Compute weights using common RGBD camera noise model
    if (weightedTransformEstimation_) {
        std::vector< double > weights(correspondences->size());
        for (int i = 0; i < static_cast<int>(correspondences->size()); i++) {
            float depth = sourcePointcloud->points[(*correspondences)[i].index_match].z;
            float aux = (depth - passThroughMinLimit_) * (depth - passThroughMinLimit_);
            float sigma_z = 0.0012f + 0.0019f * aux;
            (*correspondences)[i].weight = 0.0012f / sigma_z;
            weights[i] = (*correspondences)[i].weight;
        }
        weightedTransformEstimator.setWeights(weights);
        weightedTransformEstimator.estimateRigidTransformation(*sourceNormals,
                *targetNormals, *correspondences, estimatedTransform);
    } else {
        transformEstimator.estimateRigidTransformation(*sourceNormals,
                *targetNormals, *correspondences, estimatedTransform);
    }
    log_->debug("Transform Estimated");

    return estimatedTransform;
}

bool HandHeldMapper3d::subsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subsampled) {
    if (subsamplingSideLength_ <= 0) {
            log_->error("subsamplingSideLength should be greater than 0");
            return false;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr auxCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *auxCloud);
    pcl::VoxelGrid<pcl::PointXYZ> subsampling_filter;
    subsampling_filter.setInputCloud(auxCloud);
    subsampling_filter.setLeafSize(subsamplingSideLength_,
        subsamplingSideLength_, subsamplingSideLength_);
    subsampling_filter.filter(*cloud_subsampled);
    return true;
}

boost::optional<std::vector<int>> HandHeldMapper3d::randomSubsampling(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
    log_->debug("Random sumsampling");
    std::vector<int> removedIndices;

    if (randomlSamplingTotalPointsDivision_ <= 0) {
            log_->error("randomlSamplingTotalPointsDivision should be greater than 0");
            return boost::none;
    }

    pcl::RandomSample<pcl::PointXYZRGBA> srcRandomSampling;
    srcRandomSampling.setInputCloud(cloud);
    srcRandomSampling.setSample(static_cast<unsigned int>
        (cloud->size()/randomlSamplingTotalPointsDivision_));
    srcRandomSampling.filter(removedIndices);

    log_->debug("Random sumsampling finished");
    return removedIndices;
}

boost::optional<std::vector<int>> HandHeldMapper3d::normalsSubsampling(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals,
        int subsamplingMultiplication) {
    log_->debug("Normals subsampling");
    std::vector<int> removedIndices;

    if (normalSamplingTotalPointsDivision_ <= 0) {
        log_->error("normalSamplingTotalPointsDivision should be greater than 0");
        return boost::none;
    }
    if (normalSamplingBinSize_ <= 0) {
        log_->error("normalSamplingBinSize should be greater than 0");
        return boost::none;
    }
    // Source Normal space subsampling
    pcl::NormalSpaceSampling<pcl::PointXYZRGBA, pcl::Normal> srcNormalSampling;
    srcNormalSampling.setInputCloud(cloud);
    srcNormalSampling.setNormals(normals);
    srcNormalSampling.setBins(normalSamplingBinSize_,
        normalSamplingBinSize_, normalSamplingBinSize_);
    srcNormalSampling.setSeed(0);
    srcNormalSampling.setSample(static_cast<unsigned int>
        (cloud->size()/(normalSamplingTotalPointsDivision_*subsamplingMultiplication)));
    srcNormalSampling.filter(removedIndices);

    log_->debug("Normals sumsampling finished");
    return removedIndices;
}

bool HandHeldMapper3d::computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) {
    struct timeval start, end;
    int64_t mtime, seconds, useconds;
    gettimeofday(&start, NULL);

    log_->debug("Computing normals");

    if (normalEstimationSearchRadius_ <= 0) {
            log_->error("normalEstimationSearchRadius should be greater than 0");
            return false;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr auxCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *auxCloud);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
    normal_estimation_filter.setInputCloud(auxCloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation_filter.setSearchMethod(search_tree);
    normal_estimation_filter.setRadiusSearch(normalEstimationSearchRadius_);
    normal_estimation_filter.compute(*cloud_normals);

    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    log_->info("Normals computing elapsed time: {0} milliseconds", mtime);
    return true;
}

boost::optional<pcl::PointCloud<pcl::Normal>::Ptr> HandHeldMapper3d::computeIntegralNormals(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
        float cameraPoseX, float cameraPoseY, float cameraPoseZ) {
    struct timeval start, end;
    int64_t mtime, seconds, useconds;
    gettimeofday(&start, NULL);

    log_->debug("Computing integral normals");
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> sourceNormalEstimation;

    sourceNormalEstimation.setNormalEstimationMethod(sourceNormalEstimation.AVERAGE_3D_GRADIENT);

    if (normalMaxDepthChangeFactor_ < 0) {
        log_->error("normalMaxDepthChangeFactor should be greater or equal to 0");
        return boost::none;
    }

    sourceNormalEstimation.setMaxDepthChangeFactor(normalMaxDepthChangeFactor_);
    sourceNormalEstimation.setNormalSmoothingSize(normalSmoothingSize_);
    sourceNormalEstimation.setInputCloud(cloud);
    sourceNormalEstimation.setViewPoint(cameraPoseX,
        cameraPoseY, cameraPoseZ);
    sourceNormalEstimation.compute(*sourceNormals);

    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    log_->info("Integral normals computing elapsed time: {0} milliseconds", mtime);
    return sourceNormals;
}

bool HandHeldMapper3d::computeFPFHKeypoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                 const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                 pcl::PointCloud<pcl::FPFHSignature33>::Ptr outputFeatures,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints) {
    if (multiscaleFeaturePersistenceScaleValue1_ == 0 ||
            multiscaleFeaturePersistenceScaleValue2_ == 0 ||
            multiscaleFeaturePersistenceScaleValue3_ == 0) {
            log_->error("MultiscaleFeaturePersistenceScaleValues could not be 0");
            return false;
    }
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr
        fpfh_estimation (new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal,
        pcl::FPFHSignature33>);
    fpfh_estimation->setInputCloud(cloud);
    fpfh_estimation->setInputNormals(normals);

    pcl::MultiscaleFeaturePersistence<pcl::PointXYZ, pcl::FPFHSignature33> feature_persistence;
    std::vector<float> scale_values;
    scale_values.push_back(multiscaleFeaturePersistenceScaleValue1_);
    scale_values.push_back(multiscaleFeaturePersistenceScaleValue2_);
    scale_values.push_back(multiscaleFeaturePersistenceScaleValue3_);
    feature_persistence.setScalesVector(scale_values);

    feature_persistence.setAlpha(FPFHPersintenceAlpha_);
    feature_persistence.setFeatureEstimator(fpfh_estimation);
    feature_persistence.setDistanceMetric(pcl::CS);
    pcl::IndicesPtr outputIndices(new std::vector<int> ());
    feature_persistence.determinePersistentFeatures(*outputFeatures, outputIndices);

    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_filter;
    extract_indices_filter.setInputCloud(cloud);
    extract_indices_filter.setIndices(outputIndices);
    extract_indices_filter.setNegative(false);
    extract_indices_filter.filter(*keypoints);
    return true;
}

bool HandHeldMapper3d::isFileExist(const std::string filename) {
    // Open the file
    FILE* f = fopen(filename.c_str(), "rb");
    // in the event of a failure, return false
    if (!f)
        return false;
    fclose(f);
    return true;
}

}  // namespace mapper3d
}  // namespace applications
}  // namespace crf
