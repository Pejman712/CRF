/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */
#define PCL_NO_PRECOMPILE
#include "RGBDVisionUtility/PCLUtils.hpp"
#include "EventLogger/EventLogger.hpp"

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/features/shot_omp.h>

namespace crf {
namespace utility {
namespace rgbdvisionutility {

crf::utility::logger::EventLogger PCLUtils::log_("PCLUtils");

boost::optional<pcl::CorrespondencesPtr> PCLUtils::correspondenceEstimation(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputSource,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputTarget,
        float correspondencesMaximumDistance) {
        log_->debug("Correspondence estimation");
        pcl::CorrespondencesPtr correspondences (new pcl::Correspondences ());
        pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGBA,
            pcl::PointXYZRGBA> correspKdtree;
        correspKdtree.setInputSource(inputSource);
        correspKdtree.setInputTarget(inputTarget);
        if (correspondencesMaximumDistance < 0) {
            log_->error("CorrespondencesMaximumDistance should be greater than 0");
            return boost::none;
        }
        correspKdtree.determineCorrespondences(*correspondences,
            correspondencesMaximumDistance);
        log_->debug("Remaining Correspondences after distance rejection: {0}",
            correspondences->size());

        return correspondences;
}

boost::optional<pcl::CorrespondencesPtr> PCLUtils::medianDistanceCorrespondeceRejection(
        const pcl::CorrespondencesPtr &initialCorrespondences,
        float correspondencesMaximumMedianDistanceFactor) {
    log_->debug("Median Distance Correspondece Rejection estimation");
    pcl::CorrespondencesPtr remainingCorrespondences (new pcl::Correspondences ());
    pcl::registration::CorrespondenceRejectorMedianDistance medDistanceRej;
    if (correspondencesMaximumMedianDistanceFactor< 0) {
        log_->error("correspondencesMaximumMedianDistanceFactor should be greater than 0");
        return boost::none;
    }
    medDistanceRej.setMedianFactor(correspondencesMaximumMedianDistanceFactor);
    medDistanceRej.setInputCorrespondences(initialCorrespondences);
    medDistanceRej.getCorrespondences(*remainingCorrespondences);
    log_->debug("Remaining Correspondences after median distance rejection: {0}",
        remainingCorrespondences->size());

    return remainingCorrespondences;
}

boost::optional<pcl::CorrespondencesPtr> PCLUtils::surfaceNormalCorrespondeceRejection(
        const pcl::CorrespondencesPtr &initialCorrespondences,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputSource,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputTarget,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &inputNormals,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &targetNormals,
        float correspondencesMaximunAngleInRad) {
    log_->debug("Surface Normal Correspondece Rejection estimation");
    pcl::CorrespondencesPtr remainingCorrespondences (new pcl::Correspondences ());
    pcl::registration::CorrespondenceRejectorSurfaceNormal normalRej;
    normalRej.initializeDataContainer<pcl::PointXYZRGBA, pcl::PointNormal>();
    normalRej.setInputSource<pcl::PointXYZRGBA>(inputSource);
    normalRej.setInputTarget<pcl::PointXYZRGBA>(inputTarget);
    normalRej.setInputNormals<pcl::PointXYZRGBA, pcl::PointNormal>(inputNormals);
    normalRej.setTargetNormals<pcl::PointXYZRGBA, pcl::PointNormal>(targetNormals);
    normalRej.setInputCorrespondences(initialCorrespondences);
    float fthres = cos(correspondencesMaximunAngleInRad);
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

Eigen::Matrix4d PCLUtils::transformEstimation(
        const pcl::CorrespondencesPtr &correspondences,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &sourcePointcloud,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &sourceNormals,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &targetNormals,
        bool weightedTransformEstimation,
        float passThroughMinLimit) {
    log_->debug("Transform Estimation");

    Eigen::Matrix4d estimatedTransform(Eigen::Matrix4d::Identity());
    pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal,
        pcl::PointNormal, double> transformEstimator;
    pcl::registration::TransformationEstimationPointToPlaneWeighted<pcl::PointNormal,
        pcl::PointNormal, double> weightedTransformEstimator;

    // Compute weights using common RGBD camera noise model
    if (weightedTransformEstimation) {
        std::vector< double > weights(correspondences->size());
        for (int i = 0; i < static_cast<int>(correspondences->size()); i++) {
            float depth = sourcePointcloud->points[(*correspondences)[i].index_match].z;
            float aux = (depth - passThroughMinLimit) * (
                depth - passThroughMinLimit);
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


bool PCLUtils::subsample(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud,
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_subsampled,
        float subsamplingSideLength) {
    if (subsamplingSideLength <= 0) {
            log_->error("subsamplingSideLength should be greater than 0");
            return false;
    }
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::copyPointCloud(*cloud, *auxCloud);
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> subsampling_filter;
    subsampling_filter.setInputCloud(auxCloud);
    subsampling_filter.setLeafSize(subsamplingSideLength,
        subsamplingSideLength, subsamplingSideLength);
    subsampling_filter.filter(*cloud_subsampled);
    return true;
}

boost::optional<std::vector<int>> PCLUtils::normalsSubsampling(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals,
        int normalSamplingTotalPointsDivision,
        int normalSamplingBinSize) {
    log_->debug("Normals subsampling");
    std::vector<int> removedIndices;

    if (normalSamplingTotalPointsDivision <= 1) {
        log_->error("normalSamplingTotalPointsDivision should be greater than 0");
        return boost::none;
    }
    if (normalSamplingBinSize <= 0) {
        log_->error("normalSamplingBinSize should be greater than 0");
        return boost::none;
    }
    // Source Normal space subsampling
    pcl::NormalSpaceSampling<pcl::PointXYZRGBA, pcl::Normal> srcNormalSampling;
    srcNormalSampling.setInputCloud(cloud);
    srcNormalSampling.setNormals(normals);
    srcNormalSampling.setBins(normalSamplingBinSize,
        normalSamplingBinSize, normalSamplingBinSize);
    srcNormalSampling.setSeed(0);
    srcNormalSampling.setSample(static_cast<unsigned int>
        (cloud->size())/normalSamplingTotalPointsDivision);
    srcNormalSampling.filter(removedIndices);

    log_->debug("Normals sumsampling finished");
    return removedIndices;
}

boost::optional<std::vector<int>> PCLUtils::covarianceSubsampling(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals,
        int covarianceSamplingTotalPointsDivision) {
    log_->debug("Covariance subsampling");
    std::vector<int> removedIndices;

    if (covarianceSamplingTotalPointsDivision <= 0) {
        log_->error("covarianceSamplingTotalPointsDivision should be greater than 0");
        return boost::none;
    }
    // Source Normal space subsampling
    pcl::CovarianceSampling<pcl::PointXYZRGBA, pcl::Normal> srcCovarianceSampling;
    srcCovarianceSampling.setNumberOfSamples(
        cloud->size()/covarianceSamplingTotalPointsDivision);
    srcCovarianceSampling.setInputCloud(cloud);
    srcCovarianceSampling.setNormals(normals);
    srcCovarianceSampling.filter(removedIndices);

    log_->debug("Covariance sumsampling finished");
    return removedIndices;
}

bool PCLUtils::computeFPFHKeypoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr outputFeatures,
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints,
    float multiscaleFeaturePersistenceScaleValue1,
    float multiscaleFeaturePersistenceScaleValue2,
    float multiscaleFeaturePersistenceScaleValue3,
    float FPFHPersintenceAlpha) {
    if (multiscaleFeaturePersistenceScaleValue1 <= 0.||
        multiscaleFeaturePersistenceScaleValue2 <= 0 ||
        multiscaleFeaturePersistenceScaleValue3 <= 0) {
            log_->error("MultiscaleFeaturePersistenceScaleValues should be greater than 0");
            return false;
    }

    if (FPFHPersintenceAlpha <= 0) {
        log_->error("FPFHPersintenceAlpha should be greater than 0");
        return false;
    }

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr
        fpfh_estimation (new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal,
        pcl::FPFHSignature33>);
    fpfh_estimation->setInputCloud(cloud);
    fpfh_estimation->setInputNormals(normals);

    pcl::MultiscaleFeaturePersistence<pcl::PointXYZ, pcl::FPFHSignature33> feature_persistence;
    std::vector<float> scale_values;
    scale_values.push_back(multiscaleFeaturePersistenceScaleValue1);
    scale_values.push_back(multiscaleFeaturePersistenceScaleValue2);
    scale_values.push_back(multiscaleFeaturePersistenceScaleValue3);
    feature_persistence.setScalesVector(scale_values);

    feature_persistence.setAlpha(FPFHPersintenceAlpha);
    feature_persistence.setFeatureEstimator(fpfh_estimation);
    feature_persistence.setDistanceMetric(pcl::KL);
    pcl::IndicesPtr outputIndices(new std::vector<int> ());
    feature_persistence.determinePersistentFeatures(*outputFeatures, outputIndices);

    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_filter;
    extract_indices_filter.setInputCloud(cloud);
    extract_indices_filter.setIndices(outputIndices);
    extract_indices_filter.setNegative(false);
    extract_indices_filter.filter(*keypoints);
    return true;
}

bool PCLUtils::computeShotColorKeypoints(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    pcl::PointCloud<pcl::SHOT1344>::Ptr outputFeatures,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints,
    float SHOTColormultiscaleFeaturePersistenceScaleValue1,
    float SHOTColormultiscaleFeaturePersistenceScaleValue2,
    float SHOTColormultiscaleFeaturePersistenceScaleValue3,
    float SHOTColorPersintenceAlpha,
    int SHOTComputationThreads) {
    if (SHOTColormultiscaleFeaturePersistenceScaleValue1 <= 0 ||
        SHOTColormultiscaleFeaturePersistenceScaleValue2 <= 0 ||
        SHOTColormultiscaleFeaturePersistenceScaleValue3 <= 0) {
        log_->error("SHOTColorMultiscaleFeaturePersistenceScaleValues should be greater than 0");
        return false;
    }
    if (SHOTColorPersintenceAlpha <= 0) {
        log_->error("SHOTColorPersintenceAlpha should be greater than 0");
        return false;
    }
    if (SHOTComputationThreads < 1) {
        log_->error("SHOTComputationThreads should be greater or equal to 1");
        return false;
    }

    pcl::SHOTColorEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT1344>::Ptr
        shot(new pcl::SHOTColorEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT1344>);
    shot->setInputCloud(cloud);
    shot->setInputNormals(normals);
    shot->setNumberOfThreads(SHOTComputationThreads);

    pcl::MultiscaleFeaturePersistence<pcl::PointXYZRGBA, pcl::SHOT1344> feature_persistence;
    std::vector<float> scale_values;
    scale_values.push_back(SHOTColormultiscaleFeaturePersistenceScaleValue1);
    scale_values.push_back(SHOTColormultiscaleFeaturePersistenceScaleValue2);
    scale_values.push_back(SHOTColormultiscaleFeaturePersistenceScaleValue3);
    feature_persistence.setScalesVector(scale_values);

    feature_persistence.setAlpha(SHOTColorPersintenceAlpha);
    feature_persistence.setFeatureEstimator(shot);
    feature_persistence.setDistanceMetric(pcl::KL);
    pcl::IndicesPtr outputIndices(new std::vector<int> ());
    feature_persistence.determinePersistentFeatures(*outputFeatures, outputIndices);

    pcl::ExtractIndices<pcl::PointXYZRGBA> extract_indices_filter;
    extract_indices_filter.setInputCloud(cloud);
    extract_indices_filter.setIndices(outputIndices);
    extract_indices_filter.setNegative(false);
    extract_indices_filter.filter(*keypoints);

    return true;
}

pcl::CorrespondencesPtr PCLUtils::correspondenceEStimationFPFHSignature33(
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

pcl::CorrespondencesPtr PCLUtils::correspondenceEstimationShotColor(
    const pcl::PointCloud<pcl::SHOT1344>::Ptr &sourceFeatures,
    const pcl::PointCloud<pcl::SHOT1344>::Ptr &targetFeatures) {
    log_->debug("SHOT1344 Correspondence estimation");

    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences ());
    pcl::registration::CorrespondenceEstimation<pcl::SHOT1344, pcl::SHOT1344> est;
    est.setInputSource(sourceFeatures);
    est.setInputTarget(targetFeatures);
    est.determineCorrespondences(*correspondences);
    log_->debug("SHOT1344 Correspondences size: {0}", correspondences->size());
    return correspondences;
}

boost::optional<pcl::CorrespondencesPtr> PCLUtils::sampleConsensusCorrespondeceRejection(
    const pcl::CorrespondencesPtr &initialCorrespondences,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &sourceKeypoints,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &targetKeypoints,
    float sampleConsensusInlierThreshold,
    int sampleConsensusMaximumIterations,
    bool sampleConsensusRefineModel) {
    log_->debug("Sample Consensus Correspondece Rejection");
    pcl::CorrespondencesPtr remainingCorrespondences (new pcl::Correspondences());
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejectorSac;
    rejectorSac.setInputSource(sourceKeypoints);
    rejectorSac.setInputTarget(targetKeypoints);
    if (sampleConsensusInlierThreshold <= 0) {
        log_->error("SampleConsensusInlierThreshold should be greater than 0");
        return boost::none;
    }
    rejectorSac.setInlierThreshold(sampleConsensusInlierThreshold);
    if (sampleConsensusMaximumIterations < 1) {
        log_->error("SampleConsensusMaximumIterations should be equal or greater than 1");
        return boost::none;
    }
    rejectorSac.setMaximumIterations(sampleConsensusMaximumIterations);
    rejectorSac.setRefineModel(sampleConsensusRefineModel);
    rejectorSac.setInputCorrespondences(initialCorrespondences);;
    rejectorSac.getCorrespondences(*remainingCorrespondences);

    if (remainingCorrespondences->size() < 4) {
        log_->error("Not enough points to estimate an initial transform");
        log_->info("Tune Initial aligment parameters");
        return boost::none;
    }

    return remainingCorrespondences;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCLUtils::extractBoxInliers(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputSource,
        const std::array<pcl::PointXYZRGBA, 2> &box3D) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::CropBox<pcl::PointXYZRGBA> boxFilter(true);
    pcl::PointXYZRGBA lowerLimit = box3D[0];
    pcl::PointXYZRGBA upperLimit = box3D[1];
    boxFilter.setInputCloud(inputSource);
    boxFilter.setMin(Eigen::Vector4f(lowerLimit.x, lowerLimit.y, lowerLimit.z, 1.0));
    boxFilter.setMax(Eigen::Vector4f(upperLimit.x, upperLimit.y, upperLimit.z, 1.0));
    boxFilter.setNegative(true);
    boxFilter.filter(*outputCloud);
    log_->info("Extracted {} indices", boxFilter.getRemovedIndices()->size());
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud(inputSource);
    extract.setIndices(boxFilter.getRemovedIndices());
    extract.setNegative(true);
    extract.filter(*outputCloud);
    return outputCloud;
}

}   // namespace rgbdvisionutility
}   // namespace utility
}   // namespace cern
