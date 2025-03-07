#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include "EventLogger/EventLogger.hpp"

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/features/fpfh.h>
#include <pcl/impl/point_types.hpp>
#include <boost/optional.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>


namespace crf {
namespace utility {
namespace rgbdvisionutility {

class PCLUtils {
 public :
    static boost::optional<pcl::CorrespondencesPtr> correspondenceEstimation(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputSource,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputTarget,
        float correspondencesMaximumDistance);
    static boost::optional<pcl::CorrespondencesPtr>  medianDistanceCorrespondeceRejection(
        const pcl::CorrespondencesPtr &initialCorrespondences,
        float correspondencesMaximumMedianDistanceFactor);
    static boost::optional<pcl::CorrespondencesPtr> surfaceNormalCorrespondeceRejection(
        const pcl::CorrespondencesPtr &initialCorrespondences,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputSource,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputTarget,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &inputNormals,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &targetNormals,
        float correspondencesMaximunAngleInRad);
    static Eigen::Matrix4d transformEstimation(
        const pcl::CorrespondencesPtr &correspondences,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &sourcePointcloud,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &sourceNormals,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &targetNormals,
        bool weightedTransformEstimation,
        float passThroughMinLimit);
    static bool subsample(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud,
           pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_subsampled,
           float subsamplingSideLength);
    static boost::optional<std::vector<int>> normalsSubsampling(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals,
        int normalSamplingTotalPointsDivision,
        int normalSamplingBinSize);
    static boost::optional<std::vector<int>> covarianceSubsampling(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals,
        int covarianceSamplingTotalPointsDivision);
    static bool computeFPFHKeypoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                pcl::PointCloud<pcl::FPFHSignature33>::Ptr outputFeatures,
                pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints,
                float multiscaleFeaturePersistenceScaleValue1,
                float multiscaleFeaturePersistenceScaleValue2,
                float multiscaleFeaturePersistenceScaleValue3,
                float FPFHPersintenceAlpha);
    static bool computeShotColorKeypoints(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                pcl::PointCloud<pcl::SHOT1344>::Ptr outputFeatures,
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints,
                float SHOTColormultiscaleFeaturePersistenceScaleValue1,
                float SHOTColormultiscaleFeaturePersistenceScaleValue2,
                float SHOTColormultiscaleFeaturePersistenceScaleValue3,
                float SHOTColorPersintenceAlpha,
                int SHOTComputationThreads);
    static pcl::CorrespondencesPtr correspondenceEStimationFPFHSignature33(
        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &sourceFeatures,
        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &targetFeatures);
    static pcl::CorrespondencesPtr correspondenceEstimationShotColor(
        const pcl::PointCloud<pcl::SHOT1344>::Ptr &sourceFeatures,
        const pcl::PointCloud<pcl::SHOT1344>::Ptr &targetFeatures);
    static boost::optional<pcl::CorrespondencesPtr> sampleConsensusCorrespondeceRejection(
        const pcl::CorrespondencesPtr &initialCorrespondences,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &sourceKeypoints,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &targetKeypoints,
        float sampleConsensusInlierThreshold,
        int sampleConsensusMaximumIterations,
        bool sampleConsensusRefineMode);
    static pcl::PointCloud<pcl::PointXYZRGBA>::Ptr extractBoxInliers(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputSource,
        const std::array<pcl::PointXYZRGBA, 2> &box3D);

 private :
     static crf::utility::logger::EventLogger log_;

     PCLUtils();
};
}   // namespace rgbdvisionutility
}   // namespace utility
}   // namespace cern
