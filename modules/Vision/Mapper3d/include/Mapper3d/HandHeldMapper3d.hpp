#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */
#include "EventLogger/EventLogger.hpp"
#include "Mapper3d/IMapper3d.hpp"
#include <Mapper3d/NormalsColorOctree.hpp>

#include <string>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <Eigen/Dense>
#include <vector>
#include <boost/optional.hpp>
#include <array>

namespace crf {
namespace applications {
namespace mapper3d {
class HandHeldMapper3d: public IMapper3d {
 public:
    HandHeldMapper3d() = delete;
    HandHeldMapper3d(const HandHeldMapper3d& other) = delete;
    HandHeldMapper3d(HandHeldMapper3d&& other) = delete;
    explicit HandHeldMapper3d(const std::string& configFileName);
    ~HandHeldMapper3d() override;
    boost::optional<Eigen::Matrix4f> updateMap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud,
        const Eigen::Matrix4f &sensorOrigin) override;
    boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> getPointCloudMap() override;
    boost::optional<NormalsColorOctree> getOctreeMap() override;
    bool clearMap() override;
    int getPointCloudMapSize() override;
    int getOctreeMapSize() override;
    bool savePointCloudToDisk(const std::string& savingPathWithoutExtension,
        bool plyflag) override;
    bool saveOctreeToDisk(const std::string& savingPathWithoutExtension) override;
    boost::optional<Eigen::Matrix4f> comparePointClouds(
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourcePointCloud,
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetPointCloud,
        const Eigen::Matrix4f &sourceSensorOrigin,
        const Eigen::Matrix4f &targetSensorOrigin) override;
    boost::optional<Eigen::Matrix4f> comparePointClouds(
        bool organizedPointCloud,
        bool srcNormalCalculated,
        bool tgtNormalCalculated,
        const Eigen::Matrix4f &sourceSensorOrigin,
        const Eigen::Matrix4f &targetSensorOrigin,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourcePointCloud,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetPointCloud,
        pcl::PointCloud<pcl::Normal>::Ptr sourceNormals,
        pcl::PointCloud<pcl::Normal>::Ptr targetNormals) override;

 private:
    crf::utility::logger::EventLogger log_;
    std::array<float, 12> accummulatedTransform_;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr currentCloudNormals_;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr fullCloudNormals_;
    std::unique_ptr<NormalsColorOctree> fullNormalsColorOctree_;

    // Configuration variables
    int maxNumberOfPoints_;
    bool updateOctree_;
    int maxNumberOfNodes_;
    float octreeResolutionInMeters_;
    bool enableChangeDetection_;
    float probabilityHit_;
    float probabilityMiss_;
    float clampingThresholdMin_;
    float clampingThresholdMax_;
    float occupancyThreshold_;
    bool fastBilateral_;
    bool passThrough_;
    bool voxelGrid_;
    bool statisticalOutlier_;
    bool mls_;
    float fastBilateralSigmaR_;
    float fastBilateralSigmaS_;
    float passThroughMinLimit_;
    float passThroughMaxLimit_;
    float voxelGridLeafSizeX_;
    float voxelGridLeafSizeY_;
    float voxelGridLeafSizeZ_;
    int statisticalOutlierMeanK_;
    float statisticalOutlierStddevMulThresh_;
    int mlsPolynomialOrder_;
    float mlsSearchRadius_;
    bool icpStepsVisualization_;
    bool icpSourceNormalsVisualization_;
    bool weightedTransformEstimation_;
    bool randomSampling_;
    float normalMaxDepthChangeFactor_;
    float normalSmoothingSize_;
    int normalSamplingTotalPointsDivision_;
    int randomlSamplingTotalPointsDivision_;
    int normalSamplingBinSize_;
    float correspondencesMaximumDistance_;
    float correspondencesMaximumMedianDistanceFactor_;
    float correspondencesMaximunAngleInRad_;
    double translationThreshold_;
    double rotationThreshold_;
    int maximumIterations_;
    bool icpInitialAlignment_;
    bool icpInitialAligmentVisualization_;
    float sampleConsensusInlierThreshold_;
    int sampleConsensusMaximumIterations_;
    bool sampleConsensusRefineModel_;
    float subsamplingSideLength_;
    float normalEstimationSearchRadius_;
    float FPFHPersintenceAlpha_;
    float multiscaleFeaturePersistenceScaleValue1_;
    float multiscaleFeaturePersistenceScaleValue2_;
    float multiscaleFeaturePersistenceScaleValue3_;

    boost::optional<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> checkForArtifactsInPointcloud(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
    bool insertFirstPointcloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &transformedCloud,
        const Eigen::Matrix4f &cameraPose);
    boost::optional<Eigen::Matrix4f> insertNextPointcloud(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &transformedCloud,
        const Eigen::Matrix4f &cameraPose,
        bool accummulateTransform);

    bool octreeUpdate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr newPointcloud,
        float originX, float originY, float originZ);
    boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>
        getOccupiedPointCloudFromOctree();

    boost::optional<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
        filterInputPointcloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rawCloud);
    boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>
        filterOutputPointcloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rawCloud);

    boost::optional<Eigen::Matrix4f> icpRefinement(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        srcCloud, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tgtCloud);
    boost::optional<Eigen::Matrix4f>
        icpInitialAligment(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr srcPointCloud,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tgtPointCloud);
    pcl::CorrespondencesPtr correspondenceEstimation(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputSource,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputTarget);
    pcl::CorrespondencesPtr correspondenceEStimationFPFHSignature33(
        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &sourceFeatures,
        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &targetFeatures);
    pcl::CorrespondencesPtr medianDistanceCorrespondeceRejection(
        const pcl::CorrespondencesPtr &initialCorrespondences);
    boost::optional<pcl::CorrespondencesPtr> surfaceNormalCorrespondeceRejection(
        const pcl::CorrespondencesPtr &initialCorrespondences,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputSource,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputTarget,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &inputNormals,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &targetNormals);
    boost::optional<pcl::CorrespondencesPtr> sampleConsensusCorrespondeceRejection(
        const pcl::CorrespondencesPtr &initialCorrespondences,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &sourceKeypoints,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &targetKeypoints);
    Eigen::Matrix4d transformEstimation(
        const pcl::CorrespondencesPtr &correspondences,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &sourcePointcloud,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &sourceNormals,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &targetNormals);

    bool subsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subsampled);
    boost::optional<std::vector<int>> randomSubsampling(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
    boost::optional<std::vector<int>> normalsSubsampling(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals,
        int subsamplingMultiplication);
    bool computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);
    boost::optional<pcl::PointCloud<pcl::Normal>::Ptr> computeIntegralNormals(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, float cameraPoseX,
        float cameraPoseY, float cameraPoseZ);
    bool computeFPFHKeypoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                 const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                 pcl::PointCloud<pcl::FPFHSignature33>::Ptr outputFeatures,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints);


    bool isFileExist(const std::string filename);
};
}  // namespace mapper3d
}  // namespace applications
}  // namespace crf
