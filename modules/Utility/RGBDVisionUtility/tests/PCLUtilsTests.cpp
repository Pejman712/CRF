/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author:Sergio CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <pcl/common/transforms.h>

#include "EventLogger/EventLogger.hpp"
#include "RGBDVisionUtility/PCLUtils.hpp"

using crf::utility::rgbdvisionutility::PCLUtils;

using testing::_;
using testing::Return;

class PCLUtilsShould: public ::testing::Test {
 protected:
    PCLUtilsShould(): basePointCloud_(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
        logger_("PCLUtilsShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("PCLUtilsTests.cpp"));
        *basePointCloud_ = *pointCloudLoading();
    }

    ~PCLUtilsShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }


    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloudLoading() {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>());;
        pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
                "data/testOrganizedCloud.pcd", *cloud);
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformPointCloud(
        const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &inPointCloud) {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        Eigen::Matrix4f transform;
        transform << 1, 0, 0, 0.05,
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1;
        pcl::transformPointCloudWithNormals(*inPointCloud, *cloud, transform);
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr basePointCloud_;
    crf::utility::logger::EventLogger logger_;
    std::string testDirName_;
};

TEST_F(PCLUtilsShould, notEstimateCorrespondencesIfBadCorrespondencesMaximumDistanceProvided) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZRGBA>);
    float correspondencesMaximumDistance = -1.0f;
    pcl::copyPointCloud(*basePointCloud_, *inputSource);
    pcl::copyPointCloud(*transformPointCloud(basePointCloud_), *inputTarget);
    ASSERT_FALSE(PCLUtils::correspondenceEstimation(inputSource, inputTarget,
        correspondencesMaximumDistance));
}

TEST_F(PCLUtilsShould, EstimateCorrespondencesIfGoodCorrespondencesMaximumDistanceProvided) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZRGBA>);
    float correspondencesMaximumDistance = 1.0f;
    pcl::copyPointCloud(*basePointCloud_, *inputSource);
    pcl::copyPointCloud(*transformPointCloud(basePointCloud_), *inputTarget);
    ASSERT_TRUE(PCLUtils::correspondenceEstimation(inputSource, inputTarget,
        correspondencesMaximumDistance));
}

TEST_F(PCLUtilsShould, notComputeMedianDistanceCorrespondeceRejectionIfBadCorrespondencesMaximumMedianDistanceFactorProvided) { //NOLINT
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZRGBA>);
    float correspondencesMaximumDistance = 1.0f;
    pcl::copyPointCloud(*basePointCloud_, *inputSource);
    pcl::copyPointCloud(*transformPointCloud(basePointCloud_), *inputTarget);
    boost::optional<pcl::CorrespondencesPtr> boostCorrespondences =
        PCLUtils::correspondenceEstimation(inputSource, inputTarget,
            correspondencesMaximumDistance);
    pcl::CorrespondencesPtr correspondences = boostCorrespondences.get();
    float correspondencesMaximumMedianDistanceFactor = -1.0f;
    ASSERT_FALSE(PCLUtils::medianDistanceCorrespondeceRejection(correspondences,
        correspondencesMaximumMedianDistanceFactor));
}

TEST_F(PCLUtilsShould, computeMedianDistanceCorrespondeceRejectionIfBadCorrespondencesMaximumMedianDistanceFactorProvided) { //NOLINT
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZRGBA>);
    float correspondencesMaximumDistance = 1.0f;
    pcl::copyPointCloud(*basePointCloud_, *inputSource);
    pcl::copyPointCloud(*transformPointCloud(basePointCloud_), *inputTarget);
    boost::optional<pcl::CorrespondencesPtr> boostCorrespondences =
        PCLUtils::correspondenceEstimation(inputSource, inputTarget,
            correspondencesMaximumDistance);
    pcl::CorrespondencesPtr correspondences = boostCorrespondences.get();
    float correspondencesMaximumMedianDistanceFactor = 1.0f;
    ASSERT_TRUE(PCLUtils::medianDistanceCorrespondeceRejection(correspondences,
        correspondencesMaximumMedianDistanceFactor));
}

TEST_F(PCLUtilsShould,
    notComputeSurfaceNormalCorrespondeceRejectionIfNotEnoughPointstoEstimateTransform) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZRGBA>);
    float correspondencesMaximumDistance = 0.1f;
    pcl::copyPointCloud(*basePointCloud_, *inputSource);
    pcl::copyPointCloud(*transformPointCloud(basePointCloud_), *inputTarget);
    boost::optional<pcl::CorrespondencesPtr> boostCorrespondences =
        PCLUtils::correspondenceEstimation(inputSource, inputTarget,
        correspondencesMaximumDistance);
    pcl::CorrespondencesPtr correspondences = boostCorrespondences.get();
    float correspondencesMaximunAngleInRad = 0.0001f;

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
    }
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            targetNormals->push_back(normal);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr
        sourcePointNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr
        targetPointNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*inputSource, *sourceXYZ);
    pcl::copyPointCloud(*inputTarget, *targetXYZ);

    pcl::concatenateFields(*sourceXYZ, *sourceNormals, *sourcePointNormals);
    pcl::concatenateFields(*targetXYZ, *targetNormals, *targetPointNormals);

    ASSERT_FALSE(PCLUtils::surfaceNormalCorrespondeceRejection(correspondences,
        inputSource, inputTarget, sourcePointNormals, targetPointNormals,
        correspondencesMaximunAngleInRad));
}

TEST_F(PCLUtilsShould,
    computeSurfaceNormalCorrespondeceRejectionIfEnoughPointstoEstimateTransform) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZRGBA>);
    float correspondencesMaximumDistance = 1.0f;
    pcl::copyPointCloud(*basePointCloud_, *inputSource);
    pcl::copyPointCloud(*transformPointCloud(basePointCloud_), *inputTarget);
    boost::optional<pcl::CorrespondencesPtr> boostCorrespondences =
        PCLUtils::correspondenceEstimation(inputSource, inputTarget,
        correspondencesMaximumDistance);
    pcl::CorrespondencesPtr correspondences = boostCorrespondences.get();
    float correspondencesMaximunAngleInRad = 1.0f;

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
    }
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            targetNormals->push_back(normal);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr
        sourcePointNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr
        targetPointNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*inputSource, *sourceXYZ);
    pcl::copyPointCloud(*inputTarget, *targetXYZ);

    pcl::concatenateFields(*sourceXYZ, *sourceNormals, *sourcePointNormals);
    pcl::concatenateFields(*targetXYZ, *targetNormals, *targetPointNormals);

    ASSERT_TRUE(PCLUtils::surfaceNormalCorrespondeceRejection(correspondences, inputSource,
        inputTarget, sourcePointNormals, targetPointNormals, correspondencesMaximunAngleInRad));
}

TEST_F(PCLUtilsShould, returnIdentityMatrixFromTransformEstimationWhenBothPointcloudsAreEqual) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZRGBA>);
    float correspondencesMaximumDistance = 0.5f;
    pcl::copyPointCloud(*basePointCloud_, *inputSource);
    pcl::copyPointCloud(*basePointCloud_, *inputTarget);
    boost::optional<pcl::CorrespondencesPtr> boostCorrespondences =
        PCLUtils::correspondenceEstimation(inputSource, inputTarget,
        correspondencesMaximumDistance);
    pcl::CorrespondencesPtr correspondences = boostCorrespondences.get();

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
    }
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            targetNormals->push_back(normal);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr
        sourcePointNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr
        targetPointNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*inputSource, *sourceXYZ);
    pcl::copyPointCloud(*inputTarget, *targetXYZ);

    pcl::concatenateFields(*sourceXYZ, *sourceNormals, *sourcePointNormals);
    pcl::concatenateFields(*targetXYZ, *targetNormals, *targetPointNormals);

    bool weightedTransformEstimation = true;
    float passThroughMinLimit = 0.45f;

    ASSERT_EQ(PCLUtils::transformEstimation(correspondences, inputSource, sourcePointNormals,
        targetPointNormals, weightedTransformEstimation, passThroughMinLimit),
        Eigen::Matrix4d::Identity());
}

TEST_F(PCLUtilsShould,
    returnNotIdentityMatrixTransformMatrixFromTransformEstimationWhenBothPointcloudsAreDifferent) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZRGBA>);
    float correspondencesMaximumDistance = 0.1f;
    pcl::copyPointCloud(*basePointCloud_, *inputSource);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr basePointCloud2(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    *basePointCloud2 = *transformPointCloud(basePointCloud_);
    pcl::copyPointCloud(*basePointCloud2, *inputTarget);
    boost::optional<pcl::CorrespondencesPtr> boostCorrespondences =
        PCLUtils::correspondenceEstimation(inputSource, inputTarget,
        correspondencesMaximumDistance);
    pcl::CorrespondencesPtr correspondences = boostCorrespondences.get();

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
    }
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud2->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud2->points[i].normal_x;
            normal.normal_y = basePointCloud2->points[i].normal_y;
            normal.normal_z = basePointCloud2->points[i].normal_z;
            targetNormals->push_back(normal);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr
        sourcePointNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr
        targetPointNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*inputSource, *sourceXYZ);
    pcl::copyPointCloud(*inputTarget, *targetXYZ);

    pcl::concatenateFields(*sourceXYZ, *sourceNormals, *sourcePointNormals);
    pcl::concatenateFields(*targetXYZ, *targetNormals, *targetPointNormals);

    bool weightedTransformEstimation = true;
    float passThroughMinLimit = 0.45f;
    ASSERT_NE(PCLUtils::transformEstimation(correspondences, inputSource, sourcePointNormals,
        targetPointNormals, weightedTransformEstimation, passThroughMinLimit),
        Eigen::Matrix4d::Identity());
}

TEST_F(PCLUtilsShould, notSubsampleIfBadSubsamplingSideLengthProvided) {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr subsampledCloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    float subsamplingSideLength = -1.0f;
    ASSERT_FALSE(PCLUtils::subsample(basePointCloud_, subsampledCloud, subsamplingSideLength));
}

TEST_F(PCLUtilsShould, subsampleIfGoodSubsamplingSideLengthProvided) {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr subsampledCloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    float subsamplingSideLength = 1.0f;
    ASSERT_TRUE(PCLUtils::subsample(basePointCloud_, subsampledCloud, subsamplingSideLength));
}

TEST_F(PCLUtilsShould, notComputeNormalsSubsamplingIfBadNormalsSubsamplingParametersProvided) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
    }
    int normalSamplingTotalPointsDivision = 2;
    int normalSamplingBinSize = -1;
    ASSERT_FALSE(PCLUtils::normalsSubsampling(inputSource, sourceNormals,
        normalSamplingTotalPointsDivision, normalSamplingBinSize));
    normalSamplingTotalPointsDivision = -1;
    normalSamplingBinSize = 2;
    ASSERT_FALSE(PCLUtils::normalsSubsampling(inputSource, sourceNormals,
        normalSamplingTotalPointsDivision, normalSamplingBinSize));
}

TEST_F(PCLUtilsShould, computeNormalsSubsamplingIfGoodNormalsSubsamplingParametersProvided) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
    }
    int normalSamplingTotalPointsDivision = 2;
    int normalSamplingBinSize = 1;
    ASSERT_TRUE(PCLUtils::normalsSubsampling(inputSource, sourceNormals,
        normalSamplingTotalPointsDivision, normalSamplingBinSize));
}

TEST_F(PCLUtilsShould,
    notComputeCovarianceSubsamplingIfBadCovarianceSubsamplingParametersProvided) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
    }
    int covarianceSamplingTotalPointsDivision = -2;
    ASSERT_FALSE(PCLUtils::covarianceSubsampling(inputSource, sourceNormals,
        covarianceSamplingTotalPointsDivision));
}

TEST_F(PCLUtilsShould, computeCovarianceSubsamplingIfGoodCovarianceSubsamplingParametersProvided) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
    }
    int covarianceSamplingTotalPointsDivision = 1;
    ASSERT_TRUE(PCLUtils::covarianceSubsampling(inputSource, sourceNormals,
        covarianceSamplingTotalPointsDivision));
}

TEST_F(PCLUtilsShould, notComputeFPFHKeypointsIfBadFPFHKeypointsParametersProvided) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
    }
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
    float multiscaleFeaturePersistenceScaleValue1 = -0.1;
    float multiscaleFeaturePersistenceScaleValue2 = 0.2;
    float multiscaleFeaturePersistenceScaleValue3 = 0.3;
    float FPFHPersintenceAlpha = 0.1;
    ASSERT_FALSE(PCLUtils::computeFPFHKeypoints(inputSource, sourceNormals, features, keypoints,
        multiscaleFeaturePersistenceScaleValue1, multiscaleFeaturePersistenceScaleValue2,
        multiscaleFeaturePersistenceScaleValue3, FPFHPersintenceAlpha));
    multiscaleFeaturePersistenceScaleValue1 = 0.1;
    multiscaleFeaturePersistenceScaleValue2 = -0.2;
    multiscaleFeaturePersistenceScaleValue3 = 0.3;
    FPFHPersintenceAlpha = 0.1;
    ASSERT_FALSE(PCLUtils::computeFPFHKeypoints(inputSource, sourceNormals, features, keypoints,
        multiscaleFeaturePersistenceScaleValue1, multiscaleFeaturePersistenceScaleValue2,
        multiscaleFeaturePersistenceScaleValue3, FPFHPersintenceAlpha));
    multiscaleFeaturePersistenceScaleValue1 = 0.1;
    multiscaleFeaturePersistenceScaleValue2 = 0.2;
    multiscaleFeaturePersistenceScaleValue3 = -0.3;
    FPFHPersintenceAlpha = 0.1;
    ASSERT_FALSE(PCLUtils::computeFPFHKeypoints(inputSource, sourceNormals, features, keypoints,
        multiscaleFeaturePersistenceScaleValue1, multiscaleFeaturePersistenceScaleValue2,
        multiscaleFeaturePersistenceScaleValue3, FPFHPersintenceAlpha));
    multiscaleFeaturePersistenceScaleValue1 = 0.1;
    multiscaleFeaturePersistenceScaleValue2 = 0.2;
    multiscaleFeaturePersistenceScaleValue3 = 0.3;
    FPFHPersintenceAlpha = -0.1;
    ASSERT_FALSE(PCLUtils::computeFPFHKeypoints(inputSource, sourceNormals, features, keypoints,
        multiscaleFeaturePersistenceScaleValue1, multiscaleFeaturePersistenceScaleValue2,
        multiscaleFeaturePersistenceScaleValue3, FPFHPersintenceAlpha));
}

TEST_F(PCLUtilsShould, DISABLED_computeFPFHKeypointsIfGoodFPFHKeypointsParametersProvided) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
    }
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
    float multiscaleFeaturePersistenceScaleValue1 = 0.1;
    float multiscaleFeaturePersistenceScaleValue2 = 0.2;
    float multiscaleFeaturePersistenceScaleValue3 = 0.3;
    float FPFHPersintenceAlpha = 0.1;
    ASSERT_TRUE(PCLUtils::computeFPFHKeypoints(inputSource, sourceNormals, features, keypoints,
        multiscaleFeaturePersistenceScaleValue1, multiscaleFeaturePersistenceScaleValue2,
        multiscaleFeaturePersistenceScaleValue3, FPFHPersintenceAlpha));
}

TEST_F(PCLUtilsShould, notComputeShotColorKeypointsIfBadShotColorKeypointsParametersProvided) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
    }
    pcl::PointCloud<pcl::SHOT1344>::Ptr features(new pcl::PointCloud<pcl::SHOT1344>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
    float SHOTColormultiscaleFeaturePersistenceScaleValue1 = -0.1;
    float SHOTColormultiscaleFeaturePersistenceScaleValue2 = 0.2;
    float SHOTColormultiscaleFeaturePersistenceScaleValue3 = 0.3;
    float SHOTColorPersintenceAlpha = 0.1;
    int SHOTComputationThreads = 1;
    ASSERT_FALSE(PCLUtils::computeShotColorKeypoints(inputSource, sourceNormals, features,
        keypoints, SHOTColormultiscaleFeaturePersistenceScaleValue1,
        SHOTColormultiscaleFeaturePersistenceScaleValue2,
        SHOTColormultiscaleFeaturePersistenceScaleValue3,
        SHOTColorPersintenceAlpha, SHOTComputationThreads));
    SHOTColormultiscaleFeaturePersistenceScaleValue1 = 0.1;
    SHOTColormultiscaleFeaturePersistenceScaleValue2 = -0.2;
    SHOTColormultiscaleFeaturePersistenceScaleValue3 = 0.3;
    SHOTColorPersintenceAlpha = 0.1;
    SHOTComputationThreads = 1;
    ASSERT_FALSE(PCLUtils::computeShotColorKeypoints(inputSource, sourceNormals, features,
        keypoints, SHOTColormultiscaleFeaturePersistenceScaleValue1,
        SHOTColormultiscaleFeaturePersistenceScaleValue2,
        SHOTColormultiscaleFeaturePersistenceScaleValue3,
        SHOTColorPersintenceAlpha, SHOTComputationThreads));
    SHOTColormultiscaleFeaturePersistenceScaleValue1 = 0.1;
    SHOTColormultiscaleFeaturePersistenceScaleValue2 = 0.2;
    SHOTColormultiscaleFeaturePersistenceScaleValue3 = -0.3;
    SHOTColorPersintenceAlpha = 0.1;
    SHOTComputationThreads = 1;
    ASSERT_FALSE(PCLUtils::computeShotColorKeypoints(inputSource, sourceNormals, features,
        keypoints, SHOTColormultiscaleFeaturePersistenceScaleValue1,
        SHOTColormultiscaleFeaturePersistenceScaleValue2,
        SHOTColormultiscaleFeaturePersistenceScaleValue3,
        SHOTColorPersintenceAlpha, SHOTComputationThreads));
    SHOTColormultiscaleFeaturePersistenceScaleValue1 = 0.1;
    SHOTColormultiscaleFeaturePersistenceScaleValue2 = 0.2;
    SHOTColormultiscaleFeaturePersistenceScaleValue3 = 0.3;
    SHOTColorPersintenceAlpha = -0.1;
    SHOTComputationThreads = 1;
    ASSERT_FALSE(PCLUtils::computeShotColorKeypoints(inputSource, sourceNormals, features,
        keypoints, SHOTColormultiscaleFeaturePersistenceScaleValue1,
        SHOTColormultiscaleFeaturePersistenceScaleValue2,
        SHOTColormultiscaleFeaturePersistenceScaleValue3,
        SHOTColorPersintenceAlpha, SHOTComputationThreads));
    SHOTColormultiscaleFeaturePersistenceScaleValue1 = 0.1;
    SHOTColormultiscaleFeaturePersistenceScaleValue2 = 0.2;
    SHOTColormultiscaleFeaturePersistenceScaleValue3 = 0.3;
    SHOTColorPersintenceAlpha = 0.1;
    SHOTComputationThreads = 0;
    ASSERT_FALSE(PCLUtils::computeShotColorKeypoints(inputSource, sourceNormals, features,
        keypoints, SHOTColormultiscaleFeaturePersistenceScaleValue1,
        SHOTColormultiscaleFeaturePersistenceScaleValue2,
        SHOTColormultiscaleFeaturePersistenceScaleValue3,
        SHOTColorPersintenceAlpha, SHOTComputationThreads));
}

TEST_F(PCLUtilsShould, DISABLED_computeShotColorKeypointsIfGoodShotColorKeypointsParametersProvided) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
    }
    pcl::PointCloud<pcl::SHOT1344>::Ptr features(new pcl::PointCloud<pcl::SHOT1344>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
    float SHOTColormultiscaleFeaturePersistenceScaleValue1 = 0.1;
    float SHOTColormultiscaleFeaturePersistenceScaleValue2 = 0.2;
    float SHOTColormultiscaleFeaturePersistenceScaleValue3 = 0.3;
    float SHOTColorPersintenceAlpha = 0.1;
    int SHOTComputationThreads = 1;
    ASSERT_TRUE(PCLUtils::computeShotColorKeypoints(inputSource, sourceNormals, features,
        keypoints, SHOTColormultiscaleFeaturePersistenceScaleValue1,
        SHOTColormultiscaleFeaturePersistenceScaleValue2,
        SHOTColormultiscaleFeaturePersistenceScaleValue3,
        SHOTColorPersintenceAlpha, SHOTComputationThreads));
}

TEST_F(PCLUtilsShould, DISABLED_estimateFPFHSignature33CorrespondencesIfInputFeaturesNotEmpty) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*basePointCloud_, *inputTarget);

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
            targetNormals->push_back(normal);
    }

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFeatures(
        new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceKeypoints(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFeatures(
        new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
    float multiscaleFeaturePersistenceScaleValue1 = 0.1;
    float multiscaleFeaturePersistenceScaleValue2 = 0.2;
    float multiscaleFeaturePersistenceScaleValue3 = 0.3;
    float FPFHPersintenceAlpha = 0.1;
    ASSERT_TRUE(PCLUtils::computeFPFHKeypoints(inputSource, sourceNormals, sourceFeatures,
        sourceKeypoints, multiscaleFeaturePersistenceScaleValue1,
        multiscaleFeaturePersistenceScaleValue2,
        multiscaleFeaturePersistenceScaleValue3,
        FPFHPersintenceAlpha));

    ASSERT_TRUE(PCLUtils::computeFPFHKeypoints(inputTarget, targetNormals, targetFeatures,
        targetKeypoints, multiscaleFeaturePersistenceScaleValue1,
        multiscaleFeaturePersistenceScaleValue2,
        multiscaleFeaturePersistenceScaleValue3,
        FPFHPersintenceAlpha));

    pcl::CorrespondencesPtr correspondences = PCLUtils::correspondenceEStimationFPFHSignature33(
        sourceFeatures, targetFeatures);
    ASSERT_NE(correspondences->size(), 0);
}


TEST_F(PCLUtilsShould, DISABLED_notEstimateFPFHSignature33CorrespondencesIfInputFeaturesAreEmpty) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*basePointCloud_, *inputTarget);

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
            targetNormals->push_back(normal);
    }

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFeatures(
        new pcl::PointCloud<pcl::FPFHSignature33>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFeatures(
        new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
    float multiscaleFeaturePersistenceScaleValue1 = 0.1;
    float multiscaleFeaturePersistenceScaleValue2 = 0.2;
    float multiscaleFeaturePersistenceScaleValue3 = 0.3;
    float FPFHPersintenceAlpha = 0.1;
    ASSERT_TRUE(PCLUtils::computeFPFHKeypoints(inputSource, sourceNormals, sourceFeatures,
        sourceKeypoints, multiscaleFeaturePersistenceScaleValue1,
        multiscaleFeaturePersistenceScaleValue2,
        multiscaleFeaturePersistenceScaleValue3,
        FPFHPersintenceAlpha));

    pcl::CorrespondencesPtr correspondences = PCLUtils::correspondenceEStimationFPFHSignature33(
        sourceFeatures, targetFeatures);
    ASSERT_EQ(correspondences->size(), 0);
}


TEST_F(PCLUtilsShould, DISABLED_estimateShotColorCorrespondencesIfInputFeaturesNotEmpty) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*basePointCloud_, *inputTarget);

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
            targetNormals->push_back(normal);
    }

    pcl::PointCloud<pcl::SHOT1344>::Ptr sourceFeatures(new pcl::PointCloud<pcl::SHOT1344>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourceKeypoints(
        new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::PointCloud<pcl::SHOT1344>::Ptr targetFeatures(new pcl::PointCloud<pcl::SHOT1344>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetKeypoints(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    float SHOTColormultiscaleFeaturePersistenceScaleValue1 = 0.1;
    float SHOTColormultiscaleFeaturePersistenceScaleValue2 = 0.2;
    float SHOTColormultiscaleFeaturePersistenceScaleValue3 = 0.3;
    float SHOTColorPersintenceAlpha = 0.1;
    int SHOTComputationThreads = 1;
    ASSERT_TRUE(PCLUtils::computeShotColorKeypoints(inputSource, sourceNormals, sourceFeatures,
        sourceKeypoints, SHOTColormultiscaleFeaturePersistenceScaleValue1,
        SHOTColormultiscaleFeaturePersistenceScaleValue2,
        SHOTColormultiscaleFeaturePersistenceScaleValue3,
        SHOTColorPersintenceAlpha, SHOTComputationThreads));
    ASSERT_TRUE(PCLUtils::computeShotColorKeypoints(inputTarget, targetNormals, targetFeatures,
        targetKeypoints, SHOTColormultiscaleFeaturePersistenceScaleValue1,
        SHOTColormultiscaleFeaturePersistenceScaleValue2,
        SHOTColormultiscaleFeaturePersistenceScaleValue3,
        SHOTColorPersintenceAlpha, SHOTComputationThreads));

    pcl::CorrespondencesPtr correspondences = PCLUtils::correspondenceEstimationShotColor(
        sourceFeatures, targetFeatures);
    ASSERT_NE(correspondences->size(), 0);
}

TEST_F(PCLUtilsShould, DISABLED_notEstimateShotColorCorrespondencesIfInputFeaturesAreEmpty) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::copyPointCloud(*basePointCloud_, *inputSource);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*basePointCloud_, *inputTarget);

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
            targetNormals->push_back(normal);
    }

    pcl::PointCloud<pcl::SHOT1344>::Ptr sourceFeatures(new pcl::PointCloud<pcl::SHOT1344>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourceKeypoints(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::SHOT1344>::Ptr targetFeatures(new pcl::PointCloud<pcl::SHOT1344>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetKeypoints(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    float SHOTColormultiscaleFeaturePersistenceScaleValue1 = 0.1;
    float SHOTColormultiscaleFeaturePersistenceScaleValue2 = 0.2;
    float SHOTColormultiscaleFeaturePersistenceScaleValue3 = 0.3;
    float SHOTColorPersintenceAlpha = 0.1;
    int SHOTComputationThreads = 1;
    ASSERT_TRUE(PCLUtils::computeShotColorKeypoints(inputSource, sourceNormals, sourceFeatures,
        sourceKeypoints, SHOTColormultiscaleFeaturePersistenceScaleValue1,
        SHOTColormultiscaleFeaturePersistenceScaleValue2,
        SHOTColormultiscaleFeaturePersistenceScaleValue3,
        SHOTColorPersintenceAlpha, SHOTComputationThreads));

    pcl::CorrespondencesPtr correspondences = PCLUtils::correspondenceEstimationShotColor(

        sourceFeatures, targetFeatures);
    ASSERT_EQ(correspondences->size(), 0);
}


TEST_F(PCLUtilsShould, DISABLED_computeSampleConsensusCorrespondeceRejectionIfGoodParametersProvided) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*basePointCloud_, *inputTarget);

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
            targetNormals->push_back(normal);
    }

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFeatures(
        new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFeatures(
        new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
    float multiscaleFeaturePersistenceScaleValue1 = 0.1;
    float multiscaleFeaturePersistenceScaleValue2 = 0.2;
    float multiscaleFeaturePersistenceScaleValue3 = 0.3;
    float FPFHPersintenceAlpha = 0.1;
    ASSERT_TRUE(PCLUtils::computeFPFHKeypoints(inputSource, sourceNormals, sourceFeatures,
        sourceKeypoints, multiscaleFeaturePersistenceScaleValue1,
        multiscaleFeaturePersistenceScaleValue2,
        multiscaleFeaturePersistenceScaleValue3,
        FPFHPersintenceAlpha));

    ASSERT_TRUE(PCLUtils::computeFPFHKeypoints(inputTarget, targetNormals, targetFeatures,
        targetKeypoints, multiscaleFeaturePersistenceScaleValue1,
        multiscaleFeaturePersistenceScaleValue2,
        multiscaleFeaturePersistenceScaleValue3,
        FPFHPersintenceAlpha));

    pcl::CorrespondencesPtr correspondences = PCLUtils::correspondenceEStimationFPFHSignature33(
        sourceFeatures, targetFeatures);
    ASSERT_NE(correspondences->size(), 0);


    float sampleConsensusInlierThreshold = 0.02;
    int sampleConsensusMaximumIterations = 100;
    bool sampleConsensusRefineModel = true;
    ASSERT_TRUE(PCLUtils::sampleConsensusCorrespondeceRejection(correspondences, sourceKeypoints,
        targetKeypoints, sampleConsensusInlierThreshold,
        sampleConsensusMaximumIterations, sampleConsensusRefineModel));

    sampleConsensusInlierThreshold = 0.02;
    sampleConsensusMaximumIterations = 100;
    sampleConsensusRefineModel = false;
    ASSERT_TRUE(PCLUtils::sampleConsensusCorrespondeceRejection(correspondences, sourceKeypoints,
        targetKeypoints, sampleConsensusInlierThreshold,
        sampleConsensusMaximumIterations, sampleConsensusRefineModel));
}


TEST_F(PCLUtilsShould, DISABLED_notComputeSampleConsensusCorrespondeceRejectionIfBadParametersProvided) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*basePointCloud_, *inputTarget);

    // Create separate clouds from source full cloud
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < static_cast<int>(basePointCloud_->size()); i++) {
            pcl::Normal normal;
            normal.normal_x = basePointCloud_->points[i].normal_x;
            normal.normal_y = basePointCloud_->points[i].normal_y;
            normal.normal_z = basePointCloud_->points[i].normal_z;
            sourceNormals->push_back(normal);
            targetNormals->push_back(normal);
    }

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFeatures(
        new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFeatures(
        new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
    float multiscaleFeaturePersistenceScaleValue1 = 0.1;
    float multiscaleFeaturePersistenceScaleValue2 = 0.2;
    float multiscaleFeaturePersistenceScaleValue3 = 0.3;
    float FPFHPersintenceAlpha = 0.1;
    ASSERT_TRUE(PCLUtils::computeFPFHKeypoints(inputSource, sourceNormals, sourceFeatures,
        sourceKeypoints, multiscaleFeaturePersistenceScaleValue1,
        multiscaleFeaturePersistenceScaleValue2,
        multiscaleFeaturePersistenceScaleValue3, FPFHPersintenceAlpha));

    ASSERT_TRUE(PCLUtils::computeFPFHKeypoints(inputTarget, targetNormals, targetFeatures,
        targetKeypoints, multiscaleFeaturePersistenceScaleValue1,
        multiscaleFeaturePersistenceScaleValue2,
        multiscaleFeaturePersistenceScaleValue3, FPFHPersintenceAlpha));

    pcl::CorrespondencesPtr correspondences = PCLUtils::correspondenceEStimationFPFHSignature33(
        sourceFeatures, targetFeatures);
    ASSERT_NE(correspondences->size(), 0);


    float sampleConsensusInlierThreshold = -0.02;
    int sampleConsensusMaximumIterations = 100;
    bool sampleConsensusRefineModel = true;
    ASSERT_FALSE(PCLUtils::sampleConsensusCorrespondeceRejection(correspondences, sourceKeypoints,
        targetKeypoints, sampleConsensusInlierThreshold,
        sampleConsensusMaximumIterations, sampleConsensusRefineModel));

    sampleConsensusInlierThreshold = 0.02;
    sampleConsensusMaximumIterations = -100;
    ASSERT_FALSE(PCLUtils::sampleConsensusCorrespondeceRejection(correspondences, sourceKeypoints,
        targetKeypoints, sampleConsensusInlierThreshold,
        sampleConsensusMaximumIterations, sampleConsensusRefineModel));

    correspondences->clear();
    sampleConsensusInlierThreshold = 0.02;
    sampleConsensusMaximumIterations = 100;
    ASSERT_FALSE(PCLUtils::sampleConsensusCorrespondeceRejection(correspondences, sourceKeypoints,
        targetKeypoints, sampleConsensusInlierThreshold,
        sampleConsensusMaximumIterations, sampleConsensusRefineModel));
}

TEST_F(PCLUtilsShould, keepAllOutliers) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);
    std::array<pcl::PointXYZRGBA, 2> robotBox;
    pcl::PointXYZRGBA referencePoint;
    referencePoint.x = -1;
    referencePoint.y = .0;
    referencePoint.z = .0;
    robotBox[0] = referencePoint;
    referencePoint.x = 1;
    referencePoint.y = 1;
    robotBox[1] = referencePoint;
    auto resultCloud = PCLUtils::extractBoxInliers(inputSource, robotBox);
    ASSERT_EQ(inputSource->size(), resultCloud->size());
}

TEST_F(PCLUtilsShould, removeInliersFromPointCloud) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*basePointCloud_, *inputSource);
    std::array<pcl::PointXYZRGBA, 2> robotBox;
    pcl::PointXYZRGBA referencePoint;
    referencePoint.x = 3.1;
    referencePoint.y = 9.0;
    referencePoint.z = .0;
    robotBox[0] = referencePoint;
    pcl::PointXYZRGBA inlierPoint;
    inlierPoint.x = 5;
    inlierPoint.y = 9.05;
    inlierPoint.z = 0.011;
    inputSource->points.push_back(inlierPoint);
    pcl::PointXYZRGBA referencePoint2;
    referencePoint2.x = 5.1;
    referencePoint2.y = 9.1;
    referencePoint2.z = 0.2;
    robotBox[1] = referencePoint2;
    auto resultCloud = PCLUtils::extractBoxInliers(inputSource, robotBox);
    ASSERT_EQ(basePointCloud_->size(), resultCloud->size());
}





