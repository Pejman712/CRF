/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
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

#include "EventLogger/EventLogger.hpp"
#include "ObjectPoseEstimator/ObjectPoseEstimator.hpp"
#include "ObjectPoseEstimator/IObjectPoseEstimator.hpp"
#include "ObjectPoseEstimator/PoseEstimationData.hpp"

using crf::applications::objectposeestimator::ObjectPoseEstimator;

using testing::_;
using testing::Return;

class ObjectPoseEstimatorShould: public ::testing::Test {
 protected:
    ObjectPoseEstimatorShould():
        logger_("ObjectPoseEstimatorShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("ObjectPoseEstimatorTest.cpp"));
        testDirName_ += "config/";
    }

    ~ObjectPoseEstimatorShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::string testDirName_;
    std::unique_ptr<crf::applications::objectposeestimator::IObjectPoseEstimator> sut_;
};

TEST_F(ObjectPoseEstimatorShould, throwExceptionWhenIncorrectConfigFileProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_badSyntax.json")),
        std::invalid_argument);
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_badKey.json")),
        std::invalid_argument);
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator("configFile_badFileName.json")),
        std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, notThrowExceptionWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
}

TEST_F(ObjectPoseEstimatorShould, throwExceptionWhenpassThroughMinLimitNotProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badpassThroughMinLimit.json")),
        std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, throwExceptionWhenBadClusterToleranceProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badClusterToleranceProvided.json")),
        std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, throwExceptionWhenBadMinClusterSizeProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badMinClusterSizeProvided.json")),
        std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, throwExceptionWhenBadMaxClusterSizeProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badMaxClusterSizeProvided.json")),
        std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould,
        throwExceptionWhenBadCovarianceSamplingTotalPointsDivisionProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badCovarianceSamplingTotalPointsDivision.json")),
        std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, throwExceptionWhenBadNormalSamplingTotalPointsDivisionProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badNormalSamplingTotalPointsDivision.json")),
        std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, throwExceptionWhenBadNormalSamplingBinSizeProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badNormalSamplingBinSize.json")),
        std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, notComputePoseIfBadSubSamplingSideLengthProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badSubsamplingSideLength.json")), std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould,
        notComputePoseIfBadShotColorMultiscaleFeaturePersistenceScaleValuesProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badShotColorMultiscaleFeaturePersistenceScaleValues.json")),
         std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, notComputePoseIfBadSHOTColorPersintenceAlphaProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badSHOTColorPersintenceAlpha.json")), std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, notComputePoseIfBadSHOTComputationThreadsProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badSHOTComputationThreads.json")), std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, notComputePoseIfBadFPFHPersintenceAlphaProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badFPFHPersintenceAlpha.json")), std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, notComputePoseIfBadConvergenceCriteriaParametersProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badConvergenceCriteriaParameters.json")),
        std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, notComputePoseIfBadNearestNeighborDistanceProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badNearestNeighborDistance.json")), std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, notComputePoseIfBadCorrespondencesMaximumDistanceProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badCorrespondencesMaximumDistance.json")),
        std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould,
        notComputePoseIfBadCorrespondencesMaximumMedianDistanceFactorProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badCorrespondencesMaximumMedianDistanceFactor.json")),
        std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, notComputePoseIfBadSampleConsensusInlierThresholdProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badSampleConsensusInlierThreshold.json")),
        std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, notComputePoseIfBadSampleConsensusMaximumIterationsProvided) {
    ASSERT_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_badSampleConsensusMaximumIterations.json")),
        std::invalid_argument);
}

TEST_F(ObjectPoseEstimatorShould, notComputePoseIfTargetPointCloudIsEmpty) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    auxCloud->clear();
    ASSERT_FALSE(sut_->computeTargetPose(auxCloud));
}

TEST_F(ObjectPoseEstimatorShould,
        DISABLED_computePoseIfTargetPointCloudIsNotEmptyAndConfigSetWithoutIcpRefinement) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
}

TEST_F(ObjectPoseEstimatorShould,
        DISABLED_computePoseIfTargetPointCloudIsNotEmptyAndConfigSetWithIcpRefinement) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_goodICPRefinement.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
}

TEST_F(ObjectPoseEstimatorShould,
        DISABLED_computePoseIfTargetPointCloudIsNotEmptyAndConfigSetWithoutSHOTColorFeatures) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_goodNoSHOTColorFeatures.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
}

TEST_F(ObjectPoseEstimatorShould,
        DISABLED_computePoseIfTargetPointCloudIsNotEmptyAndConfigSetWithoutWeightedTransformEstimation) { // NOLINT
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_goodNoWeightedTransformEstimation.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
}

TEST_F(ObjectPoseEstimatorShould,
        DISABLED_computePoseIfTargetPointCloudIsNotEmptyAndConfigSetWithCovarianceSubsampling) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_goodCovarianceSubsampling.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
}

TEST_F(ObjectPoseEstimatorShould, DISABLED_getBestFitnessScoreBeforeAndAfterComputePose) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    float bestFitnessScore = sut_->getBestFitnessScore();
    ASSERT_EQ(bestFitnessScore, 0.0f);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
    bestFitnessScore = sut_->getBestFitnessScore();
    ASSERT_GT(bestFitnessScore, 0.0f);
}

TEST_F(ObjectPoseEstimatorShould, notClearBestFitnessScoreIfComputedPoseNotCalledBefore) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    ASSERT_FALSE(sut_->clearBestFitnessScore());
}

TEST_F(ObjectPoseEstimatorShould, DISABLED_clearBestFitnessScoreIfComputedPoseCalledBefore) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
    ASSERT_TRUE(sut_->clearBestFitnessScore());
}

TEST_F(ObjectPoseEstimatorShould, DISABLED_getBestInverseFitnessScoreBeforeAndAfterComputePose) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    float bestinverseFitnessScore = sut_->getBestInverseFitnessScore();
    ASSERT_EQ(bestinverseFitnessScore, 0.0f);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
    bestinverseFitnessScore = sut_->getBestInverseFitnessScore();
    ASSERT_GT(bestinverseFitnessScore, 0.0f);
}

TEST_F(ObjectPoseEstimatorShould, notClearBestInverseFitnessScoreIfComputedPoseNotCalledBefore) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    ASSERT_FALSE(sut_->clearBestInverseFitnessScore());
}

TEST_F(ObjectPoseEstimatorShould, DISABLED_clearBestInverseFitnessScoreIfComputedPoseCalledBefore) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(
        testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
    ASSERT_TRUE(sut_->clearBestInverseFitnessScore());
}

TEST_F(ObjectPoseEstimatorShould, DISABLED_getBestComposeFitnessScoreBeforeAndAfterComputePose) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    float bestComposeFitnessScore = sut_->getBestComposeFitnessScore();
    ASSERT_EQ(bestComposeFitnessScore, 0.0f);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
    bestComposeFitnessScore = sut_->getBestComposeFitnessScore();
    ASSERT_GT(bestComposeFitnessScore, 0.0f);
}

TEST_F(ObjectPoseEstimatorShould, notClearBestComposeFitnessScoreIfComputedPoseNotCalledBefore) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    ASSERT_FALSE(sut_->clearBestComposeFitnessScore());
}

TEST_F(ObjectPoseEstimatorShould, DISABLED_clearBestComposeFitnessScoreIfComputedPoseCalledBefore) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
    ASSERT_TRUE(sut_->clearBestComposeFitnessScore());
}

TEST_F(ObjectPoseEstimatorShould, DISABLED_getBestPoseBeforeAndAfterComputePose) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    std::array<float, 12> initialPose = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
    std::array<float, 12> bestPose = sut_->getBestPose();
    ASSERT_EQ(bestPose, initialPose);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
    bestPose = sut_->getBestPose();
    ASSERT_NE(bestPose, initialPose);
}

TEST_F(ObjectPoseEstimatorShould, notClearBestPoseIfComputedPoseNotCalledBefore) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    ASSERT_FALSE(sut_->clearBestPose());
}

TEST_F(ObjectPoseEstimatorShould, DISABLED_clearBestPoseIfComputedPoseCalledBefore) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
    ASSERT_TRUE(sut_->clearBestPose());
}

TEST_F(ObjectPoseEstimatorShould, DISABLED_getCurrentObjectClusterAfterComputePose) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
    ASSERT_TRUE(sut_->getCurrentObjectCluster());
}

TEST_F(ObjectPoseEstimatorShould, notGetCurrentObjectClusterBeforeComputePose) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    ASSERT_FALSE(sut_->getCurrentObjectCluster());
}

TEST_F(ObjectPoseEstimatorShould, notClearCurrentObjectClusterIfCurrentObjectClusterIsEmpty) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    ASSERT_FALSE(sut_->clearCurrentObjectCluster());
}
TEST_F(ObjectPoseEstimatorShould, DISABLED_clearCurrentObjectClusterIfCurrentObjectClusterIsNotEmpty) {  // NOLINT
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
    ASSERT_TRUE(sut_->clearCurrentObjectCluster());
}

TEST_F(ObjectPoseEstimatorShould, getObjectModelIfPreScanObjectPathIsInitialized) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    ASSERT_TRUE(sut_->getObjectModel());
}

TEST_F(ObjectPoseEstimatorShould, DISABLED_getBestObject3DBoundingBoxBeforeAndAfterComputePose) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    crf::applications::objectposeestimator::Object3DBoundingBox best3DBoundingBox;
    crf::applications::objectposeestimator::Object3DBoundingBox initial3DBoundingBox;
    pcl::PointXYZRGBNormal zeroPoint;
    zeroPoint.x = 0.0;
    zeroPoint.y = 0.0;
    zeroPoint.z = 0.0;
    Eigen::Matrix3f rotationalMatrixOBB;
    rotationalMatrixOBB.setIdentity();
    initial3DBoundingBox.minPointOBB = zeroPoint;
    initial3DBoundingBox.maxPointOBB = zeroPoint;
    initial3DBoundingBox.positionOBB = zeroPoint;
    initial3DBoundingBox.rotationalMatrixOBB = rotationalMatrixOBB;

    best3DBoundingBox = sut_->getBestObject3DBoundingBox();
    ASSERT_EQ(best3DBoundingBox.rotationalMatrixOBB, initial3DBoundingBox.rotationalMatrixOBB);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
    best3DBoundingBox = sut_->getBestObject3DBoundingBox();
    ASSERT_NE(best3DBoundingBox.rotationalMatrixOBB, initial3DBoundingBox.rotationalMatrixOBB);
}

TEST_F(ObjectPoseEstimatorShould, DISABLED_clearBest3DObjectBoundingBoxIfComputePoseAlreadyCalled) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    ASSERT_TRUE(sut_->computeTargetPose(auxCloud));
    ASSERT_TRUE(sut_->clearBest3DObjectBoundingBox());
}

TEST_F(ObjectPoseEstimatorShould, dontClearBest3DObjectBoundingBoxIfComputePoseNotCalledBefore) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);
    ASSERT_FALSE(sut_->clearBest3DObjectBoundingBox());
}

TEST_F(ObjectPoseEstimatorShould, correctlySetAndUnsetICPRefinement) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimator(testDirName_ + "configFile_good.json")));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr auxCloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (testDirName_+
            "../data/testOrganizedCloud.pcd", *auxCloud), -1);

    ASSERT_TRUE(sut_->setICPRefinement(true));
    ASSERT_TRUE(sut_->setICPRefinement(false));
    ASSERT_TRUE(sut_->setICPRefinement(true));

    ASSERT_TRUE(sut_->setICPRefinement(false));
    ASSERT_TRUE(sut_->setICPRefinement(true));
    ASSERT_FALSE(sut_->setICPRefinement(true));

    ASSERT_TRUE(sut_->setICPRefinement(false));
    ASSERT_TRUE(sut_->setICPRefinement(true));
    ASSERT_TRUE(sut_->setICPRefinement(false));

    ASSERT_TRUE(sut_->setICPRefinement(true));
    ASSERT_TRUE(sut_->setICPRefinement(false));
    ASSERT_FALSE(sut_->setICPRefinement(false));
    ASSERT_TRUE(sut_->setICPRefinement(true));
}
