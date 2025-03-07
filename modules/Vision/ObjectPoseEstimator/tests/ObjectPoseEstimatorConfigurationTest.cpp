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
#include "ObjectPoseEstimator/ObjectPoseEstimatorConfiguration.hpp"

using crf::applications::objectposeestimator::ObjectPoseEstimatorConfiguration;

using testing::_;
using testing::Return;

class ObjectPoseEstimatorConfigurationShould: public ::testing::Test {
 protected:
    ObjectPoseEstimatorConfigurationShould():
        logger_("ObjectPoseEstimatorConfigurationShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find(
            "ObjectPoseEstimatorConfigurationTest.cpp"));
        testDirName_ += "config/";
    }

    ~ObjectPoseEstimatorConfigurationShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::string testDirName_;
    std::unique_ptr<crf::applications::objectposeestimator::ObjectPoseEstimatorConfiguration> sut_; //NOLINT
};

TEST_F(ObjectPoseEstimatorConfigurationShould, throwExceptionWhenIncorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badSyntax.json"));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badKey.json"));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badFileName.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould, notThrowExceptionWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould, throwExceptionWhenpassThroughMinLimitNotProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badpassThroughMinLimit.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould, throwExceptionWhenBadClusterToleranceProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badClusterToleranceProvided.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould, throwExceptionWhenBadMinClusterSizeProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badMinClusterSizeProvided.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould, throwExceptionWhenBadMaxClusterSizeProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badMaxClusterSizeProvided.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould,
        throwExceptionWhenBadCovarianceSamplingTotalPointsDivisionProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(
        testDirName_ + "configFile_badCovarianceSamplingTotalPointsDivision.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould,
        throwExceptionWhenBadNormalSamplingTotalPointsDivisionProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(
        testDirName_ + "configFile_badNormalSamplingTotalPointsDivision.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould,
    throwExceptionWhenBadNormalSamplingBinSizeProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badNormalSamplingBinSize.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould,
    throwExceptionWhenBadSubSamplingSideLengthProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badSubsamplingSideLength.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould,
        throwExceptionWhenBadShotColorMultiscaleFeaturePersistenceScaleValuesProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(
        testDirName_ + "configFile_badShotColorMultiscaleFeaturePersistenceScaleValues.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould,
    throwExceptionWhenBadSHOTColorPersintenceAlphaProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badSHOTColorPersintenceAlpha.json"));
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    throwExceptionWhenBadSHOTComputationThreadsProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badSHOTComputationThreads.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould, throwExceptionWhenBadFPFHPersintenceAlphaProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badFPFHPersintenceAlpha.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould,
    throwExceptionWhenfBadConvergenceCriteriaParametersProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badConvergenceCriteriaParameters.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould,
    throwExceptionWhenBadNearestNeighborDistanceProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badNearestNeighborDistance.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould,
    throwExceptionWhenBadCorrespondencesMaximumDistanceProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badCorrespondencesMaximumDistance.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould,
        throwExceptionWhenBadCorrespondencesMaximumMedianDistanceFactorProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(
        testDirName_ + "configFile_badCorrespondencesMaximumMedianDistanceFactor.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould,
    throwExceptionWhenBadSampleConsensusInlierThresholdProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badSampleConsensusInlierThreshold.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould,
    throwExceptionWhenBadSampleConsensusMaximumIterationsProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_FALSE(sut_->parse(
        testDirName_ + "configFile_badSampleConsensusMaximumIterations.json"));
}

TEST_F(ObjectPoseEstimatorConfigurationShould, correctlySetAndUnsetICPRefinement) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));

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

TEST_F(ObjectPoseEstimatorConfigurationShould, getModelPathWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    // Get current directory
    std::string directory = __FILE__;
    directory = directory.substr(0, directory.find("cpproboticframework"));
    directory += "cpproboticframework/tests/Applications/ObjectPoseEstimatorTests/data/BlmModelHalf2.pcd"; //NOLINT
    ASSERT_EQ(sut_->getModelPath(), directory);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getCluserVisualizationValueWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));

    ASSERT_EQ(sut_->getCluserVisualizationValue(), false);
}
TEST_F(ObjectPoseEstimatorConfigurationShould, getClusterToleranceWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getClusterTolerance(), 0.05);
}
TEST_F(ObjectPoseEstimatorConfigurationShould, getMinClusterSizeWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getMinClusterSize(), 50);
}
TEST_F(ObjectPoseEstimatorConfigurationShould, getMaxClusterSizeWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getMaxClusterSize(), 250000);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getMomentOfInertiaVisualizationValueWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getMomentOfInertiaVisualizationValue(), false);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getNearestNeighborDistanceWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getNearestNeighborDistance(), 0.001);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getIcpStepsVisualizationValueWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getIcpStepsVisualizationValue(), false);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getIcpSourceNormalsVisualizationValueWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getIcpSourceNormalsVisualizationValue(), false);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getWeightedTransformEstimationValueWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getWeightedTransformEstimationValue(), true);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getCovarianceSubsamplingValueWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getCovarianceSubsamplingValue(), false);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getCovarianceSamplingTotalPointsDivisionWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getCovarianceSamplingTotalPointsDivision(), 2);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getNormalSamplingTotalPointsDivisionWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getNormalSamplingTotalPointsDivision(), 2);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getNormalSamplingBinSizeWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getNormalSamplingBinSize(), 50);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getCorrespondencesMaximumDistanceWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getCorrespondencesMaximumDistance(), 0.05);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getCorrespondencesMaximumMedianDistanceFactorWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getCorrespondencesMaximumMedianDistanceFactor(), 1.5);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getCorrespondencesMaximunAngleInRadWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getCorrespondencesMaximunAngleInRad(), 0.2);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getTranslationThresholdWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getTranslationThreshold(), 9e-08);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getRotationThresholdWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getRotationThreshold(), 0.0044);
}
TEST_F(ObjectPoseEstimatorConfigurationShould, getMaximumIterationsWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getMaximumIterations(), 100);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getPassThroughMinLimitWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getPassThroughMinLimit(), 0.45);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getIcpRefinementValueWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getIcpRefinementValue(), false);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getIcpInitialAligmentVisualizationValueWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getIcpInitialAligmentVisualizationValue(), false);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getSampleConsensusInlierThresholdWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getSampleConsensusInlierThreshold(), 0.02);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getSampleConsensusMaximumIterationsWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getSampleConsensusMaximumIterations(), 1000000);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getSampleConsensusRefineModelValueWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getSampleConsensusRefineModelValue(), false);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getSubsamplingSideLengthWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getSubsamplingSideLength(), 0.01);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getFPFHPersintenceAlphaWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getFPFHPersintenceAlpha(), 0.05);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getMultiscaleFeaturePersistenceScaleValue1WhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getMultiscaleFeaturePersistenceScaleValue1(), 0.09);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getMultiscaleFeaturePersistenceScaleValue2WhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getMultiscaleFeaturePersistenceScaleValue2(), 0.11);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getMultiscaleFeaturePersistenceScaleValue3WhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getMultiscaleFeaturePersistenceScaleValue3(), 0.15);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getSHOTColorFeaturesValueWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getSHOTColorFeaturesValue(), true);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getSHOTComputationThreadsWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_EQ(sut_->getSHOTComputationThreads(), 4);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getSHOTColorPersintenceAlphaWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getSHOTColorPersintenceAlpha(), 0.05);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getSHOTColormultiscaleFeaturePersistenceScaleValue1WhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getSHOTColormultiscaleFeaturePersistenceScaleValue1(), 0.09);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getSHOTColormultiscaleFeaturePersistenceScaleValue2WhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getSHOTColormultiscaleFeaturePersistenceScaleValue2(), 0.11);
}
TEST_F(ObjectPoseEstimatorConfigurationShould,
    getSHOTColormultiscaleFeaturePersistenceScaleValue3WhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new ObjectPoseEstimatorConfiguration()));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    ASSERT_FLOAT_EQ(sut_->getSHOTColormultiscaleFeaturePersistenceScaleValue3(), 0.15);
}
