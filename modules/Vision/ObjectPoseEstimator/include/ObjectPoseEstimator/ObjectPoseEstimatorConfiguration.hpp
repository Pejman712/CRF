#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include <chrono>
#include <string>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace applications {
namespace objectposeestimator {

class ObjectPoseEstimatorConfiguration {
 public:
    ObjectPoseEstimatorConfiguration();
    virtual ~ObjectPoseEstimatorConfiguration() = default;
    virtual bool parse(const std::string& configFileName);
    std::string getModelPath();
    bool getCluserVisualizationValue();
    float getClusterTolerance();
    int getMinClusterSize();
    int getMaxClusterSize();
    bool getMomentOfInertiaVisualizationValue();
    float getNearestNeighborDistance();
    bool getIcpStepsVisualizationValue();
    bool getIcpSourceNormalsVisualizationValue();
    bool getWeightedTransformEstimationValue();
    bool getCovarianceSubsamplingValue();
    int getCovarianceSamplingTotalPointsDivision();
    int getNormalSamplingTotalPointsDivision();
    int getNormalSamplingBinSize();
    float getCorrespondencesMaximumDistance();
    float getCorrespondencesMaximumMedianDistanceFactor();
    float getCorrespondencesMaximunAngleInRad();
    double getTranslationThreshold();
    double getRotationThreshold();
    int getMaximumIterations();
    float getPassThroughMinLimit();
    bool getIcpRefinementValue();
    bool setICPRefinement(bool icpFlag);
    bool getIcpInitialAligmentVisualizationValue();
    float getSampleConsensusInlierThreshold();
    int getSampleConsensusMaximumIterations();
    bool getSampleConsensusRefineModelValue();
    float getSubsamplingSideLength();
    float getFPFHPersintenceAlpha();
    float getMultiscaleFeaturePersistenceScaleValue1();
    float getMultiscaleFeaturePersistenceScaleValue2();
    float getMultiscaleFeaturePersistenceScaleValue3();
    bool getSHOTColorFeaturesValue();
    int getSHOTComputationThreads();
    float getSHOTColorPersintenceAlpha();
    float getSHOTColormultiscaleFeaturePersistenceScaleValue1();
    float getSHOTColormultiscaleFeaturePersistenceScaleValue2();
    float getSHOTColormultiscaleFeaturePersistenceScaleValue3();

 protected:
    utility::logger::EventLogger logger_;

    // Configuration names
    std::string modelPath_;
    bool clusterVisualization_;
    float clusterTolerance_;
    int minClusterSize_;
    int maxClusterSize_;
    bool momentOfInertiaVisualization_;
    float nearestNeighborDistance_;
    bool icpStepsVisualization_;
    bool icpSourceNormalsVisualization_;
    bool weightedTransformEstimation_;
    bool covarianceSubsampling_;
    int covarianceSamplingTotalPointsDivision_;
    int normalSamplingTotalPointsDivision_;
    int normalSamplingBinSize_;
    float correspondencesMaximumDistance_;
    float correspondencesMaximumMedianDistanceFactor_;
    float correspondencesMaximunAngleInRad_;
    double translationThreshold_;
    double rotationThreshold_;
    int maximumIterations_;
    float passThroughMinLimit_;
    bool icpRefinement_;
    bool icpInitialAligmentVisualization_;
    float sampleConsensusInlierThreshold_;
    int sampleConsensusMaximumIterations_;
    bool sampleConsensusRefineModel_;
    float subsamplingSideLength_;
    float FPFHPersintenceAlpha_;
    float multiscaleFeaturePersistenceScaleValue1_;
    float multiscaleFeaturePersistenceScaleValue2_;
    float multiscaleFeaturePersistenceScaleValue3_;
    bool SHOTColorFeatures_;
    int SHOTComputationThreads_;
    float SHOTColorPersintenceAlpha_;
    float SHOTColormultiscaleFeaturePersistenceScaleValue1_;
    float SHOTColormultiscaleFeaturePersistenceScaleValue2_;
    float SHOTColormultiscaleFeaturePersistenceScaleValue3_;
};

}  // namespace objectposeestimator
}  // namespace applications
}  // namespace crf
