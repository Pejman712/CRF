/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include <exception>
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ObjectPoseEstimator/ObjectPoseEstimatorConfiguration.hpp"

namespace crf {
namespace applications {
namespace objectposeestimator {

ObjectPoseEstimatorConfiguration::ObjectPoseEstimatorConfiguration():
    logger_("ObjectPoseEstimatorConfiguration") {}

bool ObjectPoseEstimatorConfiguration::parse(const std::string& configFileName) {
    try {
        // Read configuration names from .json
        std::ifstream config(configFileName);
        nlohmann::json jConfig;
        config >> jConfig;

        // Get current directory
        std::string directory = __FILE__;
        directory = directory.substr(0, directory.find("cpproboticframework"));

        modelPath_ = directory + jConfig.at("preScanObjectPath").get<std::string>();
        std::ifstream modelPath(modelPath_);
        if (!modelPath) {
            throw std::invalid_argument("Couldn't read PCD file");
        }
        // Load Clustering, ICP and InitialAligment parameters
        clusterVisualization_ =
            jConfig.at("clusteringParameters").at("clusterVisualization").get<bool>();
        clusterTolerance_ = jConfig.at("clusteringParameters").at("clusterTolerance").get<float>();
        if (clusterTolerance_ <= 0) {
             throw std::invalid_argument("Cluster tolerance should be greater than 0");
        }
        minClusterSize_ = jConfig.at("clusteringParameters").at("minClusterSize").get<int>();
        if (minClusterSize_ < 1) {
             throw std::invalid_argument("MinClusterSize should be greater or equal to 1");
        }
        maxClusterSize_ = jConfig.at("clusteringParameters").at("maxClusterSize").get<int>();
        if (maxClusterSize_ < 1) {
             throw std::invalid_argument("MaxClusterSize should be greater or equal to 1");
        }
        momentOfInertiaVisualization_ =
            jConfig.at("momentOfInertiaParameters").at("momentOfInertiaVisualization").get<bool>();
        nearestNeighborDistance_ =
            jConfig.at("fitnessParameters").at("nearestNeighborDistance").get<float>();
        if (nearestNeighborDistance_ <= 0) {
            throw std::invalid_argument(
                "Impossible to estimate fitness, NearestNeighborDistance should be greater than 0");
        }
        icpStepsVisualization_ =
            jConfig.at("icpRefinementParameters").at("icpStepsVisualization").get<bool>();
        icpSourceNormalsVisualization_ =
            jConfig.at("icpRefinementParameters").at("icpSourceNormalsVisualization").get<bool>();
        weightedTransformEstimation_ =
            jConfig.at("icpRefinementParameters").at("weightedTransformEstimation").get<bool>();
        covarianceSubsampling_ =
            jConfig.at("icpRefinementParameters").at("covarianceSubsampling").get<bool>();
        covarianceSamplingTotalPointsDivision_ = jConfig.at("icpRefinementParameters").at(
            "covarianceSamplingTotalPointsDivision").get<int>();
        if (covarianceSamplingTotalPointsDivision_ <= 0) {
            throw std::invalid_argument(
                "covarianceSamplingTotalPointsDivision should be greater than 0");
        }
        normalSamplingTotalPointsDivision_ = jConfig.at("icpRefinementParameters").
            at("normalSamplingTotalPointsDivision").get<int>();
        if (normalSamplingTotalPointsDivision_ <= 0) {
            throw std::invalid_argument(
                "normalSamplingTotalPointsDivision should be greater than 0");
        }
        normalSamplingBinSize_ =
            jConfig.at("icpRefinementParameters").at("normalSamplingBinSize").get<int>();
        if (normalSamplingBinSize_ <= 0) {
            throw std::invalid_argument("normalSamplingBinSize should be greater than 0");
        }
        correspondencesMaximumDistance_ =
            jConfig.at("icpRefinementParameters").at("correspondencesMaximumDistance").get<float>();
        if (correspondencesMaximumDistance_ < 0) {
            throw std::invalid_argument("CorrespondencesMaximumDistance should be greater than 0");
        }
        correspondencesMaximumMedianDistanceFactor_ =
            jConfig.at("icpRefinementParameters").at(
            "correspondencesMaximumMedianDistanceFactor").get<float>();
        if (correspondencesMaximumMedianDistanceFactor_ < 0) {
            throw std::invalid_argument(
                "CorrespondencesMaximumMedianDistanceFactor should be greater than 0");
        }
        correspondencesMaximunAngleInRad_ = jConfig.at("icpRefinementParameters").
            at("correspondencesMaximunAngleInRad").get<float>();
        translationThreshold_ = jConfig.at("icpRefinementParameters").
            at("translationThreshold").get<double>();
        rotationThreshold_ = jConfig.at("icpRefinementParameters").
            at("rotationThreshold").get<double>();
        maximumIterations_ = jConfig.at("icpRefinementParameters").
            at("maximumIterations").get<int>();
        if (translationThreshold_ <=0 || rotationThreshold_ <=0 || maximumIterations_ <=0) {
            throw std::invalid_argument(
                "Translation, rotation and maximumIterations should be greater than 0");
        }
        passThroughMinLimit_ =
            jConfig.at("icpRefinementParameters").at("passThroughMinLimit").get<float>();
        if (passThroughMinLimit_ < 0) {
             throw std::invalid_argument(
                "PassThrough minimum limit should be greater or equal to 0");
        }
        icpRefinement_ = jConfig.at("icpRefinementParameters").
            at("icpRefinement").get<bool>();
        icpInitialAligmentVisualization_ = jConfig.at("initialAlignmentParameters").
            at("icpInitialAligmentVisualization").get<bool>();
        sampleConsensusInlierThreshold_ = jConfig.at("initialAlignmentParameters").
            at("sampleConsensusInlierThreshold").get<float>();
        if (sampleConsensusInlierThreshold_ <= 0) {
            throw std::invalid_argument("SampleConsensusInlierThreshold should be greater than 0");
        }
        sampleConsensusMaximumIterations_ = jConfig.at("initialAlignmentParameters").
            at("sampleConsensusMaximumIterations").get<int>();
        if (sampleConsensusMaximumIterations_ < 1) {
            throw std::invalid_argument(
                "SampleConsensusMaximumIterations_ should be equal or greater than 1");
        }
        sampleConsensusRefineModel_ =
            jConfig.at("initialAlignmentParameters").at("sampleConsensusRefineModel").get<bool>();

        subsamplingSideLength_ =
            jConfig.at("initialAlignmentParameters").at("subsamplingSideLength").get<float>();
        if (subsamplingSideLength_ <= 0) {
             throw std::invalid_argument("subsamplingSideLength should be greater than 0");
        }
        FPFHPersintenceAlpha_ =
            jConfig.at("initialAlignmentParameters").at("FPFHPersintenceAlpha").get<float>();
        if (FPFHPersintenceAlpha_ <= 0) {
            throw std::invalid_argument("FPFHPersintenceAlpha should be greater than 0");
        }
        multiscaleFeaturePersistenceScaleValue1_ =
            jConfig.at("initialAlignmentParameters").
            at("multiscaleFeaturePersistenceScaleValue1").get<float>();
        multiscaleFeaturePersistenceScaleValue2_ =
            jConfig.at("initialAlignmentParameters").
            at("multiscaleFeaturePersistenceScaleValue2").get<float>();
        multiscaleFeaturePersistenceScaleValue3_ =
            jConfig.at("initialAlignmentParameters").
            at("multiscaleFeaturePersistenceScaleValue3").get<float>();
        if (multiscaleFeaturePersistenceScaleValue1_ <= 0 ||
            multiscaleFeaturePersistenceScaleValue2_ <= 0||
            multiscaleFeaturePersistenceScaleValue3_ <= 0) {
            throw std::invalid_argument(
                "MultiscaleFeaturePersistenceScaleValues should be greater than 0");
        }
        SHOTColorFeatures_ =
            jConfig.at("initialAlignmentParameters").at("SHOTColorFeatures").get<bool>();
        SHOTComputationThreads_ =
            jConfig.at("initialAlignmentParameters").at("SHOTComputationThreads").get<int>();
        if (SHOTComputationThreads_ < 1) {
            throw std::invalid_argument("SHOTComputationThreads should be greater or equal to 1");
        }
        SHOTColorPersintenceAlpha_ =
            jConfig.at("initialAlignmentParameters").at("SHOTColorPersintenceAlpha").get<float>();
        if (SHOTColorPersintenceAlpha_ <= 0) {
            throw std::invalid_argument("SHOTColorPersintenceAlpha should be greater than 0");
        }
        SHOTColormultiscaleFeaturePersistenceScaleValue1_ =
            jConfig.at("initialAlignmentParameters").
            at("SHOTColormultiscaleFeaturePersistenceScaleValue1").get<float>();
        SHOTColormultiscaleFeaturePersistenceScaleValue2_ =
            jConfig.at("initialAlignmentParameters").
            at("SHOTColormultiscaleFeaturePersistenceScaleValue2").get<float>();
        SHOTColormultiscaleFeaturePersistenceScaleValue3_ =
            jConfig.at("initialAlignmentParameters").
            at("SHOTColormultiscaleFeaturePersistenceScaleValue3").get<float>();
        if (SHOTColormultiscaleFeaturePersistenceScaleValue1_ <= 0 ||
            SHOTColormultiscaleFeaturePersistenceScaleValue2_ <= 0 ||
            SHOTColormultiscaleFeaturePersistenceScaleValue3_ <= 0) {
            throw std::invalid_argument(
                "SHOTColorMultiscaleFeaturePersistenceScaleValues should be greater than 0");
        }
    } catch (const std::exception& e) {
        logger_->warn("Failed to parse because: {}", e.what());
        return false;
    }


    return true;
}

std::string ObjectPoseEstimatorConfiguration::getModelPath() {
    return modelPath_;
}
bool ObjectPoseEstimatorConfiguration::getCluserVisualizationValue() {
    return clusterVisualization_;
}
float ObjectPoseEstimatorConfiguration::getClusterTolerance() {
    return clusterTolerance_;
}
int ObjectPoseEstimatorConfiguration::getMinClusterSize() {
    return minClusterSize_;
}
int ObjectPoseEstimatorConfiguration::getMaxClusterSize() {
    return maxClusterSize_;
}
bool ObjectPoseEstimatorConfiguration::getMomentOfInertiaVisualizationValue() {
    return momentOfInertiaVisualization_;
}
float ObjectPoseEstimatorConfiguration::getNearestNeighborDistance() {
    return nearestNeighborDistance_;
}
bool ObjectPoseEstimatorConfiguration::getIcpStepsVisualizationValue() {
    return icpStepsVisualization_;
}
bool ObjectPoseEstimatorConfiguration::getIcpSourceNormalsVisualizationValue() {
    return icpSourceNormalsVisualization_;
}
bool ObjectPoseEstimatorConfiguration::getWeightedTransformEstimationValue() {
    return weightedTransformEstimation_;
}
bool ObjectPoseEstimatorConfiguration::getCovarianceSubsamplingValue() {
    return covarianceSubsampling_;
}
int ObjectPoseEstimatorConfiguration::getCovarianceSamplingTotalPointsDivision() {
    return covarianceSamplingTotalPointsDivision_;
}
int ObjectPoseEstimatorConfiguration::getNormalSamplingTotalPointsDivision() {
    return normalSamplingTotalPointsDivision_;
}
int ObjectPoseEstimatorConfiguration::getNormalSamplingBinSize() {
    return normalSamplingBinSize_;
}
float ObjectPoseEstimatorConfiguration::getCorrespondencesMaximumDistance() {
    return correspondencesMaximumDistance_;
}
float ObjectPoseEstimatorConfiguration::getCorrespondencesMaximumMedianDistanceFactor() {
    return correspondencesMaximumMedianDistanceFactor_;
}
float ObjectPoseEstimatorConfiguration::getCorrespondencesMaximunAngleInRad() {
    return correspondencesMaximunAngleInRad_;
}
double ObjectPoseEstimatorConfiguration::getTranslationThreshold() {
    return translationThreshold_;
}
double ObjectPoseEstimatorConfiguration::getRotationThreshold() {
    return rotationThreshold_;
}
int ObjectPoseEstimatorConfiguration::getMaximumIterations() {
    return maximumIterations_;
}
float ObjectPoseEstimatorConfiguration::getPassThroughMinLimit() {
    return passThroughMinLimit_;
}
bool ObjectPoseEstimatorConfiguration::getIcpRefinementValue() {
    return icpRefinement_;
}
bool ObjectPoseEstimatorConfiguration::getIcpInitialAligmentVisualizationValue() {
    return icpInitialAligmentVisualization_;
}
float ObjectPoseEstimatorConfiguration::getSampleConsensusInlierThreshold() {
    return sampleConsensusInlierThreshold_;
}
int ObjectPoseEstimatorConfiguration::getSampleConsensusMaximumIterations() {
    return sampleConsensusMaximumIterations_;
}
bool ObjectPoseEstimatorConfiguration::getSampleConsensusRefineModelValue() {
    return sampleConsensusRefineModel_;
}
float ObjectPoseEstimatorConfiguration::getSubsamplingSideLength() {
    return subsamplingSideLength_;
}
float ObjectPoseEstimatorConfiguration::getFPFHPersintenceAlpha() {
    return FPFHPersintenceAlpha_;
}
float ObjectPoseEstimatorConfiguration::getMultiscaleFeaturePersistenceScaleValue1() {
    return multiscaleFeaturePersistenceScaleValue1_;
}
float ObjectPoseEstimatorConfiguration::getMultiscaleFeaturePersistenceScaleValue2() {
    return multiscaleFeaturePersistenceScaleValue2_;
}
float ObjectPoseEstimatorConfiguration::getMultiscaleFeaturePersistenceScaleValue3() {
    return multiscaleFeaturePersistenceScaleValue3_;
}
bool ObjectPoseEstimatorConfiguration::getSHOTColorFeaturesValue() {
    return SHOTColorFeatures_;
}
int ObjectPoseEstimatorConfiguration::getSHOTComputationThreads() {
    return SHOTComputationThreads_;
}
float ObjectPoseEstimatorConfiguration::getSHOTColorPersintenceAlpha() {
    return SHOTColorPersintenceAlpha_;
}
float ObjectPoseEstimatorConfiguration::getSHOTColormultiscaleFeaturePersistenceScaleValue1() {
    return SHOTColormultiscaleFeaturePersistenceScaleValue1_;
}
float ObjectPoseEstimatorConfiguration::getSHOTColormultiscaleFeaturePersistenceScaleValue2() {
    return SHOTColormultiscaleFeaturePersistenceScaleValue2_;
}
float ObjectPoseEstimatorConfiguration::getSHOTColormultiscaleFeaturePersistenceScaleValue3() {
    return SHOTColormultiscaleFeaturePersistenceScaleValue3_;
}

bool ObjectPoseEstimatorConfiguration::setICPRefinement(bool icpFlag) {
    if (icpFlag) {
        if (icpRefinement_) {
            logger_->warn("ICP Refinement already set");
            return false;
        } else {
            icpRefinement_ = true;
            return true;
        }
    } else {
        if (!icpRefinement_) {
            logger_->warn("ICP Refinement already unset");
            return false;
        } else {
            icpRefinement_ = false;
            return true;
        }
    }
}

}  // namespace objectposeestimator
}  // namespace applications
}  // namespace crf

