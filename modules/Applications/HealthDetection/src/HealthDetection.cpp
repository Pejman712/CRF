/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>
#include <numeric>

#include <boost/filesystem.hpp>
#include <eigen3/unsupported/Eigen/FFT>

#include "FourierTransform/FFT.hpp"
#include "HealthDetection/HealthDetection.hpp"

#define MAX_VARIANCE 3.0f  // maximum allowed variance for target range bin changes
#define OUTLIER_REMOVE_THRESHOLD 1.5f  // threshold, that removes occasional false detection points
#define KALMAN_INITIALIZATION_SAMPLE_AMOUNT 10

namespace crf {
namespace applications {
namespace healthdetection {

HealthDetection::HealthDetection(std::shared_ptr<sensors::fraunhoferradar::IRadar> radar,
    std::shared_ptr<algorithms::peakdetection::IPeakDetection> cfar,
    std::shared_ptr<algorithms::peakdetection::IPeakDetection> heartPeakDetector,
    std::shared_ptr<algorithms::peakdetection::IPeakDetection> respirationPeakDetector,
    std::shared_ptr<algorithms::stateestimator::IStateEstimator> stateEstimator,
    std::string recordDir):
    radar_(radar),
    cfar_(cfar),
    heartPeakDetector_(heartPeakDetector),
    respirationPeakDetector_(respirationPeakDetector),
    stateEstimator_(stateEstimator),
    logger_("HealthDetection"),
    initialized_(false),
    recordDir_(recordDir),
    frameNumber_(0),
    maxFrequency_(0) {
    logger_->debug("CTor");
}

HealthDetection::~HealthDetection() {
    logger_->debug("DTor");
    deinitialize();
}

bool HealthDetection::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("HealthDetection already initialized");
        return false;
    }
    if (!radar_->initialize()) {
        return false;
    }
    maxFrequency_ = radar_->getMaxObservationFrequency();
    initialized_ = true;
    return true;
}

bool HealthDetection::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->debug("HealthDetection already deinitialized");
        return false;
    }
    if (!radar_->deinitialize()) {
        return false;
    }
    initialized_ = false;
    return true;
}

boost::optional<VitalSignalPacket> HealthDetection::getVitalSigns() {
    logger_->debug("getVitalSigns");
    radarPacket_ = {};
    if (!initialized_) {
        logger_->warn("Application was not initialized");
        return boost::none;
    }
    radarPacket_.radarData = radar_->getFrame();
    if (recordDir_ != "none") {
        saveData(radarPacket_.radarData);
    }
    crf::algorithms::fouriertransform::FFT fft;
    for (int i = 0; i < radarPacket_.radarData.size(); i++) {
        radarPacket_.rangeFftMatrix.push_back(fft.getFFT(radarPacket_.radarData[i]));
    }
    getPeaks();
    if (radarPacket_.peakMatrix.empty()) {
        logger_->debug("No peaks were detected");
        return boost::none;
    }
    // Init Kalman filter
    for (int i = 0; i < KALMAN_INITIALIZATION_SAMPLE_AMOUNT; i++) {
        std::vector<float> tempVec;
        tempVec.push_back(radarPacket_.peakMatrix[i]);
        stateEstimator_->addMeasurement(tempVec);
    }
    // Use filter to replace peakValues
    for (int i = 0; i < radarPacket_.peakMatrix.size(); i++) {
        std::vector<float> tempVec;
        tempVec.push_back(radarPacket_.peakMatrix[i]);
        stateEstimator_->addMeasurement(tempVec);
        auto estimate = stateEstimator_->getEstimate();
        radarPacket_.peakMatrix[i] = estimate[0];
    }
    removeOutliers();
    if (!radarPacket_.variance) {
        logger_->debug("variance too high, large movement detected");
        return boost::none;
    }
    getTargetBin();
    unwrapPhase();
    return extractVitalSignalFrequencies();
}

RadarPacket HealthDetection::getPacket() {
    return radarPacket_;
}

void HealthDetection::getPeaks() {
    logger_->debug("getPeaks");
    for (int i = 0; i < radarPacket_.rangeFftMatrix.size(); i++) {
        std::vector<float> absFrequencyVector;
        for (int j = 0; j < radarPacket_.rangeFftMatrix[i].size(); j++) {
          absFrequencyVector.push_back(
            std::abs(radarPacket_.rangeFftMatrix[i][j]));
        }
        std::vector<int> peakVec = cfar_->findPeaks(absFrequencyVector);
        if (!peakVec.empty()) {
            radarPacket_.peakMatrix.push_back(peakVec[0]);
        }
    }
    return;
}

void HealthDetection::getVariance() {
    logger_->debug("getVariance");
    float sum = std::accumulate(
        radarPacket_.peakMatrix.begin(), radarPacket_.peakMatrix.end(), 0.0);
    radarPacket_.mean = sum / radarPacket_.peakMatrix.size();
    float sq_sum = std::inner_product(
    radarPacket_.peakMatrix.begin(),
        radarPacket_.peakMatrix.end(),
        radarPacket_.peakMatrix.begin(),
        0.0);
    float calculatedVariance = std::sqrt(
        sq_sum / radarPacket_.peakMatrix.size() - pow(radarPacket_.mean, 2));
    if (calculatedVariance > MAX_VARIANCE) {
        radarPacket_.variance = boost::none;
    } else {
      radarPacket_.variance = calculatedVariance;
    }
    return;
}

void HealthDetection::removeOutliers() {
    logger_->debug("removeOutliers");
    getVariance();
    if (!radarPacket_.variance) {
        return;
    }
    radarPacket_.peakMatrix.erase(
        std::remove_if(radarPacket_.peakMatrix.begin(),
        radarPacket_.peakMatrix.end(), [this](int peak) {
            return ((peak - radarPacket_.mean) / radarPacket_.variance.get()) >
                OUTLIER_REMOVE_THRESHOLD; }), radarPacket_.peakMatrix.end());
    getVariance();
    return;
}

void HealthDetection::getTargetBin() {
    logger_->debug("getTargetBin");
    int max_count = 0;
    for (int i=0; i < radarPacket_.peakMatrix.size(); i++) {
        int count = 1;
        for (int j = i+1; j < radarPacket_.peakMatrix.size(); j++) {
            if (radarPacket_.peakMatrix[i] == radarPacket_.peakMatrix[j]) {
                count++;
            }
        }
        if (count > max_count) {
            max_count = count;
        }
    }
    for (int i=0; i < radarPacket_.peakMatrix.size(); i++) {
        int count = 1;
        for (int j=i+1; j < radarPacket_.peakMatrix.size(); j++) {
            if (radarPacket_.peakMatrix[i] == radarPacket_.peakMatrix[j]) {
                count++;
            }
        }
        if (count == max_count) {
            radarPacket_.targetBin  = radarPacket_.peakMatrix[i];
            return;
        }
    }
    radarPacket_.targetBin = boost::none;
    return;
}

void HealthDetection::unwrapPhase() {
    logger_->debug("unwrapPhase");
    if (!radarPacket_.targetBin) {
        radarPacket_.unwrappedPhaseVector = boost::none;
        return;
    }
    std::vector<float> tempUnwrappedPhaseVector;
    int k = 0;
    for (size_t i = 0; i < radarPacket_.rangeFftMatrix.size() -1; i++) {
        tempUnwrappedPhaseVector.push_back(
            std::arg(radarPacket_.rangeFftMatrix[i][radarPacket_.targetBin.get()]) + 2*M_PI*k);
        if (std::abs(std::arg(radarPacket_.rangeFftMatrix[i+1][radarPacket_.targetBin.get()]) -
            std::arg(radarPacket_.rangeFftMatrix[i][radarPacket_.targetBin.get()])) > M_PI) {
            if (std::arg(radarPacket_.rangeFftMatrix[i+1][radarPacket_.targetBin.get()]) <
                std::arg(radarPacket_.rangeFftMatrix[i][radarPacket_.targetBin.get()])) {
                    k += 1;
            } else {
                k -= 1;
            }
        }
    }
    radarPacket_.unwrappedPhaseVector = tempUnwrappedPhaseVector;
    return;
}

boost::optional<VitalSignalPacket> HealthDetection::extractVitalSignalFrequencies() {
    logger_->debug("extractVitalSignalFrequencies");
    VitalSignalPacket vitalSignalPacket;
    std::vector<std::complex<float>> respFrequencyVector, heartFrequencyVector;
    if (!radarPacket_.unwrappedPhaseVector) {
        return boost::none;
    }
    Eigen::FFT<float> fft;
    fft.fwd(respFrequencyVector, radarPacket_.unwrappedPhaseVector.get());
    radarPacket_.signalFrequencyDistrVector = respFrequencyVector;
    heartFrequencyVector = respFrequencyVector;
    for (int i = 0; i < radarPacket_.signalFrequencyDistrVector.size(); i++) {
        float frequency = i*(maxFrequency_/(radarPacket_.signalFrequencyDistrVector.size() / 2.0));
        if (frequency >= 0.1 && frequency <= 0.6) {  // expected lung frequency range
            respFrequencyVector[i] = radarPacket_.signalFrequencyDistrVector[i];
        } else {
            respFrequencyVector[i] = 0;
        }
        if (frequency >= 0.8 && frequency <= 2.0) {  // expected heartbeat frequency range
            heartFrequencyVector[i] = radarPacket_.signalFrequencyDistrVector[i];
        } else {
            heartFrequencyVector[i] = 0;
        }
    }
    fft.inv(radarPacket_.respTimeSignal, respFrequencyVector);
    fft.inv(radarPacket_.heartTimeSignal, heartFrequencyVector);
    radarPacket_.lungPeaks = respirationPeakDetector_->findPeaks(radarPacket_.respTimeSignal);
    radarPacket_.heartPeaks = heartPeakDetector_->findPeaks(radarPacket_.heartTimeSignal);
    if (radarPacket_.lungPeaks.size() == 0 && radarPacket_.heartPeaks.size() == 0) {
        logger_->warn("No Movement has been detected");
        return boost::none;
    }
    // getting rate from Hz to BPM
    vitalSignalPacket.respirationRate =
        static_cast<int>(radarPacket_.lungPeaks.size() / (1 / (2 * maxFrequency_) *
        radarPacket_.unwrappedPhaseVector.get().size()) * 60);
    vitalSignalPacket.heartRate =
        static_cast<int>(radarPacket_.heartPeaks.size() / (1 / (2 * maxFrequency_) *
        radarPacket_.unwrappedPhaseVector.get().size()) * 60);
    logger_->debug("respirations: {}, heartbeats: {}", radarPacket_.lungPeaks.size(),
        radarPacket_.heartPeaks.size());
    return vitalSignalPacket;
}

bool HealthDetection::saveData(std::vector<std::vector<float>> data) {
    logger_->debug("saveData");
    if (!boost::filesystem::exists(recordDir_)) {
        logger_->debug("Folder {} was created", recordDir_);
        boost::filesystem::create_directory(recordDir_);
    }
    std::ofstream out(recordDir_ + "frame" + std::to_string(frameNumber_) + ".dat");
    for (int i = 0; i < data.size(); ++i) {
        for (int j = 0; j < data[0].size(); ++j) {
            out << std::real(data[i][j]) << " ";
        }
        out << std::endl;
    }
    out.close();
    frameNumber_ += 1;
    return true;
}

}  // namespace healthdetection
}  // namespace applications
}  // namespace crf
