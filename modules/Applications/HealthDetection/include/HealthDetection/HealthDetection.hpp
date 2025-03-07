#pragma once

/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <vector>
#include <memory>
#include <string>

#include <boost/optional.hpp>

#include "Radars/IRadar.hpp"
#include "PeakDetection/IPeakDetection.hpp"
#include "StateEstimator/IStateEstimator.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace applications {
namespace healthdetection {

struct RadarPacket {
    std::vector<std::vector<float>> radarData;
    std::vector<std::vector<std::complex<float>>> rangeFftMatrix;
    std::vector<int> peakMatrix;
    boost::optional<float> variance;
    float mean;
    boost::optional<int> targetBin;
    std::vector<std::complex<float>> signalFrequencyDistrVector;
    boost::optional<std::vector<float>> unwrappedPhaseVector;
    std::vector<float> respTimeSignal;
    std::vector<float> heartTimeSignal;
    std::vector<int> heartPeaks;
    std::vector<int> lungPeaks;
};

struct VitalSignalPacket {
    int respirationRate;
    int heartRate;
};

/**
* HealthDetection class uses radar to detect person's heartbeat and respiration frequency across
* distance.
*/

class HealthDetection {
 public:
    HealthDetection(std::shared_ptr<sensors::fraunhoferradar::IRadar> radar,
        std::shared_ptr<algorithms::peakdetection::IPeakDetection> cfar,
        std::shared_ptr<algorithms::peakdetection::IPeakDetection> heartPeakDetector,
        std::shared_ptr<algorithms::peakdetection::IPeakDetection> respirationPeakDetector,
        std::shared_ptr<algorithms::stateestimator::IStateEstimator> stateEstimator,
        std::string recordDir = "none");
    ~HealthDetection();

    bool initialize();
    bool deinitialize();
    /**
    * Returns vital signal packet (heartbeat and respiration rate) if a person is detected in FOV.
    * Returns boost::none when the signal is not detected or unavailable.
    */
    boost::optional<VitalSignalPacket> getVitalSigns();
    /**
    * This method is introduced just for example/debugging purpose.
    * Returns raw data signal, vital signal, as well as intermediate values.
    * Always returns radarPacket. In case of failure returns empty packet.
    */
    RadarPacket getPacket();

 private:
    void getPeaks();
    void getVariance();
    void removeOutliers();
    void getTargetBin();
    void unwrapPhase();
    boost::optional<VitalSignalPacket> extractVitalSignalFrequencies();
    bool saveData(std::vector<std::vector<float>> data);

    std::shared_ptr<crf::sensors::fraunhoferradar::IRadar> radar_;
    std::shared_ptr<crf::algorithms::peakdetection::IPeakDetection> cfar_;
    std::shared_ptr<crf::algorithms::peakdetection::IPeakDetection> heartPeakDetector_;
    std::shared_ptr<crf::algorithms::peakdetection::IPeakDetection> respirationPeakDetector_;
    std::shared_ptr<crf::algorithms::stateestimator::IStateEstimator> stateEstimator_;
    RadarPacket radarPacket_;
    bool initialized_;
    int frameNumber_;
    std::string recordDir_;
    crf::utility::logger::EventLogger logger_;
    float maxFrequency_;
};

}  // namespace healthdetection
}  // namespace applications
}  // namespace crf
