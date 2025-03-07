#pragma once

/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <complex>
#include <chrono>
#include <vector>
#include <memory>

#include <nlohmann/json.hpp>

#include "Radars/IRadar.hpp"
#include "SerialCommunication/SerialCommunication.hpp"

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace fraunhoferradar {

enum ERROR_CODES_RUB_RADAR {  // possible radar error codes
    RADAR_NO_ERROR = 0,
    RADAR_CONNECTION_ERROR = -1,
    RADAR_NOT_SUPPORTED = -2,
    RADAR_STATUS_ERROR = -3,
    RADAR_TIMEOUT = -4,
    RADAR_DATA_INCOMPLETE = -5,
    RADAR_PARAMETER_ERROR = -6,
    RADAR_MEMORY_ERROR = -7,
    RADAR_BUSY = -8,
    RADAR_PLL_NOT_LOCKED = -9,
    RADAR_ERROR_UNDEFINED = -10
};

class FraunhoferRadar : public IRadar {
 public:
    FraunhoferRadar() = delete;
    FraunhoferRadar(const FraunhoferRadar&) = delete;
    FraunhoferRadar(FraunhoferRadar&&) = delete;
    explicit FraunhoferRadar(const nlohmann::json &configRadar,
      std::shared_ptr<communication::serialcommunication::ISerialCommunication> serial);
    ~FraunhoferRadar() override;
    bool initialize() override;
    bool deinitialize() override;
    std::vector<std::vector<float>> getFrame() override;
    float getMaxObservationFrequency() override;

 private:
    int ADC_VREF_, ADC_ResBit_, IF_Gain_, Ramp_Count_, selfReflectionBins_, PLL_INIT_Ndiv_,
      PLL_RAMP_Stepcount_, PLL_RAMP_Step_Size_, ADC_Ndata_, Ndata_;
    float PLL_HelpVCO_Freq_, PLL_PFD_Freq_, ADC_fa_, PLL_StartFreq_, PLL_StopFreq_, PLL_RampLength_,
      PLL_RAMP_BW_;
    uint32_t Ramp_Wait_;
    bool initialized_;
    std::shared_ptr<communication::serialcommunication::ISerialCommunication> serial_;
    utility::logger::EventLogger logger_;

    void getRadarConfigParams(const nlohmann::json &configRadar);
    int radarConfiguration();
    int calculateRadarParameters();
    int startRamp();
    int readStatus();
};

}  // namespace fraunhoferradar
}  // namespace sensors
}  // namespace crf
