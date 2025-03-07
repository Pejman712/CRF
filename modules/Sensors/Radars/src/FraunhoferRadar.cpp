/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <cmath>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <csignal>
#include <vector>
#include <complex>
#include <numeric>
#include <string>
#include <memory>
#include <chrono>

#include "Radars/FraunhoferRadar/FraunhoferRadar.hpp"

#define MAX_RADAR_MESSAGE_LENGTH 26

namespace crf {
namespace sensors {
namespace fraunhoferradar {

FraunhoferRadar::FraunhoferRadar(const nlohmann::json &configRadar,
  std::shared_ptr<communication::serialcommunication::ISerialCommunication> serial):
  initialized_(false),
  serial_(serial),
  logger_("FraunhoferRadar") {
    logger_->debug("CTor");
    getRadarConfigParams(configRadar);
}

FraunhoferRadar::~FraunhoferRadar() {
    logger_->debug("DTor");
    deinitialize();
}

bool FraunhoferRadar::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }
    if (!serial_->initialize()) {
        logger_->error("Could not initialize serial port");
        return false;
    }
    if (radarConfiguration() != RADAR_NO_ERROR) {
        logger_->error("Radar configuration failed");
        return false;
    }
    initialized_ = true;
    return true;
}

bool FraunhoferRadar::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Already deinitialized");
        return false;
    }
    if (!serial_->deinitialize()) {
        logger_->error("could not deinitialize serial port");
        return false;
    }
    initialized_ = false;
    return true;
}

std::vector<std::vector<float>> FraunhoferRadar::getFrame() {
    logger_->debug("getFrame");
    std::vector<std::vector<float>> radarData;
    if (!initialized_) {
        logger_->error("Radar was not initialized, returning empty 2dVector");
        return radarData;
    }
    auto startCounter = std::chrono::steady_clock::now();
    if (startRamp() != RADAR_NO_ERROR) {
        throw std::runtime_error("Could not start ramp");
        return radarData;
    }
    int expectedBytes = sizeof(int16_t) * ADC_Ndata_ * Ramp_Count_;
    std::string buff;
    int fullBufferReads = expectedBytes / 4095;
    for (int i = 0; i < fullBufferReads; i++) {
        std::string tempBuff;
        serial_->read(&tempBuff, 4095);
        buff += tempBuff;
    }
    if (expectedBytes - fullBufferReads * 4095 > 0) {
        std::string tempBuff;
        serial_->read(&tempBuff, expectedBytes - fullBufferReads * 4095);
        buff += tempBuff;
    }
    if (buff.length() != expectedBytes) {
        logger_->error("Received byte ({}) amount mismatch ({})", buff.length(), expectedBytes);
        throw std::runtime_error("Received byte amount mismatch fail");
        return radarData;
    }
    if (std::chrono::duration <double, std::milli> (
       std::chrono::steady_clock::now() - startCounter).count() >
         Ramp_Count_ * Ramp_Wait_ * 1e-3 * 1.05) {  // 5% threshold for computer related processes
       logger_->error("Read timeout fail");
       throw std::runtime_error("Read timeout fail");
       return radarData;
    }
    std::vector<float> rawData;
    int16_t data(0);
    for (size_t i = 0; i < buff.length() / 2; i++) {
        memcpy(&data, buff.data() +i*2, 2);
        rawData.push_back(data);
    }
    for (int i = 0; i < Ramp_Count_; i++) {
        std::vector<std::complex<float>> IfFrequencyVector, filteredIfFrequencyVector;
        auto first = rawData.cbegin() + i * ADC_Ndata_;
        auto last = rawData.cbegin() + (i+1) * ADC_Ndata_;
        std::vector<float> _singleRampData(first, last);
        radarData.push_back(_singleRampData);
    }
    return radarData;
}

float FraunhoferRadar::getMaxObservationFrequency() {
    logger_->debug("getMaxObservationFrequency");
    return 1.0/(2.0*Ramp_Wait_*1e-6);
}

void FraunhoferRadar::getRadarConfigParams(const nlohmann::json &configRadar) {
    logger_->debug("getRadarConfigParams");
    try {
        ADC_VREF_ = configRadar.at("ADC_VREF").get<int>();
        ADC_ResBit_ = configRadar.at("ADC_ResBit").get<int>();
        PLL_HelpVCO_Freq_ = configRadar.at("PLL_HelpVCO_Freq").get<float>();
        PLL_PFD_Freq_ = configRadar.at("PLL_PFD_Freq").get<float>();
        ADC_fa_ = configRadar.at("ADC_fa").get<float>();
        PLL_StartFreq_ = configRadar.at("PLL_StartFreq").get<float>();
        PLL_StopFreq_ = configRadar.at("PLL_StopFreq").get<float>();
        PLL_RampLength_ = configRadar.at("PLL_RampLength").get<float>();
        IF_Gain_ = configRadar.at("IF_Gain").get<int>();
        Ramp_Count_ = configRadar.at("Ramp_Count").get<int>();
        selfReflectionBins_ = configRadar.at("selfReflectionBins").get<int>();
    } catch (const nlohmann::json::exception& e) {
        logger_->error("Failed to read config because: {}", e.what());
        throw std::invalid_argument("Could not read config");
    }
    Ramp_Wait_ = (PLL_RampLength_ + configRadar.at("radarSampleTime").get<float>()) * 1e6;
}

int FraunhoferRadar::radarConfiguration() {
    logger_->debug("radarConfiguration");
    if (calculateRadarParameters() != RADAR_NO_ERROR) {
        return RADAR_PARAMETER_ERROR;
    }
    serial_->write("INIT\r");
    if (readStatus() != 100) {
        logger_->error("Error while reading status after INIT.");
        return RADAR_STATUS_ERROR;
    }
    serial_->write("NDIV=" + std::to_string(PLL_INIT_Ndiv_) + "\r");
    if (readStatus() != 100) {
        logger_->error("Error while reading status after setting NDIV to {}", PLL_INIT_Ndiv_);
        return RADAR_STATUS_ERROR;
    }
    serial_->write("STEPCOUNT=" + std::to_string(PLL_RAMP_Stepcount_) + "\r");
    if (readStatus() != 100) {
        logger_->error("Error while reading status after setting STEPCOUNT to {}",
        PLL_RAMP_Stepcount_);
        return RADAR_STATUS_ERROR;
    }
    serial_->write("STEPSIZE=" + std::to_string(PLL_RAMP_Step_Size_) + "\r");
    if (readStatus() != 100) {
        logger_->error("Error while reading status after setting STEPSIZE to {}",
        PLL_RAMP_Step_Size_);
        return RADAR_STATUS_ERROR;
    }
    serial_->write("SAMPLECOUNT=" + std::to_string(ADC_Ndata_) + "\r");
    if (readStatus() != 100) {
        logger_->error("Error while reading status after setting SAMPLECOUNT to {}", ADC_Ndata_);
        return RADAR_STATUS_ERROR;
    }
    serial_->write("IFGAIN=" + std::to_string(IF_Gain_) + "\r");
    if (readStatus() != 100) {
        logger_->error("Error while reading status after setting IFGAIN to {}", IF_Gain_);
        return RADAR_STATUS_ERROR;
    }
    serial_->write("RAMPCOUNT=" + std::to_string(Ramp_Count_) + "\r");
    if (readStatus() != 100) {
        logger_->error("Error while reading status after setting RAMPCOUNT to {}", Ramp_Count_);
        return RADAR_STATUS_ERROR;
    }
    if (Ramp_Wait_ < ADC_Ndata_ + 1000) {
        logger_->error(
          "Attention: RAMPWAIT was to small -> will be changed from {} to valid value {}",
          Ramp_Wait_, ADC_Ndata_ + 1000);
        Ramp_Wait_ = ADC_Ndata_ + 1000;
    }
    serial_->write("RAMPWAIT=" + std::to_string(Ramp_Wait_) + "\r");
    if (readStatus() != 100) {
        logger_->error("Error while reading status after setting RAMPWAIT to {}", Ramp_Wait_);
        return RADAR_STATUS_ERROR;
    }
    serial_->write("MAINPLLLOCK?\r");
    if (readStatus() != 111) {
        logger_->error("MAINPLL not locked");
        return RADAR_PLL_NOT_LOCKED;
    }
    serial_->write("AUXPLLLOCK?\r");
    if (readStatus() != 111) {
        logger_->error("AUXPLL not locked");
        return RADAR_PLL_NOT_LOCKED;
    }
    serial_->write("STOPRAMP\r");
    if (readStatus() != 100) {
        logger_->error("Error while reading status after STOPRAMP");
        return RADAR_STATUS_ERROR;
    }
    logger_->info("Radar initialization completed.");
    return RADAR_NO_ERROR;
}

int FraunhoferRadar::calculateRadarParameters() {
    logger_->debug("calculateRadarParameters");
    if (PLL_RampLength_ > 16e-3 || PLL_RampLength_ < 0.) {
        logger_->error("Ramp length {} outside length limits (0.. 16 ms)", PLL_RampLength_);
        return RADAR_PARAMETER_ERROR;
    }
    PLL_INIT_Ndiv_ = floor((-((PLL_StopFreq_/4)-PLL_HelpVCO_Freq_))/PLL_PFD_Freq_);
    PLL_RAMP_BW_ = (PLL_StopFreq_-PLL_StartFreq_);
    if (PLL_RAMP_BW_ > 25.5e9 || PLL_RAMP_BW_ < 0.) {
        logger_->error(
          "error: Ramp bandwidth {} outside bandwidth limits (0.. 25.5 GHz)", PLL_RAMP_BW_);
        return RADAR_PARAMETER_ERROR;
    }
    PLL_RAMP_Stepcount_ = round(PLL_RampLength_*PLL_PFD_Freq_);
    PLL_RAMP_Step_Size_ = round(((PLL_RAMP_BW_/4)/PLL_RAMP_Stepcount_*16777216)/PLL_PFD_Freq_);
    ADC_Ndata_ = PLL_RampLength_ * ADC_fa_;
    Ndata_ = ADC_Ndata_;
    return RADAR_NO_ERROR;
}

int FraunhoferRadar::startRamp() {
    logger_->debug("startRamp");
    serial_->write("STARTRAMP\r");
    int receivedStatus = readStatus();
    if (receivedStatus == 203) {
        logger_->error("Radar busy");
        return RADAR_BUSY;
    }
    if (receivedStatus != 102) {
        logger_->error("Error while reading status after sending STARTRAMP");
        return RADAR_STATUS_ERROR;
    }
    return RADAR_NO_ERROR;
}

int FraunhoferRadar::readStatus() {
    logger_->debug("readStatus");
    int i = 0;
    std::string receivedMessage, buff;
    while (i < (MAX_RADAR_MESSAGE_LENGTH - 1)) {
        if (serial_->read(&buff, 1) != 1) {
            logger_->warn("Could not read byte.");
            return -1;
        }
        if (buff != "\r") {
            i += 1;
            receivedMessage += buff;
            continue;
        }
        if (serial_->read(&buff, 1) != 1) {
            logger_->warn("Could not read byte.");
            return -1;
        }
        if (buff != "\n") {
            logger_->warn("Unexpected end of message received: {}", buff);
            return -1;
        }
        return std::stoi(receivedMessage.substr(0, 3));
    }
    logger_->warn("max length of message exceeded: {}", receivedMessage);
    return -1;
}

}  // namespace fraunhoferradar
}  // namespace sensors
}  // namespace crf
