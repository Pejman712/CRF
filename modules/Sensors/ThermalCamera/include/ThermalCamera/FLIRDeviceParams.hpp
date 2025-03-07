/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Leanne Attard CERN EN-SMM-MRO
 * 
 *  ==================================================================================================
 */
#pragma once
#define _UNIX_
#include <PvSampleUtils.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvPipeline.h>
#include <PvBuffer.h>

#include <string>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace thermalcamera {

class DeviceParams {
 public:
    DeviceParams(PvDevice* device);  // NOLINT
    ~DeviceParams();
    bool load_params();
    PvGenCommand * getCommand(string commandName);
    double R, F, B, O, X, alpha1, alpha2, beta1, beta2, atmTao, emissivity, extOptTransm, J1, \
        extOptTemp;
    int64_t J0;
    // declared as public for the purpose of testing
    bool get_double_param(string paramName, double* val);
    bool get_integer_param(string paramName, int64_t* val);
 private:
    PvGenParameterArray * lDeviceParams;
    PvDevice* device_;
    utility::logger::EventLogger logger_;
};

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
