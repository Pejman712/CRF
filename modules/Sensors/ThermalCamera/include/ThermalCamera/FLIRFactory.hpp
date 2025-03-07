/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Leanne Attard CERN EN-SMM-MRO
 * 
 *  ==================================================================================================
 */
#pragma once
#include <PvSampleUtils.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvPipeline.h>
#include <PvBuffer.h>

#include <cmath>

#include "EventLogger/EventLogger.hpp"
#include "ThermalCamera/IFLIRFactory.hpp"

#define _UNIX_

namespace crf {
namespace sensors {
namespace thermalcamera {

class FLIRFactory:public IFLIRFactory {
 public:
    FLIRFactory();
    ~FLIRFactory() override;

    DeviceParams* createDeviceParams(PvDevice *device);
    TempConverter*createTempConverter(DeviceParams* params);
    PvString* selectThermalDevice(int index);

 private:
    utility::logger::EventLogger logger_;
};

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
