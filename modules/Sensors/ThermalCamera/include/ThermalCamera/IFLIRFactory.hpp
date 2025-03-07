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
#include "ThermalCamera/FLIRDeviceParams.hpp"
#include "ThermalCamera/FLIRTempConverter.hpp"

namespace crf {
namespace sensors {
namespace thermalcamera {

/**
* Interface class for FLIR-specific classes factory
* Known subclasses: FLIRFactory, FLIRFactoryMock
*/
class IFLIRFactory {
 public:
    IFLIRFactory() = default;
    virtual ~IFLIRFactory() = default;
      /**
    * Loads the device parameters
    * @param[in] device FLIR device
    * @param[in,out] params pointer to be initialised with an instance of DeviceParams
    * @return success or not
    */
    virtual DeviceParams* createDeviceParams(PvDevice *device) = 0;

      /**
    * Creates a converter to change from raw data to celsius
    * @param[in] params pointer to device params
    * @param[in,out] params pointer to be initialised with an instance of TempConverter
    * @returnsuccess or not
    */
    virtual TempConverter* createTempConverter(DeviceParams* params) = 0;

    /**
    * Looks for possible devices to connect to and get the connction id for the device index passed
    * @param[in,out] aConnectionID pointer to the connection id of the selected device
    * @param[in] deviceIndex index of device to select
    * @return true if found else false
    */
    virtual PvString* selectThermalDevice(int index) = 0;
};

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
