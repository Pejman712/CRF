/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Leanne Attard CERN EN-SMM-MRO
 * 
 *  ==================================================================================================
 */

#include "ThermalCamera/FLIRCam.hpp"
#include "ThermalCamera/FLIRFactory.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace thermalcamera {

FLIRFactory::FLIRFactory():logger_("FLIRFactory") {}

FLIRFactory::~FLIRFactory() {
    logger_->debug("DTor Factory");
}

DeviceParams* FLIRFactory::createDeviceParams(PvDevice* device) {
    DeviceParams *params = new DeviceParams(device);
    if (!params)
        return nullptr;

    if (!params->load_params()) {
        return nullptr;
    }
    return params;
}

TempConverter* FLIRFactory::createTempConverter(DeviceParams* params) {
    if (!params)
        return nullptr;
    TempConverter *converter = new TempConverter(params);
    return converter;
}

PvString* FLIRFactory::selectThermalDevice(int deviceIndex) {
    PvString *aConnectionID = new PvString();
    PvResult lResult;
    const PvDeviceInfo *lSelectedDI = NULL;
    PvSystem lSystem;

    lSystem.Find();

    // Detect, select device.
    vector<const PvDeviceInfo *> lDIVector;
    for (uint32_t i = 0; i < lSystem.GetInterfaceCount(); i++) {
        const PvInterface *lInterface = dynamic_cast<const PvInterface *>(lSystem.GetInterface(i));
        if (lInterface != NULL) {
            for (uint32_t j = 0; j < lInterface->GetDeviceCount(); j++) {
                const PvDeviceInfo *lDI = dynamic_cast<const PvDeviceInfo *>(
                    lInterface->GetDeviceInfo(j));
                if (lDI != NULL) {
                    lDIVector.push_back(lDI);
                    cout << "[" << (lDIVector.size() - 1) << "]->" << "\t"
                        << lDI->GetDisplayID().GetAscii() << endl;
                }
            }
        }
    }

    if (lDIVector.size() == 0) {
        cout << "No device found!" << endl;
        return aConnectionID;
    } else if (lDIVector.size() < (deviceIndex +1)) {
        cout << "Device #" << std::to_string(deviceIndex) << " not found" << endl;
        return aConnectionID;
    } else {
        lSelectedDI = lDIVector[deviceIndex];  // TODO with index from config  // NOLINT
    }

    // If the IP Address valid?
    if (lSelectedDI->IsConfigurationValid()) {
        *aConnectionID = lSelectedDI->GetConnectionID();
        return aConnectionID;
    } else {
        cout << "The IP configuration of the device is not valid." << endl;
        return aConnectionID;
    }

    if ((lSelectedDI->GetType() == PvDeviceInfoTypeUSB) || (lSelectedDI->GetType() == PvDeviceInfoTypeU3V)) {  // NOLINT
        cout << "This device must be connected to a USB 3.0 (SuperSpeed) port." << endl;
        return aConnectionID;
    }
    return aConnectionID;
}

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
