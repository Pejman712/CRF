/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Leanne Attard CERN EN-SMM-MRO
 * 
 *  ==================================================================================================
 */

#include "ThermalCamera/FLIRDeviceParams.hpp"

namespace crf {
namespace sensors {
namespace thermalcamera {

//--------------------------------------------------------------------------------
// DeviceParams
DeviceParams::DeviceParams(PvDevice * device): device_{device}, logger_("FLIRDeviceParams") {
    if (device_) {
        lDeviceParams = device->GetParameters();
    }
}

DeviceParams::~DeviceParams() {}

bool DeviceParams::get_double_param(string paramName, double* val) {
    logger_->debug("Going to get float double param");
    logger_->debug(paramName);
    PvResult result = lDeviceParams->GetFloatValue(paramName.c_str(), *val);

    if (!result.IsOK()) {
        logger_->warn("param not found");
        return false;
    }
    return true;
}

bool DeviceParams::get_integer_param(string paramName, int64_t* val) {
    logger_->debug("Going to get float integer param");
    PvResult result = lDeviceParams->GetIntegerValue(paramName.c_str(), *val);

    if (!result.IsOK()) {
        logger_->warn("param not found");
        return false;
    }
    // return false; //string err = paramName + ":" + result.GetCodeString().GetAscii();
    return true;
}

bool DeviceParams::load_params() {
    if (!device_) {
        return false;
    }
    if (!get_double_param("R", &R))
        return false;
    if (!get_double_param("F", &F))
        return false;
    if (!get_double_param("B", &B))
        return false;
    if (!get_double_param("alpha1", &alpha1))
        return false;
    if (!get_double_param("alpha2", &alpha2))
        return false;
    if (!get_double_param("beta1", &beta1))
        return false;
    if (!get_double_param("beta2", &beta2))
        return false;
    if (!get_double_param("X", &X))
        return false;
    if (!get_double_param("AtmosphericTemperature", &atmTao))
        return false;
    if (!get_double_param("ObjectEmissivity", &emissivity))
       return false;
    if (!get_double_param("ExtOpticsTransmission", &extOptTransm))
        return false;
    if (!get_double_param("J1", &J1))
        return false;
    if (!get_integer_param("J0", &J0))
        return false;
    if (!get_double_param("ExtOpticsTemperature", &extOptTemp))
        return false;
    return true;
}

PvGenCommand * DeviceParams::getCommand(string commandName) {
    PvGenCommand *command = dynamic_cast<PvGenCommand *>(lDeviceParams->GetCommand(
        commandName.c_str()));
    return command;
}

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
