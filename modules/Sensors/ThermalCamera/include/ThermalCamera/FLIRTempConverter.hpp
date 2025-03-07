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

#include <cmath>

#include "ThermalCamera/FLIRDeviceParams.hpp"

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace thermalcamera {

class TempConverter {
 public :
    TempConverter(DeviceParams* params);  // NOLINT
    ~TempConverter();

    #define ASY_SAFEGUARD 1.0002
    #define EXP_SAFEGUARD 709.78
    #define H2O_K1 +1.5587e+0
    #define H2O_K2 +6.9390e-2
    #define H2O_K3 -2.7816e-4
    #define H2O_K4 +6.8455e-7
    #define TAO_TATM_MIN -30.0
    #define TAO_TATM_MAX  90.0
    #define TAO_SQRTH2OMAX 6.2365
    #define TAO_COMP_MIN 0.400
    #define TAO_COMP_MAX 1.000

    // camera paramaters that are used inthe conversion of raw data to celsius
    double B, F, O, R, m_J1, m_AmbTemp, m_ExtOptTemp;
    double m_RelHum, m_AtmTemp, m_ObjectDistance, m_X, m_alpha1, m_beta1, m_alpha2, m_beta2, \
        m_AtmTao, m_Emissivity, m_ExtOptTransm;
    int64_t m_J0;

    // Kelvin constant
    static constexpr double kT0{273.15};  // < Kelvin at 0 celcius


    /**
      * RawToCelsius Convert 16-bit raw data to celsius
      * @param[in] s Raw pixel value
      * @return temperature in Celsius
      */
    double RawToCelsius(uint16_t s);

 private:
    /**
      * Calculates K1, parameter used in the conversion of Raw to Temp Celsius
      * @return K1
      */
    double doCalcK1();

    /**
      * Calculates K2, parameter used in the conversion of Raw to Temp Celsius
      * @param[in] dAmbObjSig Ambient Object Signal
      * @param[in] dAtmObjSig Atmospheric Object Signal
      * @param[in] dExtOptTempObjSig Extrinsics Optics Temperature object
      * @return K2
      */
    double doCalcK2(double dAmbObjSig, double dAtmObjSig, double dExtOptTempObjSig);

    /**
      * Calculates temperature from signal
      * @param[in] dObjSig Ambient Object Signal
      * @return temp
      */
    double objSigToTemp(double dObjSig);

    /**
      * Calculate atmospheric transmission
      * @return Tao, atmospheric transmission
      */
    double doCalcAtmTao();
};

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
