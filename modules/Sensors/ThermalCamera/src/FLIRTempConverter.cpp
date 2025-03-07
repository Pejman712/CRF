/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Leanne Attard CERN EN-SMM-MRO
 * 
 *  ==================================================================================================
 */

#include "ThermalCamera/FLIRTempConverter.hpp"
#include "ThermalCamera/FLIRDeviceParams.hpp"

namespace crf {
namespace sensors {
namespace thermalcamera {

//--------------------------------------------------------------------------------
// TempConverter
/**
  * Constructor of converter used to change from Raw to Temp Celsius
  * @param[in] params device parameters
  */
TempConverter::TempConverter(DeviceParams* params) {
    if (params) {
        B = params->B;
        F = params->F;
        R = params->R;
        m_alpha1 = params->alpha1;
        m_alpha2 = params->alpha2;
        m_beta1 = params->beta1;
        m_beta2 = params->beta2;
        m_X = params->X;
        m_AtmTao = params->atmTao;
        m_AtmTemp = params->extOptTransm;
        m_Emissivity = params->emissivity;
        m_ExtOptTransm = params->extOptTransm;
        m_J1 = params->J1;
        m_J0 = params->J0;
        m_ExtOptTemp = params->extOptTemp;
        m_AmbTemp = 293.15;
    }
}

/**
  * Destructor of converter used to change from Raw to Temp Celsius
  * @param[in] params device parameters
  */
TempConverter::~TempConverter() {}

double TempConverter::doCalcK1() {
    double dblVal = 1.0;

    dblVal = m_AtmTao * m_Emissivity * m_ExtOptTransm;

    if (dblVal > 0.0)
        dblVal = 1/dblVal;

    return (dblVal);
}

double TempConverter::doCalcK2(double dAmbObjSig, double dAtmObjSig, double dExtOptTempObjSig) {
    double emi;
    double temp1 = 0.0;
    double temp2 = 0.0;
    double temp3 = 0.0;

    emi  = m_Emissivity;

    if (emi > 0.0) {
        temp1 = (1.0 - emi)/emi * dAmbObjSig;

        if (m_AtmTao > 0.0) {
            temp2 = (1.0 - m_AtmTao)/(emi*m_AtmTao)* dAtmObjSig;

            if (m_ExtOptTransm > 0.0 && m_ExtOptTransm < 1.0) {
                temp3 = (1.0 - m_ExtOptTransm)/(emi*m_AtmTao*m_ExtOptTransm)*dExtOptTempObjSig;
            }
        }
    }

    return (temp1 + temp2 + temp3);
}

double TempConverter::objSigToTemp(double dObjSig) {
    double dbl_reg, tmp;
    double TKelvin;

    // Tkelvin = B /log(R / objSign + F)

    if (dObjSig > 0.0) {
        dbl_reg = R / dObjSig + F;

        if (F <= 1.0) {
            if (dbl_reg < ASY_SAFEGUARD)
                dbl_reg = ASY_SAFEGUARD;  // Don't get above a R/(1-F)
                                          // (horizontal) asymptote
        } else {  // if (m_F > 1.0)
            tmp = F * ASY_SAFEGUARD;
            if (dbl_reg < tmp)
                dbl_reg = tmp;
                // Don't get too close to a B/ln(F) (vertical) asymptote
        }
        TKelvin = B / log(dbl_reg);
    }

    return (TKelvin);
}

double TempConverter::RawToCelsius(uint16_t s) {
    // imgToTemp

    // (1)imgtoPow
    double pow_ = (s - m_J0) / m_J1;

    // (2)powToObjSig
    m_AtmTao = doCalcAtmTao();
    double m_K1 = doCalcK1();
    double m_K2 = doCalcK2(m_AmbTemp, m_AtmTemp, m_ExtOptTemp);

    double x = (m_K1 * static_cast<double>(pow_ - m_K2));

    // (3)objSigToTemp
    double temp = objSigToTemp(x);

    return temp - kT0;
}

double TempConverter::doCalcAtmTao() {
    double tao, dtao;
    double sqrtH2O;
    double TT;
    double a1b1sqH2O, a2b2sqH2O, exp1, exp2;

    #define H2O_K1 +1.5587e+0
    #define H2O_K2 +6.9390e-2
    #define H2O_K3 -2.7816e-4
    #define H2O_K4 +6.8455e-7
    #define TAO_TATM_MIN -30.0
    #define TAO_TATM_MAX  90.0
    #define TAO_SQRTH2OMAX 6.2365
    #define TAO_COMP_MIN 0.400
    #define TAO_COMP_MAX 1.000

    double H, C, T, sqrtD, X, a1, b1, a2, b2;
    H = m_RelHum;
    C = m_AtmTemp;
    T = C - kT0;  // We need Celsius to use constants defined above
    sqrtD = sqrt(m_ObjectDistance);
    X  = m_X;
    a1 = m_alpha1;
    b1 = m_beta1;
    a2 = m_alpha2;
    b2 = m_beta2;

    if (T < TAO_TATM_MIN)
        T = TAO_TATM_MIN;
    else if (T > TAO_TATM_MAX)
        T = TAO_TATM_MAX;

    TT = T*T;

    sqrtH2O = sqrt(H*exp(H2O_K1 + H2O_K2*T + H2O_K3*TT + H2O_K4*TT*T));

    if (sqrtH2O > TAO_SQRTH2OMAX)
        sqrtH2O = TAO_SQRTH2OMAX;

    a1b1sqH2O = (a1+b1*sqrtH2O);
    a2b2sqH2O = (a2+b2*sqrtH2O);
    exp1 = exp(-sqrtD*a1b1sqH2O);
    exp2 = exp(-sqrtD*a2b2sqH2O);

    tao = X*exp1 + (1-X)*exp2;
    dtao = -(a1b1sqH2O*X*exp1+a2b2sqH2O*(1-X)*exp2);
    // The real D-derivative is also divided by 2 and sqrtD.
    // Here we only want the sign of the slope! */

    if (tao < TAO_COMP_MIN) {
        tao = TAO_COMP_MIN;      // below min value, clip
    } else if (tao > TAO_COMP_MAX) {
        // check tao at 1 000 000 m dist
        tao  = X*exp(-(1.0E3)*a1b1sqH2O)+(1.0-X)*exp(-(1.0E3)*a2b2sqH2O);

        if (tao > 1.0)    // above max, staying up, assume \/-shape
            tao = TAO_COMP_MIN;
        else
            tao = TAO_COMP_MAX;  // above max, going down, assume /\-shape
    } else if (dtao > 0.0 && m_ObjectDistance > 0.0) {
        tao = TAO_COMP_MIN;   // beween max & min, going up, assume \/
    }

    // else between max & min, going down => OK as it is, ;-)

    return( tao);
}

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
