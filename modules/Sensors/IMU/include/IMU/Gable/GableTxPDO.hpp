/*
 * © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Henry Paul Espinosa Peralta CERN BE/CEM/MRO 2023
 *         Giancarlo D'Ago CERN BE/CEM/MRO 2023
 * 
 *  ==================================================================================================
 */

#pragma once

#include <stdint.h>

namespace crf::sensors::imu {

/**
 * @ingroup group_gable
 * @brief Output PDOs for IMU Gable ONESeries (SE1, SE2 and SE3)
 * 
 */
#pragma pack(push, 1)
struct GableTxPDO {
    uint8_t Data0;
    uint8_t Data1;
    uint8_t Data2;
    uint8_t Data3;
    uint8_t Data4;
    uint8_t Data5;
    uint8_t Data6;
    uint8_t Data7;
    uint8_t Data8;
    uint8_t Data9;
    uint8_t Data10;
    uint8_t Data11;
    uint8_t Data12;
    uint8_t Data13;
    uint8_t Data14;
    uint8_t Data15;
    uint8_t Data16;
    uint8_t Data17;
    uint8_t Data18;
    uint8_t Data19;
    uint8_t Data20;
    uint8_t Data21;
    uint8_t Data22;
    uint8_t Data23;
    uint8_t Data24;
    uint8_t Data25;
    uint8_t Data26;
    uint8_t Data27;
    uint8_t MID;
    uint8_t Command;
    uint8_t DataLength;
};
#pragma pack(pop)

}  // namespace crf::sensors::imu
