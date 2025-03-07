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
 * @ingroup group_gable_se2se3
 * @brief Input PDOs for IMU Gable ONESeries SE2 and SE3
 * 
 */
#pragma pack(push, 1)
struct GableRxPDO_SE2SE3 {
    float qw;
    float qx;
    float qy;
    float qz;
    float ex;
    float ey;
    float ez;
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float axHR;
    float ayHR;
    float azHR;
    float gxHR;
    float gyHR;
    float gzHR;
    float magX;
    float magY;
    float magZ;
    uint32_t CRC_Errors;
    uint8_t WakeUp;
    uint8_t CommandResponse;
    uint32_t CommunicationErrors;
    uint8_t Baudrate;
    int8_t Firmware;
    int8_t Hardware;
    uint8_t FilterProfile;
    uint32_t DeviceID;
};
#pragma pack(pop)

}  // namespace crf::sensors::imu
