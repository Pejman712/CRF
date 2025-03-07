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

namespace crf::sensors::imu {

/**
 * @ingroup group_gable
 * @brief Technical and configuration infos of the device. In particular:
 *        (i) Baudrate between XSENS® IMU and microcontroller
 *        (ii) Major firmware version of the XSENS® IMU
 *        (iii) Major hardware version of the XSENS® IMU
 *        (iv) Active filter profile
 *        (v) XSENS® MTi-1 Device ID
 */
struct GableInfo {
    unsigned int Baudrate;
    int Firmware;
    int Hardware;
    crf::expected<unsigned int> FilterProfile;
    unsigned int DeviceID;
};

/**
 * @ingroup group_gable
 * @brief Available filter profiles
 *          - General: 50                       (Available for Gable ONE-SERIES SE2)
 *          - High magnetometer dependence: 51  (Available for Gable ONE-SERIES SE3)
 *          - Dynamic: 52                       (Available for Gable ONE-SERIES SE3)
 *          - North referenced: 53              (Available for Gable ONE-SERIES SE3)
 *          - VRU general: 54                   (Available for Gable ONE-SERIES SE2 and SE3)
 */
enum class FilterProfile {
    General = 50,
    HighMagnetometerDependence = 51,
    Dynamic = 52,
    NorthReferenced = 53,
    VRUGeneral = 54
};

}  // namespace crf::sensors::imu
