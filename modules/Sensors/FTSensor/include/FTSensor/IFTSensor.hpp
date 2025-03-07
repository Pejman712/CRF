/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsotl Pasztori EN/SMM/MRO 2018
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include "CommonInterfaces/IInitializable.hpp"
#include "Types/TaskTypes/TaskForceTorque.hpp"
#include "Types/TaskTypes/TaskPose.hpp"

namespace crf {
namespace sensors {
namespace ftsensor {

class IFTSensor : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IFTSensor() = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /*
     * @brief If the sensor is calibrated the force-torque value is equal to FContact + FGravity,
     *        deleting the Bias and Noise components of the readings. If it is not calibrated the
     *        value is the raw force, equal to FContact + FBias + FNoise + FGravity.
     * @return The force-torque measurements of the sensor.
     */
    virtual crf::utility::types::TaskForceTorque getFT() = 0;

    /*
     * @brief The unfiltered value of the sensor, that might be important for calibration. It is
     *        equal to FContact + FBias + FNoise + FGravity, where:
     *          - FContact: the value actually needed.
     *          - FBias: low frequency.
     *          - FNoise: high frequency.
     *          - FGravity: noise due to the gravity force.
     * @return The raw force-torque measurements of the sensor.
     */
    virtual crf::utility::types::TaskForceTorque getRawFT() = 0;
    /*
     * @brief Gets the contact Force-Torque measurement FContact if the sensor is calibrated,
     *        cutting of the gravity.
     * @param sensorPosition: The pose of the sensor is fundamental to understand how to rotate the
     *        vector of force gravity in the right frame.
     * @param inWorldCoordinateSystem: true if you want the force in the world frame, false if you
     *        want it in the sensor frame.
     * @return The contact force-torque measurements of the sensor.
     */
    virtual crf::utility::types::TaskForceTorque getFTGravityFree(
        const crf::utility::types::TaskPose& sensorPosition,
        bool inWorldCoordinateSystem = false) = 0;
    /*
     * @brief Updates the Bias of the sensor based on the calibration file. If no previous
     *        calibration file is available it fails.
     */
    virtual bool updateBias(const crf::utility::types::TaskPose& sensorPosition,
        const std::string& logFilePath = "ftSensorCalibration.json") = 0;
    /*
     * @return Returns whether the sensor is calibrated for bias or not
     */
    virtual bool isCalibrated() = 0;
};

}  // namespace ftsensor
}  // namespace sensors
}  // namespace crf
